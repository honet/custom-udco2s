#include <limits.h>

#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/clock.h"

#define FCY (_XTAL_FREQ/2)

#include <libpic30.h>
#include <stdio.h>
#include <string.h>

#include "scd30.h"

#define I2C_RETRY_MAX           100
#define I2C_DEVICE_TIMEOUT      150   // define slave timeout 
#define SCD30ADDR 0x61

#define SCD30_WRITEDELAY 5

static uint8_t CalcCRC(uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for(int i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; bit++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static float toValue(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3){
    uint32_t ival = (((uint32_t)d0) << 24) |
                    (((uint32_t)d1) << 16) |
                    (((uint32_t)d2) << 8) | d3;

    return *(float*)&ival;
}

I2C1_MESSAGE_STATUS SCD30Send(const uint8_t *data, int len)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

    // Now it is possible that the slave device will be slow.
    // As a work around on these slaves, the application can
    // retry sending the transaction
    uint16_t retryTimeOut = 0;
    uint16_t slaveTimeOut = 0;

    while (status != I2C1_MESSAGE_FAIL) {
        I2C1_MasterWrite((uint8_t*)data, len, SCD30ADDR, &status);

        // wait for the message to be sent or status has changed.
        while (status == I2C1_MESSAGE_PENDING) {
            // add some delay here
            __delay_ms(1);

            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == I2C_DEVICE_TIMEOUT){
                break;
            } else {
                slaveTimeOut++;
            }
        } 
        if ((slaveTimeOut == I2C_DEVICE_TIMEOUT) || 
            (status == I2C1_MESSAGE_COMPLETE))
            break;

        // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK, or I2C1_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (retryTimeOut == I2C_RETRY_MAX) { 
            break;
        } else {
            retryTimeOut++;
        }
    }

    return status;
}

I2C1_MESSAGE_STATUS SCD30Read(uint8_t *rbuff, int len){

    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

    // this portion will read the byte from the memory location.
    uint16_t retryTimeOut = 0;
    uint16_t slaveTimeOut = 0;

    while(status != I2C1_MESSAGE_FAIL) {
        I2C1_MasterRead(rbuff, len, SCD30ADDR, &status);

        // wait for the message to be sent or status has changed.
        while (status == I2C1_MESSAGE_PENDING) {
            // add some delay here
            __delay_ms(1);

            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == I2C_DEVICE_TIMEOUT) {
                return (0);
            } else {
                slaveTimeOut++;
            }
        }

        if (status == I2C1_MESSAGE_COMPLETE)
            break;

        // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK, or I2C1_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (retryTimeOut == I2C_RETRY_MAX)
            break;
        else
            retryTimeOut++;
    }

    return status;
}

uint16_t SCD30ReadWordValue(uint8_t cmd0, uint8_t cmd1){
    const uint8_t data[2] = { cmd0, cmd1 };
    I2C1_MESSAGE_STATUS status = SCD30Send(data, 2);

    uint8_t rbuff[4] = {0};
    if (status == I2C1_MESSAGE_COMPLETE){
        __delay_ms(3);
        SCD30Read(rbuff, 3);

        uint8_t crc = CalcCRC(rbuff, 2);
        if (crc == rbuff[2]){
            return (uint16_t)rbuff[0] << 8 | rbuff[1];
        }
    }
    return 0xffff;
}

bool SCD30WriteWordValue(uint8_t cmd0, uint8_t cmd1, uint16_t value){
    uint8_t data[5] = { cmd0, cmd1, 0};
    data[2] = (value >> 8) & 0xff;
    data[3] = value & 0xff;
    data[4] = CalcCRC(&data[2], 2);
    I2C1_MESSAGE_STATUS status = SCD30Send(data, 5);
    __delay_ms(SCD30_WRITEDELAY);
    return status == I2C1_MESSAGE_COMPLETE;
}


// Trigger continuous measurement with optional ambient pressure compensation
int16_t SCD30StartPeriodicMeasurement(uint16_t ambient_pressure_mbar) {
    if (ambient_pressure_mbar &&
        (ambient_pressure_mbar < 700 || ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return 0xffff;
    }
    return SCD30WriteWordValue(0x00, 0x10, ambient_pressure_mbar);
}

// Stop continuous measurement
bool SCD30StopPeriodicMeasurement() {
    const uint8_t data[2] = { 0x01, 0x04 };
    I2C1_MESSAGE_STATUS status = SCD30Send(data, 2);
    return status == I2C1_MESSAGE_COMPLETE;
}

// Set measurement interval
bool SCD30SetMeasurementInterval(uint16_t interval_sec) {
    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return false;
    }
    return SCD30WriteWordValue(0x46, 0x00, interval_sec);
}

// Get measurement interval
uint16_t SCD30GetMeasurementInterval() {
    return SCD30ReadWordValue(0x46, 0x00);
}


// Get data ready status
uint16_t SCD30ReadDataReadyStatus() {
    return SCD30ReadWordValue(0x02, 0x02);
}

// Read measurement
bool SCD30ReadMeasurement(float *co2, float *temperature, float *humidity) {
    const uint8_t data[2] = { 0x03, 0x00 };
    I2C1_MESSAGE_STATUS status = SCD30Send(data, 2);

    uint8_t rbuff[32] = {0};
    if (status == I2C1_MESSAGE_COMPLETE){
        __delay_ms(3);
        SCD30Read(rbuff, 18);

        for (int i=0 ;i<6; i++){
            uint8_t crc = CalcCRC(&rbuff[i*3], 2);
            if (crc != rbuff[i*3+2])
                return false;
        }

        *co2 = toValue(rbuff[0], rbuff[1], rbuff[3], rbuff[4]);
        *temperature = toValue(rbuff[6], rbuff[7], rbuff[9], rbuff[10]);
        *humidity = toValue(rbuff[12], rbuff[13], rbuff[15], rbuff[16]);
    } else {
        return false;
    }

    return true;
}

// (De-)Activate continuous calculation of reference value for automatic self-calibration (ASC)
bool SCD30SetASC(uint8_t enable_asc) {
    uint16_t asc = !!enable_asc;
    return SCD30WriteWordValue(0x53, 0x06, asc);
}

int16_t SCD30GetASC() {
    return SCD30ReadWordValue(0x53, 0x06);
}

// Set external reference value for forced recalibration (FRC)
bool SCD30SetForcedRecalibrationValue(uint16_t co2_ppm) {
    return SCD30WriteWordValue(0x52, 0x04, co2_ppm);
}

// Get external reference value for forced recalibration (FRC)
uint16_t SCD30GetForcedRecalibrationValue() {
    return SCD30ReadWordValue(0x52, 0x04);
}

// Set temperature offset for onboard RH/T sensor
bool SCD30SetTemperatureOffset(uint16_t temperature_offset) {
    return SCD30WriteWordValue(0x54, 0x03, temperature_offset);
}

// Get temperature offset for onboard RH/T sensor
uint16_t SCD30GetTemperatureOffset() {
    return SCD30ReadWordValue(0x54, 0x03);
}

// Set Altitude compensation
bool SCD30SetAltitude(uint16_t altitude) {
    return SCD30WriteWordValue(0x51, 0x02, altitude);
}

// Get Altitude compensation
int16_t SCD30GetAltitude() {
    return SCD30ReadWordValue(0x51, 0x02);
}

// Read firmware version
uint16_t SCD30ReadFWVer() {
    return SCD30ReadWordValue(0xd1, 0x00);
}

// Soft reset
bool SCD30SoftReset() {
    const uint8_t data[2] = { 0xd3, 0x04 };
    I2C1_MESSAGE_STATUS status = SCD30Send(data, 2);
    __delay_ms(SCD30_WRITEDELAY);
    return status == I2C1_MESSAGE_COMPLETE;
}


