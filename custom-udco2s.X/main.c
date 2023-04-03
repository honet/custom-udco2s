/**
  @Summary
    Custom Firmware for UD-CO2S

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.1
        Device            :  PIC24FJ64GB202
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.70
        MPLAB 	          :  MPLAB X v5.50
*/
#include <limits.h>

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/usb/usb.h"
#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/tmr1.h"
#include "mcc_generated_files/clock.h"
#include "mcc_generated_files/oc1.h"
#include "mcc_generated_files/oc2.h"
#include "mcc_generated_files/oc3.h"

#define FCY (_XTAL_FREQ/2)

#include <libpic30.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "scd30.h"

static uint32_t systemCounter = 0;

#define TXBUFFLEN 1024
#define RXBUFFLEN 256
static uint8_t cdcReadBuffer[64];
static uint8_t cdcSendBuffer[TXBUFFLEN];
static uint8_t txBuffer[TXBUFFLEN];
static int txIdx = 0;
static uint8_t rxBuffer[RXBUFFLEN];
static int rxIdx = 0;

void CDC_Send(uint8_t *data, int len){
    if (txIdx + len >= TXBUFFLEN) return; // overflow

    memcpy(txBuffer + txIdx, data, len);
    txIdx  += len;
}

void CDC_SendStr(char *str){
    uint8_t *data = (uint8_t*)str;
    int len = strlen(str);
    if (txIdx + len >= TXBUFFLEN) return; // overflow

    memcpy(txBuffer + txIdx, data, len);
    txIdx  += len;
}

bool ParseValue(char *p, int *value){
    while (*p == ' ') p++;
    if (strlen(p)==0){
        return false;
    } else {
        *value = atoi(p);
        return true;
    }
}

void OKNG(bool ret){
    if (ret){
        CDC_SendStr("OK\r\n");
    } else {
        CDC_SendStr("NG\r\n");
    }
}

void upperstr(char *s){
    while(*s != '\0') {
        *s = toupper(*s);
        s++;
    }
}

void ReadASC(){
    char msgbuff[128];
    int value = SCD30GetASC();
    sprintf(msgbuff, "ASC = %d\r\n", value);
    CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
}

void ReadFRC(){
    char msgbuff[128];
    uint16_t value = SCD30GetForcedRecalibrationValue();
    sprintf(msgbuff, "FRC = %d\r\n", value);
    CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
}

void ReadTOffset(){
    char msgbuff[128];
    uint16_t offset = SCD30GetTemperatureOffset();
    sprintf(msgbuff, "TEMP OFFSET = %d\r\n", offset);
    CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
}

void ReadAltitude(){
    char msgbuff[128];
    uint16_t alt = SCD30GetAltitude();
    sprintf(msgbuff, "ALT = %d\r\n", alt);
    CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
}

void DoCommand(char *str){
    int value;
    char msgbuff[256];
    
    upperstr(str);
    
    if (strncmp(str, "STA", 3) == 0) {
        OKNG(SCD30StartPeriodicMeasurement(0));
    }
    else if (strncmp(str, "STP", 3) == 0) {
        OKNG(SCD30StopPeriodicMeasurement());
    }
    else if (strncmp(str, "ID?", 3) == 0) {
        CDC_SendStr("OK ID=UD-CO2S-MOD\r\n");
    }
    else if (strncmp(str, "PARAM?", 6) == 0) {
        uint16_t ver = SCD30ReadFWVer();
        sprintf(msgbuff, "VER 0x%04x\r\n", ver);
        CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));

        ReadASC();
        ReadFRC();
        ReadTOffset();
        ReadAltitude();

        uint16_t interval = SCD30GetMeasurementInterval();
        sprintf(msgbuff, "Interval = %d\r\n", interval);
        CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
    }
    else if (strncmp(str, "FRC=", 4) == 0) {
        if (ParseValue(&str[4], &value)){
            OKNG(SCD30SetForcedRecalibrationValue((uint16_t)value));
        } else {
            ReadFRC();
        }
    }
    else if (strncmp(str, "FRC?", 4) == 0) {
        ReadFRC();
    }
    else if (strncmp(str, "ASC=", 4) == 0) {
        if (ParseValue(&str[4], &value)){
            OKNG(SCD30SetASC(value ? 1 : 0));
        } else {
            ReadASC();
        }
    }
    else if (strncmp(str, "ASC?", 4) == 0) {
        ReadASC();
    }
    else if (strncmp(str, "TOFFSET=", 8) == 0) {
        if (ParseValue(&str[8], &value)){
            OKNG(SCD30SetTemperatureOffset((uint16_t)value));
        } else {
            ReadTOffset();
        }
    }
    else if (strncmp(str, "TOFFSET?", 8) == 0) {
        ReadTOffset();
    }
    else if (strncmp(str, "ALT=", 4) == 0) {
        if (ParseValue(&str[4], &value)){
            OKNG(SCD30SetAltitude((uint16_t)value));
        } else {
            ReadAltitude();
        }
    }
    else if (strncmp(str, "ALT?", 4) == 0) {
        ReadAltitude();
    }
}

void USB_CDC_Task(void)
{
    if (USBGetDeviceState() < CONFIGURED_STATE) {
        return;
    }
    if (USBIsDeviceSuspended()== true) {
        return;
    }

    uint8_t nread = getsUSBUSART(cdcReadBuffer, sizeof(cdcReadBuffer));
    for (int i=0; i<nread; i++){
        switch(cdcReadBuffer[i]){
            case 0:
            case 0x0A:
            case 0x0D:
                rxBuffer[rxIdx] = 0;
                DoCommand((char*)rxBuffer);
                rxIdx = 0;
                break;

            default:
                rxBuffer[rxIdx++] = cdcReadBuffer[i];
                if (rxIdx >= RXBUFFLEN-1){
                    rxIdx = 0; // overflow. just drop
                }
                break;
        }
    }

    if (USBUSARTIsTxTrfReady() == true) {
        if (0<txIdx) {
            memcpy(cdcSendBuffer, txBuffer, txIdx);
            putUSBUSART(cdcSendBuffer, txIdx);
            txIdx = 0;
        }
    }

    CDCTxService();
}

void TMR1_CallBack(void){
    systemCounter++;
}

void SetRGBLED(int r, int g, int b){
    OC1_PrimaryValueSet(255-r);
    OC2_PrimaryValueSet(255-g);
    OC3_PrimaryValueSet(255-b);
}

// HSV(HSB)色空間からRGB色空間へ変換する 
//  h(hue)       : 色相/色合い   0~360
//  s(saturation): 彩度/鮮やかさ 0~100
//  v(Value)     : 明度/明るさ   0~100
void hsv2rgb(int h, int s, int v, int *r, int *g, int *b){
    h = h%360;
    float fs = s / 100.0f;
    float fv = v * 255.0f / 100.0f;
   
    if (s == 0) {
        *r = fv;
        *g = fv;
        *b = fv;
        return;
    }

    int h2 = (h/60);
    float dh = (h / 60.0f) - h2;
    float p = fv * (1.0f - fs);
    float q = fv * (1.0f - fs * dh);
    float t = fv * (1.0f - fs * (1.0f - dh));
   
    switch (h2) {
        case 0: *r = fv; *g = t;  *b = p; break;
        case 1: *r = q;  *g = fv; *b = p; break;
        case 2: *r = p;  *g = fv; *b = t; break;
        case 3: *r = p;  *g = q;  *b = fv; break;
        case 4: *r = t;  *g = p;  *b = fv; break;
        case 5: *r = fv; *g = p;  *b = q;  break;
    }
    return; 
}
 
//  Main application
int main(void)
{
    char msgbuff[256];
    float co2 = 0;
    int co2i = 0;
    float temperature = 0;
    float humidity = 0;
    uint32_t nextCount = 0;

    // initialize the device
    SYSTEM_Initialize();
    TMR1_SetInterruptHandler(TMR1_CallBack);

    IO_SCD30PWR_SetLow();
    __delay_ms(1000);
    IO_SCD30PWR_SetHigh();

    SCD30StopPeriodicMeasurement();
    
    for (;;) {
        // Add your application code
        if (nextCount < systemCounter){
            nextCount += 1000;

            if (SCD30ReadDataReadyStatus()){
                __delay_ms(3);
                
                SCD30ReadMeasurement(&co2, &temperature, &humidity);
                co2i = (int)(co2+0.5f);
                sprintf(msgbuff, "CO2=%d,HUM=%0.1f,TEMP=%0.1f\r\n", co2i, humidity, temperature);
                CDC_Send((uint8_t*)msgbuff, strlen(msgbuff));
            }
        }
        USB_CDC_Task();

        if (co2i <= 1000){
            SetRGBLED(0, 0, 255); // blue
        } else if (co2i <= 1500) {
            SetRGBLED(0, 255, 0); // green
        } else if (co2i <= 2500) {
            SetRGBLED(255, 255, 0); // yellow
        } else if (co2i <= 3500) {
            SetRGBLED(255, 165, 0); // orange
        } else {
            SetRGBLED(170, 0, 255); // purple
            
        }

        //uint32_t c = (systemCounter/3) % 360;
        //int r, g, b;
        //hsv2rgb(c, 100, 100, &r, &g, &b);
        //SetRGBLED(r, g, b);
    }

    return 1;
}
