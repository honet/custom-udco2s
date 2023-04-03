int16_t SCD30StartPeriodicMeasurement(uint16_t ambient_pressure_mbar);
bool SCD30StopPeriodicMeasurement();

bool SCD30SetMeasurementInterval(uint16_t interval_sec);
uint16_t SCD30GetMeasurementInterval();

uint16_t SCD30ReadDataReadyStatus();
bool SCD30ReadMeasurement(float *co2, float *temperature, float *humidity);

bool SCD30SetASC(uint8_t enable_asc);
int16_t SCD30GetASC();

bool SCD30SetForcedRecalibrationValue(uint16_t co2_ppm);
uint16_t SCD30GetForcedRecalibrationValue();

bool SCD30SetTemperatureOffset(uint16_t temperature_offset);
uint16_t SCD30GetTemperatureOffset();

bool SCD30SetAltitude(uint16_t altitude);
int16_t SCD30GetAltitude();

uint16_t SCD30ReadFWVer();
bool SCD30SoftReset();
