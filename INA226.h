#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>

#define INA226_I2C_ADDR_DEF 0b1000000u // I2C slave address (default)

enum INA226_AvgMode {
    INA226_AVG_MODE_1 = 0b000,
    INA226_AVG_MODE_4 = 0b001,
    INA226_AVG_MODE_16 = 0b010,
    INA226_AVG_MODE_64 = 0b011,
    INA226_AVG_MODE_128 = 0b100,
    INA226_AVG_MODE_256 = 0b101,
    INA226_AVG_MODE_512 = 0b110,
    INA226_AVG_MODE_1024 = 0b111
};

enum INA226_ConvTime {
    INA226_CONV_TIME_140US = 0b000,
    INA226_CONV_TIME_204US = 0b001,
    INA226_CONV_TIME_332US = 0b010,
    INA226_CONV_TIME_588US = 0b011,
    INA226_CONV_TIME_1_1MS = 0b100,
    INA226_CONV_TIME_2_116MS = 0b101,
    INA226_CONV_TIME_4_156MS = 0b110,
    INA226_CONV_TIME_8_244MS = 0b111
};

enum INA226_OpMode {
    INA226_OP_MODE_POWER_DOWN = 0b000,
    INA226_OP_MODE_SHUNT_V_TRIG = 0b001,
    INA226_OP_MODE_BUS_V_TRIG = 0b010,
    INA226_OP_MODE_SHUNT_AND_BUS_TRIG = 0b011,
    INA226_OP_MODE_SHUNT_V_CONT = 0b101,
    INA226_OP_MODE_BUS_V_CONT = 0b110,
    INA226_OP_MODE_SHUNT_AND_BUS_CONT = 0b111
};

#define INA226_MANUFACTURER_ID 0x5449
#define INA226_DIE_ID 0x2260

struct INA226_Platform {
    int (*i2cWriteReg)(uint8_t addr7bit, uint8_t regNum, const uint8_t *data, uint8_t length, uint8_t wait);
    int (*i2cReadReg)(uint8_t addr7bit, uint8_t regNum, uint8_t *data, uint8_t length, int timeout);
    void (*debugPrint)(const char *fmt, ...);

    uint8_t i2cAddress;
};

void INA226_Init(const struct INA226_Platform *platform);

bool INA226_Reset(void);
bool INA226_ReadManufacturerId(uint16_t *id);
bool INA226_ReadDieId(uint16_t *id);

bool INA226_SetCalibration(float max_expected_current, float r_shunt);
bool INA226_SetConfiguration(enum INA226_AvgMode avgMode, enum INA226_ConvTime convTime, enum INA226_OpMode opMode);

bool INA226_ReadVoltage(float *voltage);
bool INA226_ReadCurrent(float *current);
bool INA226_ReadPower(float *power);

#endif // INA226_H
