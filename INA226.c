#include "INA226.h"

// I2C registers
#define INA226_REG_CONFIGURATION 0x00
#define INA226_REG_SHUNT_VOLTAGE 0x01
#define INA226_REG_BUS_VOLTAGE   0x02
#define INA226_REG_POWER         0x03
#define INA226_REG_CURRENT       0x04
#define INA226_REG_CALIBRATION   0x05
#define INA226_REG_MASK_ENABLE   0x06
#define INA226_REG_ALERT_LIMIT   0x07
#define INA226_REG_MANUFACTURER  0xFE
#define INA226_REG_DIE_ID        0xFF

#define I2C_READ_TIMEOUT_MS 50

static const struct INA226_Platform *platform;

static float Current_LSB = 0.001; // default value 1 mA, will be overwritten by calibration set

void INA226_Init(const struct INA226_Platform *platformPtr) {
    platform = platformPtr;
}

bool INA226_Reset(void) {
    int ret;

    // reset INA using configuration register
    // note: set only reset bit (bit 15)
    // datasheet: Setting this bit to '1' generates a system reset that is the same as power-on reset. Resets all registers to default values; this bit self-clears.
    uint8_t config_reg[2] = { 0x80, 0x00 };
    ret = platform->i2cWriteReg(platform->i2cAddress, INA226_REG_CONFIGURATION, config_reg, sizeof(config_reg), 1);
    if (ret < 0) {
        platform->debugPrint("Error writing configuration register to reset INA: %d\r\n", -ret);
        return false;
    }

    return true;
}

bool INA226_ReadManufacturerId(uint16_t *id) {
    int ret;

    // read manufacturer ID
    uint8_t man_id_reg[2];
    ret = platform->i2cReadReg(platform->i2cAddress, INA226_REG_MANUFACTURER, man_id_reg, sizeof(man_id_reg), 1);
    if (ret < 0) {
        platform->debugPrint("Error reading manufacturer ID: %d\r\n", -ret);
        return false;
    }

    *id = (man_id_reg[0] << 8) | man_id_reg[1];
    return true;
}

bool INA226_ReadDieId(uint16_t *id) {
    int ret;

    // read die ID
    uint8_t die_id_reg[2];
    ret = platform->i2cReadReg(platform->i2cAddress, INA226_REG_DIE_ID, die_id_reg, sizeof(die_id_reg), 1);
    if (ret < 0) {
        platform->debugPrint("Error reading die ID: %d\r\n", -ret);
        return false;
    }

    *id = (die_id_reg[0] << 8) | die_id_reg[1];
    return true;
}

bool INA226_SetCalibration(float max_expected_current, float r_shunt) {
    int err;

    // calculate calibration (see datasheet)
    float Current_LSB_base = max_expected_current / 32768.0;
    float CAL_float = 0.00512 / Current_LSB_base / r_shunt;
    uint16_t CAL = (uint16_t)(CAL_float);

    // calculate current LSB from actual rounded CAL value
    Current_LSB = 0.00512 / (float)CAL / r_shunt;

    // write calibration
    uint8_t cal_reg[2] = { (uint8_t)((CAL >> 8) & 0x7F), (uint8_t)(CAL & 0xFF) };
    err = platform->i2cWriteReg(platform->i2cAddress, INA226_REG_CALIBRATION, cal_reg, sizeof(cal_reg), 1);
    if (err < 0) {
        platform->debugPrint("Error writing calibration: %d\r\n", -err);
        return false;
    }

    platform->debugPrint("INA Calibration: %u (Current LSB: %.6f)\r\n", CAL, Current_LSB);
    return true;
}

bool INA226_SetConfiguration(enum INA226_AvgMode avgMode, enum INA226_ConvTime convTime, enum INA226_OpMode opMode) {
    int err;

    // calculate the value of configuration register
    uint16_t config = 0;
    config |= (uint16_t)avgMode << 9; // AVG D9-D11
    config |= (uint16_t)convTime << 6; // VBUSCT D6-D8
    config |= (uint16_t)convTime << 3; // VSHCT D3-D5
    config |= (uint16_t)opMode; // MODE D0-D2

    // write configuration register
    uint8_t config_reg[2] = { (uint8_t)(config >> 8), (uint8_t)(config & 0xFF) };
    err = platform->i2cWriteReg(platform->i2cAddress, INA226_REG_CONFIGURATION, config_reg, sizeof(config_reg), 1);
    if (err < 0) {
        platform->debugPrint("Error writing configuration: %d\r\n", -err);
        return false;
    }

    platform->debugPrint("INA Configuration: 0x%04X\r\n", config);
    return true;
}

bool INA226_ReadVoltage(float *voltage) {
    int err;

    // read bus voltage register
    uint8_t bus_voltage_reg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA226_REG_BUS_VOLTAGE, bus_voltage_reg, sizeof(bus_voltage_reg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading bus voltage: %d\r\n", -err);
        *voltage = -1.0;
        return false;
    }

    // calculate voltage
    const float LSB = 0.00125; // 1.25 mV
    uint16_t bus_voltage = (bus_voltage_reg[0] << 8) | bus_voltage_reg[1];
    float voltage_float = (float)(bus_voltage)*LSB;

    *voltage = voltage_float;
    return true;
}

bool INA226_ReadCurrent(float *current) {
    int err;

    // read current register
    uint8_t current_reg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA226_REG_CURRENT, current_reg, sizeof(current_reg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading current: %d\r\n", -err);
        *current = -1.0;
        return false;
    }

    // calculate current
    // NOTE: encoding is two's complement (notwithstanding the datasheet doesn't specify explicitly for this field)
    uint16_t raw_current = (current_reg[0] << 8) | current_reg[1];
    int16_t current_signed = *(int16_t*)(&raw_current);
    float current_float = (float)current_signed * Current_LSB;
   
    *current = current_float;
    return true;
}

bool INA226_ReadPower(float *power) {
    int err;

    // read power register
    uint8_t power_reg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA226_REG_POWER, power_reg, sizeof(power_reg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading power: %d\r\n", -err);
        *power = -1.0;
        return false;
    }

    // calculate power
    uint16_t raw_power = (power_reg[0] << 8) | power_reg[1];
    float Power_LSB = 25.0f * Current_LSB; // see datasheet
    float power_float = (float)raw_power * Power_LSB;
    
    *power = power_float;
    return true;
}
