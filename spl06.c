/***************************************************************************
 *
 *  Copyright (c) 2019 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file spl06_dma.c
 *
 * @author zwh <zwh@raaworks.com>
 *
 * @data 2019.10.13
 */

#include "spl06.h"

#include <stdint.h>
#include <math.h>

#include "scheduler.h"
#include "topics.h"

static char path[4] = { 'i', 'h', 0x76 };   // hardware i2c

struct {
    struct {
        int16_t c0, c1;
        int32_t c00, c10;
        int16_t c01, c11, c20, c21, c30;
    } coe;

    float kt;
    float kp;

    int32_t press_raw;
    int32_t temp_raw;

    uint32_t t_peroid;  // measurement period
    uint64_t t_ready;   // measurement finished

    int inited;

} _spl;

static float scale_factor(int oversampling_rate)
{
    float k;

    switch (oversampling_rate) {
        case 1:
            k = 524288.0f;
        break;

        case 2:
            k = 1572864.0f;
        break;

        case 4:
            k = 3670016.0f;
        break;

        case 8:
            k = 7864320.0f;
        break;

        case 16:
            k = 253952.0f;
        break;

        case 32:
            k = 516096.0f;
        break;

        case 64:
            k = 1040384.0f;
        break;

        case 128:
            k = 2088960.0f;
        break;
    }

    return k;
}

static int read_reg(uint8_t reg, uint8_t *buf, int len)
{
    path[3] = reg;

    return read(path, buf, len);
}

static uint8_t read_reg_single(uint8_t reg)
{
    uint8_t val = 0;

    read_reg(reg, &val, 1);

    return val;
}

static int write_reg(uint8_t reg, uint8_t val)
{
    path[3] = reg;

    return write(path, &val, 1);
}

static int write_reg_check(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { val, ~val };

    usleep(1000);

    write_reg(reg, data[0]);

    usleep(1000);

    read_reg(reg, &data[1], 1);

    if (data[0] != data[1]) {
        return -1;
    }

    return 0;
}

static void init()
{
    // try to start device
    uint8_t tries = 5;

    while (tries--) {

        // check chip ID
        if (read_reg_single(0x0d) == 0x10) {

            uint8_t meas_cfg = read_reg_single(0x08);

            if (meas_cfg & (1 << 7) && meas_cfg & (1 << 6)) {
                break;
            }
        }

        usleep(1000);
    }

    if (!tries) {
        return;
    }

    uint8_t buf[18] = { 0 };

    read_reg(0x10, buf, sizeof(buf));

    _spl.coe.c0 = (uint16_t)buf[0] << 4 | (uint16_t)buf[1] >> 4;
    _spl.coe.c0 = (_spl.coe.c0 & 1 << 11) ? (0xf000 | _spl.coe.c0) : _spl.coe.c0;

    _spl.coe.c1 = (uint16_t)(buf[1] & 0x0f) << 8 | (uint16_t)buf[2];
    _spl.coe.c1 = (_spl.coe.c1 & 1 << 11) ? (0xf000 | _spl.coe.c1) : _spl.coe.c1;

    _spl.coe.c00 = (uint32_t)buf[3] << 12 | (uint32_t)buf[4] << 4 | (uint16_t)buf[5] >> 4;
    _spl.coe.c00 = (_spl.coe.c00 & 1 << 19) ? (0xfff00000 | _spl.coe.c00) : _spl.coe.c00;

    _spl.coe.c10 = (uint32_t)(buf[5] & 0x0f) << 16 | (uint32_t)buf[6] << 8 | (uint32_t)buf[7];
    _spl.coe.c10 = (_spl.coe.c10 & 1 << 19) ? (0xfff00000 | _spl.coe.c10) : _spl.coe.c10;

    _spl.coe.c01 = (uint16_t)buf[8] << 8 | buf[9];
    _spl.coe.c11 = (uint16_t)buf[10] << 8 | buf[11];
    _spl.coe.c20 = (uint16_t)buf[12] << 8 | buf[13];
    _spl.coe.c21 = (uint16_t)buf[14] << 8 | buf[15];
    _spl.coe.c30 = (uint16_t)buf[16] << 8 | buf[17];

    int state = 0;

    // compensation scale factors
    //
    // oversampling rate  : single | 2       | 4       | 8       | 16     | 32     | 64      | 128
    // scale factor(KP/KT): 524288 | 1572864 | 3670016 | 7864320 | 253952 | 516096 | 1040384 | 2088960

    // configuration of pressure measurement rate (PM_RATE) and resolution (PM_PRC)
    //
    // bit[7]: reserved
    //
    // PM_RATE[6:4]    : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: applicable for measurements in background mode only
    //
    // PM_PRC[3:0]         : 0      | 1   | 2   | 3    | 4    | 5    | 6     | 7
    // oversampling (times): single | 2   | 4   | 8    | 16   | 32   | 64    | 128
    // measurement time(ms): 3.6    | 5.2 | 8.4 | 14.8 | 27.6 | 53.2 | 104.4 | 206.8
    // precision(PaRMS)    : 5.0    |     | 2.5 |      | 1.2  | 0.9  | 0.5   |
    // note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
    state += write_reg_check(0x06, 4 << 4 | 4);
    _spl.kp = scale_factor(16);

    // configuration of temperature measurment rate (TMP_RATE) and resolution (TMP_PRC)
    //
    // temperature measurement: internal sensor (in ASIC) | external sensor (in pressure sensor MEMS element)
    // TMP_EXT[7]             : 0                         | 1
    // note: it is highly recommended to use the same temperature sensor as the source of the calibration coefficients wihch can be read from reg 0x28
    //
    // TMP_RATE[6:4]   : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: applicable for measurements in background mode only
    //
    // bit[3]: reserved
    //
    // TMP_PRC[2:0]        : 0      | 1 | 2 | 3 | 4  | 5  | 6  | 7
    // oversampling (times): single | 2 | 4 | 8 | 16 | 32 | 64 | 128
    // note: single(default) measurement time 3.6ms, other settings are optional, and may not be relevant
    // note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
    state += write_reg_check(0x07, 1 << 7 | 4 << 4 | 0);
    _spl.kt = scale_factor(1);

    // diasble all interrupts and FIFO
    //
    // bit7: interrupt(on SDO pin) active level
    //       0-active low  1-active hight
    //
    // bit6: set to '1' for FIFO full interrupt
    // bit5: set to '1' for pressure measurement ready interrupt
    // bit4: set to '1' for temperature measurement ready interrupt
    //
    // note: bit3 must be set to '1' when the temperature oversampling rate is > 8 times
    // note: bit2 must be set to '1' when the pressure oversampling rate is > 8 times
    //
    // bit1: set to '1' for FIFO enable
    //
    // bit0: set to '0' for 4-wire interface
    //       set to '1' for 3-wire interface
    //
    state += write_reg_check(0x09, 1 << 2);

    // sensor operating mode and status
    //
    // COEF_RDY[7]: 0-coefficients are not available yet;  1-coefficients are available
    //
    // SENSOR_RDY[6]: 0-sensor initialization not complete;  1-sensor initialization complete
    //                it is recommend not to start measurements until the sensor has complete the self initialization
    //
    // TMP_RDY[5]: 1-new temperature measurement is ready. cleared when temperature measurement is read
    // PRS_RDY[4]: 1-new pressure measurement is ready. cleared when procurement measurement is read.
    //
    // measurement mode: stop meas |  command mode(single) |    na |            background mode(continuous) |
    // measurement type:      idle | pres meas | temp meas |    na | pres meas | temp meas | pres&temp meas |
    // MEAS_CTRL[2:0]  :         0 |         1 |         2 | 3 | 4 |         5 |         6 |              7 |

    write_reg(0x08, 7);

    _spl.t_peroid = 1e6 / 16;
    _spl.t_ready = time() + 27600 + 3600 + 1000;

    if (!state) {
        _spl.inited = 1;
    }
}

static uint8_t _dma_reading;

void spl06_dma_request()
{
    if (!_spl.inited) {
        init();
        return;
    }

    uint64_t t = time();

    if (!_dma_reading) {
        if (t > _spl.t_ready) {
            _spl.t_ready += _spl.t_peroid;

            if (_spl.t_ready < t) {
                _spl.t_ready = t;
            }

            path[1] = 'r';
            path[3] = 0x00;     // read qmc sensor values

            read(path, 0, 6);

            _dma_reading = 1;
        }

    } else if (t > _spl.t_ready + 1e5) {
        // timeout
        _dma_reading = 0;
    }
}

void spl06_dma_process()
{
    uint64_t t = time();

    if (_dma_reading) {
        // copy sensors values
        uint8_t buffer[6] = { 0 };

        path[1] = 'c';

        if (read(path, buffer, 6) == 0) {

            _dma_reading = 0;

            _spl.temp_raw = (uint32_t)buffer[3] << 16 | (uint32_t)buffer[4] << 8 | (uint32_t)buffer[5];
            _spl.temp_raw = (_spl.temp_raw & 1 << 23) ? (0xff000000 | _spl.temp_raw) : _spl.temp_raw;

            _spl.press_raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
            _spl.press_raw = (_spl.press_raw & 1 << 23) ? (0xff000000 | _spl.press_raw) : _spl.press_raw;

            // calculate
            float ftsc = (float)_spl.temp_raw / _spl.kt;
            float fpsc = (float)_spl.press_raw / _spl.kp;
            float qua2 = (float)_spl.coe.c10 + fpsc * ((float)_spl.coe.c20 + fpsc * (float)_spl.coe.c30);
            float qua3 = ftsc * fpsc * ((float)_spl.coe.c11 + fpsc * (float)_spl.coe.c21);

            float fp = (float)_spl.coe.c00 + fpsc * qua2 + ftsc * (float)_spl.coe.c01 + qua3;

            // altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1

            // tropospheric properties (0-11km) for standard atmosphere
            const float T1 = 15.0f + 273.15f;       // temperature at base height in Kelvin
            const float a  = -6.5f / 1000.0f;       // temperature gradient in degrees per metre
            const float g  = 9.80665f;              // gravity constant in m/s/s
            const float R  = 287.05f;               // ideal gas constant in J/kg/K
            const float msl_pressure = 101325.0f;   // in Pa
            float pK = fp / msl_pressure;

            /*
             * Solve:
             *
             *     /        -(aR / g)     \
             *    | (p / p1)          . T1 | - T1
             *     \                      /
             * h = -------------------------------  + h1
             *                   a
             */

            _baro.timestamp = t;
            _baro.temperature = (float)_spl.coe.c0 * 0.5f + (float)_spl.coe.c1 * ftsc;
            _baro.pressure = fp;
            _baro.altitude = (((powf(pK, (-(a * R) / g))) * T1) - T1) / a;
        }
    }
}
