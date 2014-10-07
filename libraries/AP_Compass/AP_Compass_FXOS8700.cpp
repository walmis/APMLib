/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_FXOS8700.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_FXOS8700.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS 0x1E

#define FXOS8700_STATUS 		0x00
#define FXOS8700_OUT_X_MSB 		0x01
#define FXOS8700_OUT_X_LSB 		0x02
#define FXOS8700_OUT_Y_MSB 		0x03
#define FXOS8700_OUT_Y_LSB 		0x04
#define FXOS8700_OUT_Z_MSB 		0x05
#define FXOS8700_OUT_Z_LSB 		0x06
#define FXOS8700_F_SETUP 		0x09
#define FXOS8700_TRIG_CFG 		0x0a
#define FXOS8700_SYSMOD			0x0B
#define FXOS8700_INT_SOURCE		0x0c
#define FXOS8700_WHO_AM_I		0x0d
#define FXOS8700_XYZ_DATA_CFG		0x0e
#define FXOS8700_HP_FILTER_CUTOFF	0x0f
#define FXOS8700_PL_STATUS 		0x10
#define FXOS8700_PL_CFG 		0x11
#define FXOS8700_PL_COUNT		0x12
#define FXOS8700_PL_BF_ZCOMP		0x13
#define FXOS8700_PL_P_L_THS_REG		0x14
#define FXOS8700_FFMT_CFG		0x15
#define FXOS8700_FFMT_SRC		0x16
#define FXOS8700_FFMT_THS		0x17
#define FXOS8700_FFMT_COUNT		0x18
#define FXOS8700_TRANSIDENT1_CFG	0x1d
#define FXOS8700_TRANSIDENT1_SRC	0x1e
#define FXOS8700_TRANSIDENT1_THS	0x1f
#define FXOS8700_TRANSIDENT1_COUNT	0x20
#define FXOS8700_PULSE_CFG		0x21
#define FXOS8700_PULSE_SRC		0x22
#define FXOS8700_PULSE_THSX		0x23
#define FXOS8700_PULSE_THSY		0x24
#define FXOS8700_PULSE_THSZ		0x25
#define FXOS8700_PULSE_TMLT		0x26
#define FXOS8700_PULSE_LTCY		0x27
#define FXOS8700_PULSE_WIND		0x28
#define FXOS8700_ATSLP_COUNT		0x29
#define FXOS8700_CTRL_REG1		0x2a
#define FXOS8700_CTRL_REG2		0x2b
#define FXOS8700_CTRL_REG3		0x2c
#define FXOS8700_CTRL_REG4		0x2d
#define FXOS8700_CTRL_REG5		0x2e
#define FXOS8700_OFF_X			0x2f
#define FXOS8700_OFF_Y			0x30
#define FXOS8700_OFF_Z			0x31
#define FXOS8700_M_DR_STATUS		0x32
#define FXOS8700_M_OUT_X_MSB       	0x33
#define FXOS8700_M_OUT_X_LSB       	0x34
#define FXOS8700_M_OUT_Y_MSB       	0x35
#define FXOS8700_M_OUT_Y_LSB       	0x36
#define FXOS8700_M_OUT_Z_MSB       	0x37
#define FXOS8700_M_OUT_Z_LSB       	0x38
#define FXOS8700_CMP_X_MSB       	0x39
#define FXOS8700_CMP_X_LSB       	0x3a
#define FXOS8700_CMP_Y_MSB       	0x3b
#define FXOS8700_CMP_Y_LSB       	0x3c
#define FXOS8700_CMP_Z_MSB       	0x3d
#define FXOS8700_CMP_Z_LSB       	0x3e
#define FXOS8700_M_OFF_X_MSB       	0x3f
#define FXOS8700_M_OFF_X_LSB       	0x40
#define FXOS8700_M_OFF_Y_MSB       	0x41
#define FXOS8700_M_OFF_Y_LSB       	0x42
#define FXOS8700_M_OFF_Z_MSB       	0x43
#define FXOS8700_M_OFF_Z_LSB       	0x44
#define FXOS8700_MAX_X_MSB       	0x45
#define FXOS8700_MAX_X_LSB      	0x46
#define FXOS8700_MAX_Y_MSB       	0x47
#define FXOS8700_MAX_Y_LSB       	0x48
#define FXOS8700_MAX_Z_MSB       	0x49
#define FXOS8700_MAX_Z_LSB       	0x4a
#define FXOS8700_MIN_X_MSB       	0x4b
#define FXOS8700_MIN_X_LSB       	0x4c  //product 0x1e
#define FXOS8700_MIN_Y_MSB       	0x4d  //product 0x1d
#define FXOS8700_MIN_Y_LSB       	0x4e  //product 0x1c
#define FXOS8700_MIN_Z_MSB       	0x4f  //product 0x1f
#define FXOS8700_MIN_Z_LSB       	0x50
#define FXOS8700_M_TEMP       		0x51
#define FXOS8700_MAG_THS_CFG 		0x52
#define FXOS8700_MAG_THS_SRC 		0x53
#define FXOS8700_MAG_THS_THS_X1 	0x54
#define FXOS8700_MAG_THS_THS_X0 	0x55
#define FXOS8700_MAG_THS_THS_Y1 	0x56
#define FXOS8700_MAG_THS_THS_Y0 	0x57
#define FXOS8700_MAG_THS_THS_Z1 	0x58
#define FXOS8700_MAG_THS_THS_Z0 	0x59
#define FXOS8700_MAG_THS_COUNT		0x5a
#define FXOS8700_M_CTRL_REG1		0x5b
#define FXOS8700_M_CTRL_REG2		0x5c
#define FXOS8700_M_CTRL_REG3		0x5d
#define FXOS8700_M_INT_SOURCE		0x5e
#define FXOS8700_G_VECM_CFG  		0x5f
#define FXOS8700_G_VECM_THS_MSB  	0x60
#define FXOS8700_G_VECM_THS_LSB  	0x61
#define FXOS8700_G_VECM_CNT  		0x62
#define FXOS8700_G_VECM_INITX_MSB  	0x63
#define FXOS8700_G_VECM_INITX_LSB  	0x64
#define FXOS8700_G_VECM_INITY_MSB  	0x65
#define FXOS8700_G_VECM_INITY_LSB  	0x66
#define FXOS8700_G_VECM_INITZ_MSB  	0x67
#define FXOS8700_G_VECM_INITZ_LSB  	0x68
#define FXOS8700_M_VECM_CFG  		0x69
#define FXOS8700_M_VECM_THS_MSB 	0x6a
#define FXOS8700_M_VECM_THS_LSB  	0x6b
#define FXOS8700_M_VECM_CNT  		0x6d
#define FXOS8700_M_VECM_INITX_MSB  	0x6d
#define FXOS8700_M_VECM_INITX_LSB  	0x6e
#define FXOS8700_M_VECM_INITY_MSB  	0x6f
#define FXOS8700_M_VECM_INITY_LSB  	0x70
#define FXOS8700_M_VECM_INITZ_MSB  	0x71
#define FXOS8700_M_VECM_INITZ_LSB  	0x72
#define FXOS8700_G_FFMT_THS_X1  	0x73
#define FXOS8700_G_FFMT_THS_X0 		0x74
#define FXOS8700_G_FFMT_THS_Y1  	0x75
#define FXOS8700_G_FFMT_THS_Y0  	0x76
#define FXOS8700_G_FFMT_THS_Z1  	0x77
#define FXOS8700_G_FFMT_THS_Z0  	0x78
#define FXOS8700_G_TRAN_INIT_MSB  	0x79
#define FXOS8700_G_TRAN_INIT_LSB_X 	0x7a
#define FXOS8700_G_TRAN_INIT_LSB_Y 	0x7b
#define FXOS8700_G_TRAN_INIT_LSB_Z 	0x7d
#define FXOS8700_TM_NVM_LOCK 		0x7e
#define FXOS8700_NVM_DATA0_35		0x80
#define FXOS8700_NVM_DATA_BNK3		0xa4
#define FXOS8700_NVM_DATA_BNK2		0xa5
#define FXOS8700_NVM_DATA_BNK1		0xa6
#define FXOS8700_NVM_DATA_BNK0		0xa7

// read_register - read a register value
bool AP_Compass_FXOS8700::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_FXOS8700::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_FXOS8700::read_raw()
{
	int16_t rx, ry, rz;
	uint8_t buffer[6];

	hal.i2c->readRegisters(COMPASS_ADDRESS, FXOS8700_M_OUT_X_MSB, 6, buffer);

	rx = (((int16_t)buffer[0]) << 8) | buffer[1];
    ry = (((int16_t)buffer[2]) << 8) | buffer[3];
    rz = (((int16_t)buffer[4]) << 8) | buffer[5];

    _mag_x = -rx;
    _mag_y = -ry;
    _mag_z = rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_FXOS8700::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if (_healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 20000) {
	  // the compass gets new data at 75Hz
	  return;
   }

   if (!_i2c_sem->take_nonblocking()) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _i2c_sem->give();

   //printf("read %d %d %d\n", _mag_x, _mag_y, _mag_z);

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }
}

// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_FXOS8700::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;

    float gain_multiple = 1.0;

    hal.scheduler->delay(10);

    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get FXOS8700 semaphore"));
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    //sw reset
    write_register(FXOS8700_CTRL_REG2, 0b01000000);
    hal.scheduler->delay(1);

    //write_register(FXOS8700_M_CTRL_REG2, 0b00011001);

    //set active and 50hz output rate
    write_register(FXOS8700_CTRL_REG1, 0b00100001);

    //write_register(FXOS8700_CTRL_REG3, 0b10000000);

	//OSR = 7
	//one shot degauss
    write_register(FXOS8700_M_CTRL_REG1, 0b01000001 | (7<<2));

    _i2c_sem->give();
    _initialised = true;

	// perform an initial read
	_healthy[0] = true;
	read();


    return true;
}

// Read Sensor data
bool AP_Compass_FXOS8700::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    }
    if (!_healthy[0]) {
        if (hal.scheduler->millis() < _retry_time) {
            return false;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
	   if (!_healthy[0] || _accum_count == 0) {
		  // try again in 1 second, and set I2c clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  hal.i2c->setHighSpeed(false);
		  return false;
	   }
	}

	_field[0].x = _mag_x_accum / _accum_count;
	_field[0].y = _mag_y_accum / _accum_count;
	_field[0].z = _mag_z_accum / _accum_count;
	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = hal.scheduler->micros(); // record time of update

//    // rotate to the desired orientation
//    if (product_id == AP_COMPASS_TYPE_HMC5883L) {
//        _field[0].rotate(ROTATION_YAW_90);
//    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _field[0].rotate((enum Rotation)_orientation[0].get());

    if (!_external[0]) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _field[0].rotate(_board_orientation);
    }

    _field[0] += _offset[0].get();

    // apply motor compensation
    if(_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset[0] = _motor_compensation[0].get() * _thr_or_curr;
        _field[0] += _motor_offset[0];
    }else{
        _motor_offset[0].zero();
    }

    _healthy[0] = true;

    return true;
}
