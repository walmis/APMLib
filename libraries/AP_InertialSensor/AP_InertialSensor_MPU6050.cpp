/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_MPU6050.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define MPU6000_DRDY_PIN 70
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#include "../AP_HAL_Linux/GPIO.h"
#define MPU6000_DRDY_PIN BBB_P8_14
#endif
#endif

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin 
#       define BIT_I2C_BYPASS_EN                                 0x02    // latch data ready pin
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4                        0x14    // 0001			0100
#define MPU6000ES_REV_C5                        0x15    // 0001			0101
#define MPU6000ES_REV_D6                        0x16    // 0001			0110
#define MPU6000ES_REV_D7                        0x17    // 0001			0111
#define MPU6000ES_REV_D8                        0x18    // 0001			1000
#define MPU6000_REV_C4                          0x54    // 0101			0100
#define MPU6000_REV_C5                          0x55    // 0101			0101
#define MPU6000_REV_D6                          0x56    // 0101			0110
#define MPU6000_REV_D7                          0x57    // 0101			0111
#define MPU6000_REV_D8                          0x58    // 0101			1000
#define MPU6000_REV_D9                          0x59    // 0101			1001

#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_FIFO_RST        (0x04)
#define BIT_FIFO_EN         (0x40)

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
const float AP_InertialSensor_MPU6050::_gyro_scale = (0.0174532f / 16.4f);

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_MPU6050::AP_InertialSensor_MPU6050() :
	AP_InertialSensor(),
    _drdy_pin(NULL),
    _i2c(NULL),
    _i2c_sem(NULL),
    _num_samples(0),
    _last_sample_timestamp(0),
    _initialised(false),
    _mpu6000_product_id(AP_PRODUCT_ID_NONE),
    _last_filter_hz(0),
    _error_count(0),
    _addr(0x68)
{
}

uint16_t AP_InertialSensor_MPU6050::_init_sensor( Sample_rate sample_rate )
{
    if (_initialised) return _mpu6000_product_id;
    _initialised = true;

    _i2c = hal.i2c;
    _i2c_sem = _i2c->get_semaphore();

#ifdef MPU6000_DRDY_PIN
    _drdy_pin = hal.gpio->channel(MPU6000_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;

	bool success = _hardware_init(sample_rate);
	if (success) {
		hal.scheduler->delay(5+2);
		if (!_i2c_sem->take(100)) {
			hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
		}
		_i2c_sem->give();
	}
	if (tries++ > 5) {
		hal.scheduler->panic(PSTR("PANIC: failed to boot MPU6000 5 times"));
	}


    hal.scheduler->resume_timer_procs();
    

    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
//    _last_sample_time_micros = hal.scheduler->micros();
//    hal.scheduler->delay(10);
//    if (_i2c_sem->take(100)) {
//        _read_data_transaction();
//        _i2c_sem->give();
//    }

    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU6050::_poll_data));

#if MPU6000_DEBUG
    _dump_registers();
#endif

    return _mpu6000_product_id;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_MPU6050::wait_for_sample(uint16_t timeout_ms)
{
    //DBGTimer t;
	if (_sample_available()) {
        return true;
    }

	uint32_t start = hal.scheduler->millis();
	while(hal.scheduler->millis() - start < timeout_ms) {
		if(_sem_missed) {
			_poll_data();
			_sem_missed = false;
		}

		if(_sample_available()) {
			return true;
		}
	}

//    uint32_t start = hal.scheduler->millis();
//    while ((hal.scheduler->millis() - start) < timeout_ms) {
//        uint32_t tnow = hal.scheduler->micros();
//        // we spin for the last timing_lag microseconds. Before that
//        // we yield the CPU to allow IO to happen
//        const uint16_t timing_lag = 400;
//        if (_last_sample_timestamp + _sample_time_usec > tnow+timing_lag) {
//            hal.scheduler->delay_microseconds(_last_sample_timestamp + _sample_time_usec - (tnow+timing_lag));
//        }
//        if (_sample_available()) {
//            return true;
//        }
//    }
    return false;
}

bool AP_InertialSensor_MPU6050::update( void )
{
    // wait for at least 1 sample
    if (!wait_for_sample(1000)) {
        return false;
    }

    _previous_accel[0] = _accel[0];

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();
    _gyro[0]  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    _accel[0] = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);
    _num_samples = _sum_count;
    _accel_sum.zero();
    _gyro_sum.zero();
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();

    _gyro[0].rotate(_board_orientation);
    _gyro[0] *= _gyro_scale / _num_samples;
    _gyro[0] -= _gyro_offset[0];

    _accel[0].rotate(_board_orientation);
    _accel[0] *= MPU6000_ACCEL_SCALE_1G / _num_samples;

    Vector3f accel_scale = _accel_scale[0].get();
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0] -= _accel_offset[0];

    if (_last_filter_hz != _mpu6000_filter) {
        if (_i2c_sem->take(10)) {
            _set_filter_register(_mpu6000_filter, 0);
            _error_count = 0;
            _i2c_sem->give();
        }
    }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */


/**
 * Timer process to poll for new data from the MPU6000.
 */
void AP_InertialSensor_MPU6050::_poll_data(void)
{
	static uint8_t c = 0;
	if(hal.scheduler->in_timerprocess()) {
		c++;
		if(c < 2) {
			return;
		}
		c = 0;
	} else {
		c = 0;
	}

	if (!_i2c_sem->take_nonblocking()) {
		_sem_missed = true;
		return;
	}

	// Read accelerometer FIFO to find out how many samples are available
	/* Assumes maximum packet size is gyro (6) + accel (6). */
	uint8_t data[12];
	uint8_t packet_size = 12;
	uint16_t fifo_count, index = 0;

	// fifo_count_h register contains the number of samples in the FIFO
	hal.i2c->readRegisters(_addr, MPUREG_FIFO_COUNTH, 2, data);
	fifo_count = (data[0] << 8) | data[1];
	if (fifo_count < packet_size){
		// give back i2c semaphore
		_i2c_sem->give();
		return;
	}


	if (fifo_count > (1024 >> 1)) {
		/* FIFO is 50% full, better check overflow bit. */
		hal.i2c->readRegister(_addr, MPUREG_INT_STATUS, data);
		if (data[0] & BIT_FIFO_OVERFLOW) {
			reset_fifo(INV_XYZ_ACCEL| INV_XYZ_GYRO);
			_i2c_sem->give();
			return;
		}
	}

	// read the samples
	for (uint16_t i=0; i< fifo_count / packet_size; i++) {
		// read the data
		_i2c->readRegisters(_addr, MPUREG_FIFO_R_W, packet_size, data);

		_accel_sum.y += (int16_t) (data[index+0] << 8) | data[index+1];
		_accel_sum.x += (int16_t) (data[index+2] << 8) | data[index+3];
		_accel_sum.z -= (int16_t) (data[index+4] << 8) | data[index+5];

		_gyro_sum.y  += (int16_t) (data[6] << 8) | data[7];
		_gyro_sum.x  += (int16_t) (data[8] << 8) | data[9];
		_gyro_sum.z  -= (int16_t) (data[10] << 8) | data[11];

		_sum_count++;
		if (_sum_count == 0) {
			// rollover - v unlikely
			_accel_sum.zero();
			_gyro_sum.zero();
		}
	}

	_last_sample_timestamp = hal.scheduler->micros();

	_i2c_sem->give();
}



uint8_t AP_InertialSensor_MPU6050::_register_read( uint8_t reg )
{
	uint8_t data = 0;
	_i2c->readRegister(_addr, reg, &data);
    return data;
}

void AP_InertialSensor_MPU6050::_register_write(uint8_t reg, uint8_t val)
{
    _i2c->writeRegister(_addr, reg, val);
}

int16_t AP_InertialSensor_MPU6050::reset_fifo(uint8_t sensors)
{
    uint8_t data;

    _i2c->writeRegister(_addr, MPUREG_USER_CTRL, 0);
    _i2c->writeRegister(_addr, MPUREG_USER_CTRL, BIT_FIFO_RST);
    _i2c->writeRegister(_addr, MPUREG_USER_CTRL, BIT_FIFO_EN);

    return 0;
}

/*
  useful when debugging SPI bus errors
 */
void AP_InertialSensor_MPU6050::_register_write_check(uint8_t reg, uint8_t val)
{
    uint8_t readed;
    _register_write(reg, val);
    readed = _register_read(reg);
    if (readed != val){
	hal.console->printf_P(PSTR("Values doesn't match; written: %02x; read: %02x "), val, readed);
    }
#if MPU6000_DEBUG
    hal.console->printf_P(PSTR("Values written: %02x; readed: %02x "), val, readed);
#endif
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU6050::_set_filter_register(uint8_t filter_hz, uint8_t default_filter)
{
    uint8_t filter = default_filter;
    // choose filtering frequency
    switch (filter_hz) {
    case 5:
        filter = BITS_DLPF_CFG_5HZ;
        break;
    case 10:
        filter = BITS_DLPF_CFG_10HZ;
        break;
    case 20:
        filter = BITS_DLPF_CFG_20HZ;
        break;
    case 42:
        filter = BITS_DLPF_CFG_42HZ;
        break;
    case 98:
        filter = BITS_DLPF_CFG_98HZ;
        break;
    }

    if (filter != 0) {
        _last_filter_hz = filter_hz;
        _register_write(MPUREG_CONFIG, filter);
    }
}


bool AP_InertialSensor_MPU6050::_hardware_init(Sample_rate sample_rate)
{
    if (!_i2c_sem->take(100)) {
        hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
#if MPU6000_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU6000 5 times"));
        _i2c_sem->give();
        return false;
    }

    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode
    hal.scheduler->delay(1);

    uint8_t default_filter;


    switch (sample_rate) {
    case RATE_50HZ:
    	default_filter = BITS_DLPF_CFG_10HZ;
    	_register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_50HZ);
        _sample_time_usec = 20000;
        break;
    case RATE_100HZ:
    	default_filter = BITS_DLPF_CFG_42HZ;
    	_register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_100HZ);
        _sample_time_usec = 10000;
        break;
    case RATE_200HZ:
    	default_filter = BITS_DLPF_CFG_42HZ;
        _sample_time_usec = 5000;
        _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_200HZ);
        break;
    case RATE_400HZ:
    default:
    	default_filter = BITS_DLPF_CFG_42HZ;
    	_register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_500HZ);
        _sample_time_usec = 2000;
        break;
    }

    hal.scheduler->delay(1);
    _set_filter_register(_mpu6000_filter, default_filter);


    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _mpu6000_product_id = _register_read(MPUREG_PRODUCT_ID);
    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _mpu6000_product_id);

    if ((_mpu6000_product_id == MPU6000ES_REV_C4) || (_mpu6000_product_id == MPU6000ES_REV_C5) ||
        (_mpu6000_product_id == MPU6000_REV_C4)   || (_mpu6000_product_id == MPU6000_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,2<<3);
    }
    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN | BIT_I2C_BYPASS_EN);

    hal.scheduler->delay(50);
    configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);

    _i2c_sem->give();

    return true;
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_MPU6050::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// return true if a sample is available
bool AP_InertialSensor_MPU6050::_sample_available()
{
    return (_sum_count) > 0;
}

bool AP_InertialSensor_MPU6050::configure_fifo(uint8_t sensors)
{
    int16_t result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    // Enable or disable the interrupts
    // set_int_enable(1);

    if (sensors) {
        _i2c->writeRegister(_addr, MPUREG_FIFO_EN, sensors);
    	if (reset_fifo(sensors)) {
            return -1;
        }
    }
    return 1;
}

#if MPU6000_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU6050::_dump_registers(void)
{
    hal.console->println_P(PSTR("MPU6000 registers"));
    if (_i2c_sem->take(100)) {
        for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
            uint8_t v = _register_read(reg);
            hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
            if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
                hal.console->println();
            }
        }
        hal.console->println();
        _i2c_sem->give();
    }
}
#endif


// get_delta_time returns the time period in seconds overwhich the sensor data was collected
float AP_InertialSensor_MPU6050::get_delta_time() const
{
    return _sample_time_usec * 1.0e-6f * _num_samples;
}
