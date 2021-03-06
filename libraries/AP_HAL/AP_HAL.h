
#ifndef __AP_HAL_H__
#define __AP_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"
#include "AP_HAL_Macros.h"

extern void dbgset(uint8_t i =0);
extern void dbgclr(uint8_t i =0);
extern void dbgtgl(uint8_t i =0);

class DBGTimer {
public:
	DBGTimer() {
		dbgset();
	}
	~DBGTimer() {
		dbgclr();
	}
};

/* HAL Module Classes (all pure virtual) */
#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"

#include "utility/Print.h"
#include "utility/Stream.h"
#include "utility/BetterStream.h"

/* HAL Class definition */
#include "HAL.h"

#endif // __AP_HAL_H__

