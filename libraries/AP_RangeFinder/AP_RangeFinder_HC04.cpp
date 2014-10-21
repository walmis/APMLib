// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL.h>

#include "AP_RangeFinder_HC04.h"

extern const AP_HAL::HAL& hal;

void (*AP_RangeFinder_HC04::updateFunction)(RangeFinder::RangeFinder_State &s) = 0;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_HC04::AP_RangeFinder_HC04(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
	AP_RangeFinder_Backend(_ranger, instance, _state)
{
    state.healthy = false;
}

AP_RangeFinder_HC04::~AP_RangeFinder_HC04()
{
}

bool AP_RangeFinder_HC04::detect(RangeFinder &_ranger, uint8_t instance)
{
	return updateFunction != 0;
}

void AP_RangeFinder_HC04::update(void)
{
	if(updateFunction) {
		updateFunction(state);
	} else {
		state.healthy = false;
	}
}

