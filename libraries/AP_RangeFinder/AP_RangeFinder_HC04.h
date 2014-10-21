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
 
#ifndef AP_RangeFinder_HC04_H
#define AP_RangeFinder_HC04_H

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <functional>

class AP_RangeFinder_HC04 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_HC04(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // destructor
    ~AP_RangeFinder_HC04(void);
    
    static void setProxy(void (*f)(RangeFinder::RangeFinder_State &s)) {
    	updateFunction = f;
    }

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

protected:
    //platform specific wrapper
    static void (*updateFunction)(RangeFinder::RangeFinder_State &s);
};

#endif // AP_RangeFinder_PX4_H
