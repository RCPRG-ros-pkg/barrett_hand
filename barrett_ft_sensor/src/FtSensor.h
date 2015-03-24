/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _FT_SENSOR_H_
#define _FT_SENSOR_H_

#include <inttypes.h>
#include "CANDev.h"

class BarrettFtSensor {
public:
	enum Status {NO_ERROR = 0, WRONG_LENGTH = 0x0100, RE_TARE_SUGGESTED = 0x0200};

	BarrettFtSensor(std::string dev_name = "can0");
	~BarrettFtSensor();
	int32_t getPuckStatus();
	void setPuckStatus(int32_t status);
	int32_t readForceTorque(int16_t &fx, int16_t &fy, int16_t &fz, int16_t &tx, int16_t &ty, int16_t &tz);
	int32_t readAcceleration(int16_t &ax, int16_t &ay, int16_t &az);
	void tare();
	bool isDevOpened();
protected:
	void setProperty(int id, uint32_t property, int32_t value);
	void reqProperty(int id, uint32_t property);
	void recProperty(int id, int32_t &value);
private:
	CANDev dev;
};

#endif

