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

#include "FtSensor.h"

#include <cstring>

// Status: 0=Reset/Monitor, 2=Ready/Main
#define PROP_STAT 5
// Raw strain gage A/D value (0-4095)
#define PROP_SG1 42
// Raw strain gage A/D value (0-4095)
#define PROP_SG2 43
//Raw strain gage A/D value (0-4095)
#define PROP_SG3 44
// Raw strain gage A/D value (0-4095)
#define PROP_SG4 45
// Raw strain gage A/D value (0-4095)
#define PROP_SG5 46
// Raw strain gage A/D value (0-4095)
#define PROP_SG6 47
// Force in X. Divide by 256 to get Newtons.
#define PROP_FX 48
// Force in Y. Divide by 256 to get Newtons.
#define PROP_FY 49
// Force in Z. Divide by 256 to get Newtons.
#define PROP_FZ 50
// Torque about X. Divide by 4096 to get Newton-meters.
#define PROP_TX 51
// Torque about Y. Divide by 4096 to get Newton-meters.
#define PROP_TY 52
// Torque about Z. Divide by 4096 to get Newton-meters.
#define PROP_TZ 53
// CAN-only. Read returns 2 CAN packets containing all forces and torques. Write (any value) tares the sensor.
#define PROP_FT 54
// Acceleration in X. Divide by 1024 to get m/s^2.
#define PROP_AX 55
// Acceleration in Y. Divide by 1024 to get m/s^2.
#define PROP_AY 56
// Acceleration in Z. Divide by 1024 to get m/s^2.
#define PROP_AZ 57
// Storage area for calibration gain matrix
#define PROP_GM 58
// Storage area for calibration offset vector (unused?)
#define PROP_OV 59
// Debugging variable to set LED states
#define PROP_LED 60
// Raw thermistor A/D value (0-4095) (location? conversion?)
#define PROP_T1 61
// Raw thermistor A/D value (0-4095) (location? conversion?)
#define PROP_T2 62
// Raw thermistor A/D value (0-4095) (location? conversion?)
#define PROP_T3 63
// CAN-only. Read returns single CAN packet containing all accelerations.
#define PROP_A 64

#define ALL_GROUP 0
#define GR_PROP_FEEDBACK 6
#define GR_FT_SENSOR_FORCE 10
#define GR_FT_SENSOR_TORQUE 11
#define GR_FT_SENSOR_ACCELERATION 12
#define GROUP(from, to) 0x400 | ((from) << 5) | (to)
#define CAN_ID_REC 0x00000100
#define CAN_MASK_REC 0x000003e0

#define FORCE_TORQUE_SENSOR_ID 8

BarrettFtSensor::BarrettFtSensor(std::string dev_name) : dev(dev_name, CAN_ID_REC, CAN_MASK_REC) {
}

BarrettFtSensor::~BarrettFtSensor() {
}

void BarrettFtSensor::setProperty(int id, uint32_t property, int32_t value) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	
	frame.can_id = id;
	frame.can_dlc = 6;
	
	frame.data[0] = 0x80 | property;
	frame.data[1] = 0;
	
	for(unsigned int i=2; i<6; i++){
		frame.data[i] = (uint8_t)(value & 0x000000FF);
		value >>= 8;
	}
	dev.send(frame.can_id, frame.can_dlc, frame.data);
}

void BarrettFtSensor::reqProperty(int id, uint32_t property) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	
	frame.can_id = id;
	frame.can_dlc = 1;
	
	frame.data[0] = property;
	
	dev.send(frame.can_id, frame.can_dlc, frame.data);
}

void BarrettFtSensor::recProperty(int id, int32_t &value) {
	uint8_t data[8];
	int ret = dev.waitForReply(GROUP(id, GR_PROP_FEEDBACK), data);
	
	value = data[ret-1] & 0x80 ? -1L : 0;
	for (unsigned int i = ret-1; i >= 2; i--) {
		value = value << 8 | data[i];
	}
}

int32_t BarrettFtSensor::getPuckStatus() {
	int32_t status = -1;
	reqProperty(FORCE_TORQUE_SENSOR_ID, PROP_STAT);
	recProperty(FORCE_TORQUE_SENSOR_ID, status);
	return status;
}

void BarrettFtSensor::setPuckStatus(int32_t status) {
	setProperty(FORCE_TORQUE_SENSOR_ID, PROP_STAT, status);
}

int32_t BarrettFtSensor::readForceTorque(int16_t &fx, int16_t &fy, int16_t &fz, int16_t &tx, int16_t &ty, int16_t &tz) {
	uint8_t data[8];

	reqProperty(FORCE_TORQUE_SENSOR_ID, PROP_FT);

	int ret = dev.waitForReply(GROUP(FORCE_TORQUE_SENSOR_ID, GR_FT_SENSOR_FORCE), data);
	if (ret != 6) {
		return WRONG_LENGTH;
	}
	fx = data[0] | (data[1]<<8);
	fy = data[2] | (data[3]<<8);
	fz = data[4] | (data[5]<<8);

	ret = dev.waitForReply(GROUP(FORCE_TORQUE_SENSOR_ID, GR_FT_SENSOR_TORQUE), data);
	if (ret != 6 && ret != 7) {
		return WRONG_LENGTH;
	}
	tx = data[0] | (data[1]<<8);
	ty = data[2] | (data[3]<<8);
	tz = data[4] | (data[5]<<8);

	if (ret == 7) {
		return RE_TARE_SUGGESTED | data[6];
	}

	return NO_ERROR;
}

int32_t BarrettFtSensor::readAcceleration(int16_t &ax, int16_t &ay, int16_t &az) {
	uint8_t data[8];
	reqProperty(FORCE_TORQUE_SENSOR_ID, PROP_A);
	int ret = dev.waitForReply(GROUP(FORCE_TORQUE_SENSOR_ID, GR_FT_SENSOR_ACCELERATION), data);
	if (ret != 6) {
		return WRONG_LENGTH;
	}
	ax = data[0] | (data[1]<<8);
	ay = data[2] | (data[3]<<8);
	az = data[4] | (data[5]<<8);

	return NO_ERROR;
}

void BarrettFtSensor::tare() {
	// Write (any value) tares the sensor
	setProperty(FORCE_TORQUE_SENSOR_ID, PROP_FT, 1);
}

bool BarrettFtSensor::isDevOpened() {
	return dev.isOpened();
}


