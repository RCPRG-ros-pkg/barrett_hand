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

#include "MotorController.h"

#include <string>
#include <cstring>

// For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T 5
#define PROP_CMD 29
// 32-Bit Position. R=Act, W=Cmd
#define PROP_P 48
// Velocity (cts/ms). R=Act, W=Cmd
#define PROP_V 44
// Max velocity (cts/ms)
#define PROP_MV 45
// 32-Bit Close Target
#define PROP_CT 56
// 32-Bit Open Target
#define PROP_OT 54
// 32-Bit Endpoint
#define PROP_E 52
// Mode: 0=Idle, 2=Torque, 3=PID, 4=Vel, 5=Trap
#define PROP_MODE 8
// Temperature (puck internal)
#define PROP_TEMP 9
// Thermistor (motor) temperature
#define PROP_THERM 20
// Tactile sensor array access (special)
#define PROP_TACT 106
// Flag to hold position after move
#define PROP_HOLD 77
// Max torque
#define PROP_MT 43
// 32-Bit Counts per revolution
#define PROP_CTS 68
#define PROP_CTS2 69
// Motor current (2048+205/1A)
#define PROP_IMOTOR 22

const int MODE_IDLE      = 0;
const int MODE_TORQUE    = 2;
const int MODE_PID       = 3;
const int MODE_VELOCITY  = 4;
const int MODE_TRAPEZOID = 5;

#define CMD_HI 13
#define CMD_TC 16
#define CMD_TO 17
#define CMD_CLOSE 18
#define CMD_OPEN 20
#define CMD_STOP 21

#define ALL_GROUP 0
#define PFEEDBACK_GROUP 3
#define HAND_GROUP 5
#define TACTILE_FULL_GROUP 9

#define GROUP(from, to) 0x400 | ((from) << 5) | (to)

MotorController::MotorController(std::string dev_name) : dev(dev_name) {
}

MotorController::~MotorController() {
}

void MotorController::setProperty(int id, uint32_t property, int32_t value) {
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

void MotorController::reqProperty(int id, uint32_t property) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	
	frame.can_id = id;
	frame.can_dlc = 1;
	
	frame.data[0] = property;
	
	dev.send(frame.can_id, frame.can_dlc, frame.data);
}

void MotorController::recEncoder2(int id, int32_t &p, int32_t &jp) {
	uint8_t data[8];
	int ret = dev.waitForReply(GROUP(id, PFEEDBACK_GROUP), data);
	
	if(ret == 6) {
		p = (int32_t(0x3F & data[0]) << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2];
		jp = (int32_t(0x3F & data[3]) << 16) | ((int32_t)data[4] << 8) | (int32_t)data[5];
	} else if (ret == 3) {
		p = (int32_t(0x3F & data[0]) << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2];
	}

	// this is necessary around encoder zero
	if(p > 0x200000)
		p = 0x3FFFFF - p;
	if(jp > 0x200000)
		jp = 0x3FFFFF - jp;
}

void MotorController::recTact(int id, int32_t &gr, int32_t &a, int32_t &b, int32_t &c, int32_t &d, int32_t &e) {
	uint8_t data[8];
	int ret = dev.waitForReply(GROUP(id, TACTILE_FULL_GROUP), data);

	gr = (data[0]>>4)&0x0F;
	a = ((data[0]&0x0F)<<8) | data[1];
	b = (data[2]<<4) | ((data[3]>>4)&0x0F);
	c = ((data[3]&0x0F)<<8) | data[4];
	d = (data[5]<<4) | ((data[6]>>4)&0x0F);
	e = ((data[6]&0x0F)<<8) | data[7];
}

void MotorController::recProperty(int id, int32_t &value) {
	uint8_t data[8];
	int ret = dev.waitForReply(GROUP(id, 6), data);
	
	value = data[ret-1] & 0x80 ? -1L : 0;
	for (unsigned int i = ret-1; i >= 2; i--)
		value = value << 8 | data[i];
}

void MotorController::resetFinger(int id) {
	setProperty(11+id, PROP_CMD, CMD_HI);
}

void MotorController::initHand() {
	resetFinger(0);
	resetFinger(1);
	resetFinger(2);
}

void MotorController::stopHand() {
	setProperty(GROUP(0, HAND_GROUP), PROP_CMD, CMD_STOP);
}

void MotorController::stopFinger(int32_t id) {
	setProperty(11 + id, PROP_CMD, CMD_STOP);
}

void MotorController::setOpenTarget(int id, uint32_t ot) {
	setProperty(11 + id, PROP_OT, ot);
}

void MotorController::setCloseTarget(int id, uint32_t ct) {
	setProperty(11 + id, PROP_CT, ct);
}

void MotorController::setMaxVel(int id, uint32_t vel) {
	setProperty(11 + id, PROP_MV, vel);
}

void MotorController::setMaxTorque(int id, uint32_t vel) {
	setProperty(11 + id, PROP_MT, vel);
}

void MotorController::open(int id) {
	setProperty(11 + id, PROP_CMD, CMD_OPEN);
}

void MotorController::close(int id) {
	setProperty(11 + id, PROP_CMD, CMD_CLOSE);
}

void MotorController::setTargetPos(int id, int32_t pos) {
	setProperty(11 + id, PROP_E, pos);
}

void MotorController::setTargetVel(int id, int32_t vel) {
	setProperty(11 + id, PROP_V, vel);
}

void MotorController::moveAll() {
	setProperty(GROUP(0, HAND_GROUP), PROP_MODE, MODE_TRAPEZOID);
}

void MotorController::moveAllVel() {
	setProperty(GROUP(0, HAND_GROUP), PROP_MODE, MODE_VELOCITY);
}

void MotorController::getPosition(int id, int32_t &p, int32_t &jp) {
	reqProperty(11 + id, PROP_P);
	recEncoder2(11 + id, p, jp);
}

void MotorController::getStatus(int id, int32_t &mode) {
	reqProperty(11+id, PROP_MODE);
	recProperty(11+id, mode);
}

void MotorController::getStatusAll(int32_t &mode1, int32_t &mode2, int32_t &mode3, int32_t &mode4) {
	reqProperty(GROUP(0, HAND_GROUP), PROP_MODE);
	recProperty(11, mode1);
	recProperty(12, mode2);
	recProperty(13, mode3);
	recProperty(14, mode4);
}

void MotorController::getCurrents(double &c1, double &c2, double &c3, double &c4) {
	const double c_factor = 1.0/205.0;
	int32_t current[4] = {2048, 2048, 2048, 2048};
	reqProperty(GROUP(0, HAND_GROUP), PROP_IMOTOR);
	recProperty(11, current[0]);
	recProperty(12, current[1]);
	recProperty(13, current[2]);
	recProperty(14, current[3]);
	c1 = c_factor*(static_cast<double>(current[0])-2048.0);
	c2 = c_factor*(static_cast<double>(current[1])-2048.0);
	c3 = c_factor*(static_cast<double>(current[2])-2048.0);
	c4 = c_factor*(static_cast<double>(current[3])-2048.0);
}

void MotorController::getPositionAll(int32_t &p1, int32_t &p2, int32_t &p3, int32_t &jp1, int32_t &jp2, int32_t &jp3, int32_t &s) {
	int32_t jp;
	reqProperty(GROUP(0, HAND_GROUP), PROP_P);
	recEncoder2(11 + 0, p1, jp1);
	recEncoder2(11 + 1, p2, jp2);
	recEncoder2(11 + 2, p3, jp3);
	recEncoder2(11 + 3, s, jp);
}

void MotorController::getTactile(int id, tact_array_t &tact) {
	setProperty(11 + id, PROP_TACT, 2);
	int gr, a, b, c, d, e;
	recTact(11 + id, gr, tact[0], tact[1], tact[2], tact[3], tact[4]);
	recTact(11 + id, gr, tact[5], tact[6], tact[7], tact[8], tact[9]);
	recTact(11 + id, gr, tact[10], tact[11], tact[12], tact[13], tact[14]);
	recTact(11 + id, gr, tact[15], tact[16], tact[17], tact[18], tact[19]);
	recTact(11 + id, gr, tact[20], tact[21], tact[22], tact[23], tact[24]);
}

int32_t MotorController::getParameter(int32_t id, int32_t prop_id)
{
	int32_t value;
	reqProperty(11+id, prop_id);
	recProperty(11+id, value);
	return value;
}

void MotorController::setParameter(int32_t id, int32_t prop_id, int32_t value, bool save) {
	setProperty(11 + id, prop_id, value);
	if (save) {
		setProperty(11 + id, 30, prop_id);
	}
}

void MotorController::getTemp(int id, int32_t &temp) {
	reqProperty(11+id, PROP_TEMP);
	recProperty(11+id, temp);
}

void MotorController::getTherm(int id, int32_t &temp) {
	reqProperty(11+id, PROP_THERM);
	recProperty(11+id, temp);
}

void MotorController::getCts(int id, int32_t &cts) {
	int32_t cts1, cts2;
	reqProperty(11+id, PROP_CTS);
	recProperty(11+id, cts1);

	reqProperty(11+id, PROP_CTS2);
	recProperty(11+id, cts2);

	cts = cts1 | (cts2<<16);
}

void MotorController::setHoldPosition(int id, bool hold) {
	int32_t value = 0;
	if (hold)
		value = 1;
	setProperty(11 + id, PROP_HOLD, value);
}

bool MotorController::isDevOpened() {
	return dev.isOpened();
}


