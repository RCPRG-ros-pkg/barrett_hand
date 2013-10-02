
#include "MotorController.h"

#include <cstring>

#define PROP_CMD 29
#define PROP_P 48
#define PROP_MV 45
#define PROP_CT 56
#define PROP_OT 54
#define PROP_E 52
#define PROP_MODE 8

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
#define CMD_TACT 106

#define ALL_GROUP 0
#define PFEEDBACK_GROUP 3
#define HAND_GROUP 5

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
	//frame.data[2] = value & 0xff;
	//frame.data[3] = (value >> 8) & 0xff;
	
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
		jp = 0;
	}
	
	if(p > 2097152)
		p = 4194303 - p;
	if(jp > 2097152)
		jp = 4194303 - jp;
}

void MotorController::recProperty(int id, int32_t &value) {
	uint8_t data[8];
	int ret = dev.waitForReply(GROUP(id, 6), data);
	
	//*property = msg[0] & 0x7F;
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

void MotorController::setOpenTarget(int id, uint32_t ot) {
	setProperty(11 + id, PROP_OT, ot);
}

void MotorController::setCloseTarget(int id, uint32_t ct) {
	setProperty(11 + id, PROP_CT, ct);
}

void MotorController::setMaxVel(int id, uint32_t vel) {
	setProperty(11 + id, PROP_MV, vel);
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

void MotorController::moveAll() {
	setProperty(GROUP(0, HAND_GROUP), PROP_MODE, MODE_TRAPEZOID);
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

void MotorController::getPositionAll(int32_t &p1, int32_t &p2, int32_t &p3, int32_t &jp1, int32_t &jp2, int32_t &jp3, int32_t &s) {
	int32_t jp;
	reqProperty(GROUP(0, HAND_GROUP), PROP_P);
	recEncoder2(11 + 0, p1, jp1);
	recEncoder2(11 + 1, p2, jp2);
	recEncoder2(11 + 2, p3, jp3);
	recEncoder2(11 + 3, s, jp);
}

