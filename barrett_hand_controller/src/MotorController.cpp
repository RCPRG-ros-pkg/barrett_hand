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

#include "barrett_hand_controller/BarrettHandCan.h"

#include <controller_common/can_queue_service_requester.h>

#include <rtt/TaskContext.hpp>

MotorController::MotorController(RTT::TaskContext *owner, const std::string &dev_name, int can_id_base)
    : can_id_base_(can_id_base)
{

    // 11                               000 0000 1011
    // 12                               000 0000 1100
    // 13                               000 0000 1101
    // 14                               000 0000 1110
    // GROUP(11, PFEEDBACK_GROUP)       001 0110 0011
    // GROUP(12, PFEEDBACK_GROUP)       001 1000 0011
    // GROUP(13, PFEEDBACK_GROUP)       001 1010 0011
    // GROUP(14, PFEEDBACK_GROUP)       001 1100 0011
    // GROUP(11, 6)                     001 0110 0110
    // GROUP(12, 6)                     001 1000 0110
    // GROUP(13, 6)                     001 1010 0110
    // GROUP(14, 6)                     001 1100 0110
    // GROUP(0, HAND_GROUP)             000 0000 0101

    // mask:    0x07FF                  111 1111 1111

    std::vector<std::pair<uint32_t, uint32_t>> filters;
    for (int puck_id = 0; puck_id < 4; puck_id++) {
        filters.push_back( std::pair<uint32_t, uint32_t >(can_id_base_ + puck_id, 0x07FF) );
        filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(can_id_base_ + puck_id, PFEEDBACK_GROUP), 0x07FF) );
        filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(can_id_base_ + puck_id, 6), 0x07FF) );
    }
// TODO: remove
//    filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(0, HAND_GROUP), 0x07FF) );

    can_srv_ = owner->getProvider<controller_common::CanQueueServiceRequester>("can_queue");
    if (can_srv_ && can_srv_->initialize.ready()) {
        can_srv_->initialize(dev_name, filters);
    }
}

MotorController::~MotorController() {
    //delete pdev_;
}

bool MotorController::read() {
    return can_srv_->readQueue();
}

int MotorController::getFramesCount() const {
    return can_srv_->getFramesCount();
}

void MotorController::setProperty(int id, uint32_t property, int32_t value) {
	int can_id = id;
	int can_dlc = 6;
	int8_t data[8];

	data[0] = 0x80 | property;
	data[1] = 0;

	for(unsigned int i = 2; i < 6; i++)
    {
        data[i] = (int8_t)(value & 0x000000FF);
        value >>= 8;
    }

    can_srv_->send(can_id, can_dlc, data);
}

void MotorController::reqProperty(int id, uint32_t property) {
	int can_id = id;
	int can_dlc = 1;
	int8_t data[8];

	data[0] = property;

	can_srv_->send(can_id, can_dlc, data);
}

bool MotorController::recEncoder2(int id, int32_t &p, int32_t &jp) {
    int8_t data[8];
    uint8_t *udata = reinterpret_cast<uint8_t* >(&data[0]);
    uint16_t dlc = 0;
    if (!can_srv_->readReply(GROUP(id, PFEEDBACK_GROUP), dlc, &data[0])) {
        return false;
    }

    p = int32_t((uint32_t(0x3F & udata[0]) << 16) | ((uint32_t)udata[1] << 8) | (uint32_t)udata[2]);
	if (dlc >= 6) {
        jp = int32_t((uint32_t(0x3F & udata[3]) << 16) | ((uint32_t)udata[4] << 8) | (uint32_t)udata[5]);
	}

	// this is necessary around encoder zero
	if(p > 0x200000)
		p = 0x3FFFFF - p;
	if(jp > 0x200000)
		jp = 0x3FFFFF - jp;
    return true;
}

bool MotorController::recProperty(int id, int32_t &value) {
    int8_t data[8];
    uint16_t dlc = 0;
    if (!can_srv_->readReply(GROUP(id, 6), dlc, data)) {
        return false;
    }
    if (dlc <= 0) {
        return false;
    }
	value = data[dlc-1] & 0x80 ? -1L : 0;
	for (unsigned int i = dlc-1; i >= 2; i--)
		value = value << 8 | data[i];
    return true;
}

void MotorController::resetFinger(int id) {
    setProperty(can_id_base_+id, PROP_CMD, CMD_RESET);
}

void MotorController::initFinger(int id) {
    setProperty(can_id_base_+id, PROP_CMD, CMD_HI);
}

void MotorController::openFinger(int id) {
    setProperty(can_id_base_+id, PROP_CMD, CMD_OPEN);
}

void MotorController::closeFinger(int id) {
    setProperty(can_id_base_+id, PROP_CMD, CMD_CLOSE);
}

void MotorController::stopFinger(int32_t id) {
	setProperty(can_id_base_ + id, PROP_CMD, CMD_STOP);
}

void MotorController::setMaxVel(int id, uint32_t vel) {
	setProperty(can_id_base_ + id, PROP_MV, vel);
}

void MotorController::setMaxTorque(int id, uint32_t vel) {
	setProperty(can_id_base_ + id, PROP_MT, vel);
}

void MotorController::setTargetPos(int puck_id, int32_t pos) {
	setProperty(can_id_base_ + puck_id, PROP_E, pos);
}

void MotorController::move(int puck_id) {
	setProperty(can_id_base_ + puck_id, PROP_MODE, MODE_TRAPEZOID);
}

void MotorController::sendGetPosition(int puck_id) {
	reqProperty(can_id_base_ + puck_id, PROP_P);
}

bool MotorController::getPosition(int puck_id, int32_t &p, int32_t &jp) {
	return recEncoder2(can_id_base_ + puck_id, p, jp);
}

void MotorController::sendGetStatus(int puck_id) {
	reqProperty(can_id_base_+puck_id, PROP_MODE);
}

bool MotorController::getStatus(int puck_id, int32_t &mode) {
    bool result = recProperty(can_id_base_+puck_id, mode);
	return result;
}

void MotorController::sendGetCurrent(int puck_id) {
	reqProperty(can_id_base_+puck_id, PROP_IMOTOR);
}

bool MotorController::getCurrent(int puck_id, double &c) {
    const double c_factor = 1.0/205.0;
    int32_t current = 2048;
    if (recProperty(can_id_base_+puck_id, current)) {
        c = c_factor*(static_cast<double>(current)-2048.0);
        return true;
    }
    return false;
}

void MotorController::setHoldPosition(int id, bool hold) {
	int32_t value = 0;
	if (hold)
		value = 1;
	setProperty(can_id_base_ + id, PROP_HOLD, value);
}

bool MotorController::isDevOpened() {
    return can_srv_
        && can_srv_->initialize.ready()
        && can_srv_->readReply.ready()
        && can_srv_->send.ready();
}


