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

#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <stdint.h>

#include <boost/shared_ptr.hpp>

namespace controller_common {
    class CanQueueServiceRequester;
}

namespace RTT {
    class TaskContext;
}

class MotorController {
public:
	typedef int32_t tact_array_t[24];

	MotorController(RTT::TaskContext *owner, const std::string &dev_name, int can_id_base);
	~MotorController();

    bool read();

	void stopFinger(int32_t id);
    void resetFinger(int id);
    void initFinger(int id);
    void openFinger(int id);
    void closeFinger(int id);
    void setMaxVel(int id, uint32_t vel);
    void sendGetPosition(int puck_id);
    bool getPosition(int puck_id, int32_t &p, int32_t &jp);
    void sendGetStatus(int puck_id);
    bool getStatus(int puck_id, int32_t &mode);
    void sendGetCurrent(int puck_id);
    bool getCurrent(int puck_id, double &c);
    void setHoldPosition(int id, bool hold);
	void setMaxTorque(int id, uint32_t vel);
	void setTargetPos(int puck_id, int32_t pos);
    void move(int puck_id);

	bool isDevOpened();

protected:
	void setProperty(int id, uint32_t property, int32_t value);
	void reqProperty(int id, uint32_t property);
	bool recEncoder2(int id, int32_t &p, int32_t &jp);
	bool recProperty(int id, int32_t &value);
	void setParameter(int32_t id, int32_t prop_id, int32_t value, bool save);

    boost::shared_ptr<controller_common::CanQueueServiceRequester> can_srv_;
    int can_id_base_;
};

#endif

