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

#include "barrett_hand_hw_sim/barrett_hand_hw_can.h"

using namespace RTT;

bool BarrettHandHwCAN::isSetProperty(uint16_t dlc, int8_t *data, uint8_t property, int32_t &value) const {
    if (dlc == 6 && uint8_t(data[0]) == (0x80|property) && data[1] == 0) {
        value = *reinterpret_cast<int32_t* >(data+2);
        return true;
    }
    return false;
}

bool BarrettHandHwCAN::isReqProperty(uint16_t dlc, int8_t *data, uint8_t property) const {
    if (dlc == 1 && uint8_t(data[0]) == property) {
        return true;
    }
    return false;
}

void BarrettHandHwCAN::sendPuckPos(int puck_id) {
//    Logger::In in(std::string("BarrettHandHwCAN::sendPuckPos ") + getName());
    int8_t data[8];
    uint8_t *udata = reinterpret_cast<uint8_t* >(&data[0]);
    udata[0] = (uint32_t(p_[puck_id])>>16)&0x3F;
    udata[1] = (uint32_t(p_[puck_id])>>8)&0xFF;
    udata[2] = uint32_t(p_[puck_id])&0xFF;

    if (puck_id == 3) {
        can_srv_->send(GROUP(can_id_base_ + puck_id, PFEEDBACK_GROUP), 3, data);
    }
    else {
        udata[3] = (uint32_t(jp_[puck_id])>>16)&0x3F;
        udata[4] = (uint32_t(jp_[puck_id])>>8)&0xFF;
        udata[5] = uint32_t(jp_[puck_id])&0xFF;
        can_srv_->send(GROUP(can_id_base_ + puck_id, PFEEDBACK_GROUP), 6, data);
    }
}

void BarrettHandHwCAN::sendPuckProp(int puck_id, int32_t prop, int32_t value) {
//    Logger::In in(std::string("BarrettHandHwCAN::sendPuckProp ") + getName());
    int8_t data[8];
    data[0] = prop;
    data[1] = 0;
    *reinterpret_cast<int32_t* >(data+2) = value;
    can_srv_->send(GROUP(can_id_base_ + puck_id, 6), 6, data);
}

void BarrettHandHwCAN::processPuckMsgs() {
//    Logger::In in(std::string("BarrettHandHwCAN::processPuckMsgs "));
    for (int puck_id = 0; puck_id < 4; ++puck_id) {
        int8_t data[8];
        uint16_t dlc = 0;
        while (can_srv_->readReply(can_id_base_ + puck_id, dlc, &data[0])) {
    //        Logger::log() << Logger::Info << "processPuckMsgs: read successfully" << Logger::endl;

            int32_t value;
            if (isReqProperty(dlc, data, PROP_P)) {
                sendPuckPos(puck_id);
    //            Logger::log() << Logger::Info << "req PROP_P, puck: " << puck_id << Logger::endl;
            }
            else if (isSetProperty(dlc, data, PROP_P, value)) {
                if (value == CMD_STOP) {
    //                Logger::log() << Logger::Info << "set PROP_P stop, puck: " << puck_id << Logger::endl;
                    //TODO
                }
            }
            else if (isReqProperty(dlc, data, PROP_MODE)) {
                int status = 0;
                if (!status_idle_[puck_id]) {
                    status = 1;
                }

                sendPuckProp(puck_id, PROP_MODE, status);    // TODO: verify                

    //            sendPuckProp(puck_id, PROP_MODE, 0);    // TODO
    //            Logger::log() << Logger::Info << "req PROP_MODE, puck: " << puck_id << Logger::endl;
            }
            else if (isSetProperty(dlc, data, PROP_MODE, value)) {
//                Logger::log() << Logger::Info << "set PROP_MODE: " << puck_id << Logger::endl;
                if (value == MODE_TRAPEZOID) {
                    move_hand_[puck_id] = true;
                    
                }
                else if (value == MODE_VELOCITY) {
                }
            }
            else if (isReqProperty(dlc, data, PROP_IMOTOR)) {
                sendPuckProp(puck_id, PROP_IMOTOR, 0);    // TODO
    //            Logger::log() << Logger::Info << "req PROP_IMOTOR, puck: " << puck_id << Logger::endl;
    /*
	            const double c_factor = 1.0/205.0;
	            int32_t current[4] = {2048, 2048, 2048, 2048};
	            reqProperty(GROUP(0, HAND_GROUP), PROP_IMOTOR);
	            recProperty(11, current[0]);
	            c1 = c_factor*(static_cast<double>(current[0])-2048.0);
    */
            }
            else if (isReqProperty(dlc, data, PROP_TEMP)) {
                sendPuckProp(puck_id, PROP_TEMP, 0);    // TODO
    //            Logger::log() << Logger::Info << "req PROP_TEMP, puck: " << puck_id << Logger::endl;
            }
            else if (isReqProperty(dlc, data, PROP_THERM)) {
                sendPuckProp(puck_id, PROP_THERM, 0);    // TODO
    //            Logger::log() << Logger::Info << "req PROP_THERM, puck: " << puck_id << Logger::endl;
            }
            else if (isSetProperty(dlc, data, PROP_OT, value)) {
    //            Logger::log() << Logger::Info << "set PROP_OT, puck: " << puck_id << Logger::endl;
                //TODO
            }
            else if (isSetProperty(dlc, data, PROP_CT, value)) {
    //            Logger::log() << Logger::Info << "set PROP_CT, puck: " << puck_id << Logger::endl;
                //TODO
            }
            else if (isSetProperty(dlc, data, PROP_MV, value)) {
//                Logger::log() << Logger::Info << "set PROP_MV, puck: " << puck_id << ", v: " << value << Logger::endl;
                if (puck_id == 3) {
                    v_in_(puck_id) = S2RAD(value);
                }
                else {
                    v_in_(puck_id) = P2RAD(value);
                }
            }
            else if (isSetProperty(dlc, data, PROP_MT, value)) {
//                Logger::log() << Logger::Info << "set PROP_MT, puck: " << puck_id << ", v: " << value << Logger::endl;
                t_in_(puck_id) = value;
            }
            else if (isSetProperty(dlc, data, PROP_CMD, value)) {
    //            Logger::log() << Logger::Info << "set PROP_CMD, puck: " << puck_id << Logger::endl;
                if (value == CMD_OPEN) {
                    //TODO: open finger
                }
                else if (value == CMD_CLOSE) {
                    //TODO: close finger
                }
                else if (value == CMD_HI) {
                    // reset finger
                    move_hand_[puck_id] = true;
                    v_in_(puck_id) = 1.5/1000.0;
                    q_in_(puck_id) = 0.0;
                    t_in_(puck_id) = 4000;
                }
            }
            else if (isSetProperty(dlc, data, PROP_E, value)) {
//                Logger::log() << Logger::Info << "set PROP_E, puck: " << puck_id << ", v: " << value << Logger::endl;
                if (puck_id == 3) {
                    q_in_(puck_id) = S2RAD(value);
                }
                else {
                    q_in_(puck_id) = P2RAD(value);
                }
            }
            else if (isSetProperty(dlc, data, PROP_V, value)) {
//                Logger::log() << Logger::Info << "set PROP_V, puck: " << puck_id << ", v: " << value << Logger::endl;
                //TODO: setVel
            }
            else if (isSetProperty(dlc, data, PROP_HOLD, value)) {
    //            Logger::log() << Logger::Info << "set PROP_HOLD, puck: " << puck_id << Logger::endl;
                //TODO: setHold
            }
        }
    }
}
/*
void BarrettHandHwCAN::processGroupMsgs() {
    Logger::In in(std::string("BarrettHandHwCAN::processGroupMsgs "));
    int8_t data[8];
    uint16_t dlc = 0;
    while (can_srv_->readReply(GROUP(0, HAND_GROUP), dlc, &data[0])) {
        Logger::log() << Logger::Info << "processGroupMsgs: read successfully: dlc: " << dlc << ", data[0]: " << (int)((uint8_t)data[0]) << Logger::endl;
        int32_t value;
        if (isReqProperty(dlc, data, PROP_P)) {
//            Logger::log() << Logger::Info << "req PROP_P, group: " << GROUP(0, HAND_GROUP) << Logger::endl;
            for (int puck_id = 0; puck_id < 4; ++puck_id) {
                sendPuckPos(puck_id);
            }
        }
        else if (isSetProperty(dlc, data, PROP_MODE, value)) {
            Logger::log() << Logger::Info << "set PROP_MODE, group: " << GROUP(0, HAND_GROUP) << Logger::endl;
            if (value == MODE_TRAPEZOID) {
                move_hand_[0] = true;
                move_hand_[1] = true;
                move_hand_[2] = true;
                move_hand_[3] = true;
            }
            else if (value == MODE_VELOCITY) {
            }
        }
        else if (isReqProperty(dlc, data, PROP_MODE)) {
//            Logger::log() << Logger::Info << "req PROP_MODE, group: " << GROUP(0, HAND_GROUP) << Logger::endl;
            for (int puck_id = 0; puck_id < 4; ++puck_id) {
                int status = 0;
                if (!status_idle_[puck_id]) {
                    status = 1;
                }
                sendPuckProp(puck_id, PROP_MODE, status);    // TODO: verify                
            }
        }
        else if (isReqProperty(dlc, data, PROP_IMOTOR)) {
//            Logger::log() << Logger::Info << "req PROP_IMOTOR, group: " << GROUP(0, HAND_GROUP) << Logger::endl;
            for (int puck_id = 0; puck_id < 4; ++puck_id) {
                sendPuckProp(puck_id, PROP_IMOTOR, t_out_(puck_id));    // TODO: verify
            }
        }
    }
}
*/

bool BarrettHandHwCAN::configure(RTT::TaskContext *tc, int can_id_base) {
    Logger::In in(std::string("BarrettHandHwCAN::configure ") + tc->getName());

    can_id_base_ = can_id_base;
    if (can_id_base_ < 0) {
        return false;
    }

    std::vector<std::pair<uint32_t, uint32_t > > filters;
    for (int puck_id = 0; puck_id < 4; puck_id++) {
        filters.push_back( std::pair<uint32_t, uint32_t >(can_id_base_ + puck_id, 0x07FF) );
        filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(can_id_base_ + puck_id, PFEEDBACK_GROUP), 0x07FF) );
        filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(can_id_base_ + puck_id, 6), 0x07FF) );
    }
//    filters.push_back( std::pair<uint32_t, uint32_t >(GROUP(0, HAND_GROUP), 0x07FF) );

    can_srv_ = tc->getProvider<controller_common::CanQueueServiceRequester >("can_queue");

    if (!can_srv_ || !can_srv_->initialize.ready()) {
        Logger::log() << Logger::Error << "CAN service is not loaded" << Logger::endl;
        return false;
    }
    can_srv_->initialize("TODO", filters);

    q_in_.setZero();
    v_in_.setZero();
    t_in_.setZero();
    for (int i = 0; i < 4; ++i) {
        status_idle_[i] = true;
        move_hand_[i] = false;
    }

    return true;
}

