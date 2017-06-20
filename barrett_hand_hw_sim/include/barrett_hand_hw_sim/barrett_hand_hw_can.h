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

#ifndef VELMA_SIM_BARRETT_HAND_HW_CAN_H__
#define VELMA_SIM_BARRETT_HAND_HW_CAN_H__

#include "Eigen/Dense"

#include <rtt/TaskContext.hpp>

#include <controller_common/can_queue_service_requester.h>
#include "barrett_hand_controller/BarrettHandCan.h"

using namespace RTT;

#define P2RAD(x) ((x)/199111.1/180.0*M_PI*140.0)
#define S2RAD(x) ((x)*M_PI/35840.0)

class BarrettHandHwCAN {
public:
    typedef Eigen::Matrix<double, 4, 1> Dofs;
    typedef Eigen::Matrix<double, 8, 1> Joints;

    int can_id_base_;
    int32_t p_[4], jp_[3];
    bool move_hand_[4];
    bool status_idle_[4];

    Dofs q_in_;
    Dofs v_in_;
    Dofs t_in_;
//    Joints t_out_;

    bool isSetProperty(uint16_t dlc, int8_t *data, uint8_t property, int32_t &value) const;
    bool isReqProperty(uint16_t dlc, int8_t *data, uint8_t property) const;
    void sendPuckPos(int puck_id);
    void sendPuckProp(int puck_id, int32_t prop, int32_t value);
    void processPuckMsgs();
//    void processGroupMsgs();
    bool configure(RTT::TaskContext *tc, int can_id_base);

    boost::shared_ptr<controller_common::CanQueueServiceRequester > can_srv_;
};

#endif  // VELMA_SIM_BARRETT_HAND_HW_CAN_H__

