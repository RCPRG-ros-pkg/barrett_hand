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

#include <barrett_hand_controller_msgs/BHTemp.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include "rtt_rosclock/rtt_rosclock.h"

#include <string>
#include <math.h>
#include "MotorController.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "Eigen/Dense"

#define RAD2P(x) (static_cast<double>(x) * 180.0/3.1416 / 140.0 * 199111.1)
#define RAD2S(x) (static_cast<double>(x) * 35840.0/M_PI)

using std::string;
using RTT::InputPort;
using RTT::OutputPort;

class BHCanCommand {
public:
    enum {CMD_MAX_VEL, CMD_HOLD, CMD_RESET, CMD_MAX_TORQUE, CMD_TARGET_POS, CMD_STOP, CMD_MOVE};

    BHCanCommand()
    {}

    BHCanCommand(int id, int type, int32_t value)
        : id_(id)
        , type_(type)
        , value_(value)
    {}

    int id_;
    int type_;
    int32_t value_;
};

template <size_t SIZE >
class BHCanCommandFIFO {
public:
    BHCanCommandFIFO()
        : cmd_buf_first_(0)
        , cmd_buf_len_(0)
    {}

    bool push(const BHCanCommand& cmd) {
        if (SIZE == cmd_buf_len_) {
            return false;
        }
        cmd_buf_[(cmd_buf_first_+cmd_buf_len_)%SIZE] = cmd;
        ++cmd_buf_len_;
        return true;
    }

    bool pop(BHCanCommand& cmd) {
        if (0 == cmd_buf_len_) {
            return false;
        }
        cmd = cmd_buf_[cmd_buf_first_];
        cmd_buf_first_ = (cmd_buf_first_+1)%SIZE;
        --cmd_buf_len_;
        return true;
    }

private:
    boost::array<BHCanCommand, SIZE > cmd_buf_;
    int cmd_buf_first_;
    int cmd_buf_len_;
};

class BarrettHand : public RTT::TaskContext {
private:
    enum {SEQ_BEFORE_CMD_SEND, SEQ_CMD_SEND, SEQ_STATUS_RECV};
    const int BH_DOF;
    const int BH_JOINTS;
    const int TEMP_MAX_HI;
    const int TEMP_MAX_LO;
    enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

    int status_read_seq_;

    int32_t loop_counter_;
    MotorController *ctrl_;

    int32_t maxStaticTorque_;
    int torqueSwitch_;

    bool hold_;
    bool holdEnabled_;

    typedef Eigen::Matrix<double, 4, 1 > Joints4;
    typedef Eigen::Matrix<double, 8, 1 > Joints8;

    // port variables
    Joints4 q_in_;
    Joints4 v_in_;
    Joints4 t_in_;
    double mp_in_;
    int32_t hold_in_;
    uint32_t status_out_;
    Joints4 max_measured_pressure_in_;
    Joints8 q_out_;
    Joints8 t_out_;
    barrett_hand_controller_msgs::BHTemp temp_out_;

    std_msgs::Empty reset_in_;
    std_msgs::Empty calibrate_in_;
    std_msgs::Int32 filter_in_;

    // OROCOS ports
    InputPort<Joints4 > port_q_in_;
    InputPort<Joints4 > port_v_in_;
    InputPort<Joints4 > port_t_in_;
    InputPort<double> port_mp_in_;
    InputPort<int32_t> port_hold_in_;
    InputPort<Joints4 > port_max_measured_pressure_in_;
    InputPort<std_msgs::Empty> port_reset_in_;
    OutputPort<uint32_t> port_status_out_;
    OutputPort<Joints8> port_q_out_;
    OutputPort<Joints8> port_t_out_;
    OutputPort<barrett_hand_controller_msgs::BHTemp> port_temp_out_;


    // ROS parameters
    string dev_name_;
    string prefix_;
    int can_id_base_;

    int resetFingersCounter_;

    int32_t p1_, p2_, p3_, jp1_, jp2_, jp3_, jp4_, s_;
    double currents_[4];
    int32_t mode_[4];

    int iter_counter_;

    // circular buffer for commands
    BHCanCommandFIFO<100 > cmds_;

public:
    explicit BarrettHand(const string& name)
        : TaskContext(name, PreOperational)
        , BH_DOF(4)
        , BH_JOINTS(8)
        , TEMP_MAX_HI(65)
        , TEMP_MAX_LO(60)
        , loop_counter_(0)
        , resetFingersCounter_(0)
        , maxStaticTorque_(4700)
        , torqueSwitch_(-1)
        , ctrl_(NULL)
        , p1_(0), p2_(0), p3_(0), jp1_(0), jp2_(0), jp3_(0), jp4_(0), s_(0)
        , can_id_base_(-1)
        , currents_{0,0,0,0}
        , mode_{0,0,0,0}
    {
        holdEnabled_ = false;
        hold_ = true;
        status_out_ = 0;
        mp_in_ = 0;

        this->ports()->addPort("q_INPORT", port_q_in_);
        this->ports()->addPort("v_INPORT", port_v_in_);
        this->ports()->addPort("t_INPORT", port_t_in_);
        this->ports()->addPort("mp_INPORT", port_mp_in_);
        this->ports()->addPort("hold_INPORT", port_hold_in_);

        this->ports()->addPort("q_OUTPORT", port_q_out_);
        this->ports()->addPort("t_OUTPORT", port_t_out_);
        this->ports()->addPort("status_OUTPORT", port_status_out_);

        this->ports()->addPort("BHTemp", port_temp_out_);

        this->ports()->addPort("max_measured_pressure_INPORT", port_max_measured_pressure_in_);
        this->ports()->addPort("reset_fingers_INPORT", port_reset_in_);

        this->addProperty("device_name", dev_name_);
        this->addProperty("prefix", prefix_);
        this->addProperty("can_id_base", can_id_base_);
    }

    ~BarrettHand() {
    }

    void cleanupHook() {
        if (ctrl_ != NULL) {
            delete ctrl_;
        }
    }

    // RTT configure hook
    bool configureHook() {
        RTT::Logger::In in(std::string("BarrettHand(") + getName() + ")::configureHook");
        if (can_id_base_ < 0) {
            RTT::log(RTT::Error) << "the parameter 'can_id_base' is not set " << RTT::endlog();
            return false;
        }
        if (dev_name_.empty()) {
            RTT::log(RTT::Error) << "the parameter 'dev_name' is not set " << RTT::endlog();
            return false;
        }
        if (prefix_.empty()) {
            RTT::log(RTT::Error) << "the parameter 'prefix' is not set " << RTT::endlog();
            return false;
        }
        
        if (ctrl_ == NULL) {
            ctrl_ = new MotorController(this, dev_name_, can_id_base_);

            max_measured_pressure_in_.setZero();

            if (ctrl_->isDevOpened()) {
                return true;
            }
        }
        return false;
    }

    // RTT start hook
    bool startHook()
    {
        holdEnabled_ = false;
        hold_ = true;
        status_out_ = 0;
        iter_counter_ = 0;

        cmds_.push(BHCanCommand(0, BHCanCommand::CMD_HOLD, false));
        cmds_.push(BHCanCommand(1, BHCanCommand::CMD_HOLD, false));
        cmds_.push(BHCanCommand(2, BHCanCommand::CMD_HOLD, false));
        cmds_.push(BHCanCommand(3, BHCanCommand::CMD_HOLD, false));

        cmds_.push(BHCanCommand(0, BHCanCommand::CMD_MAX_VEL, RAD2P(1)/1000.0));
        cmds_.push(BHCanCommand(1, BHCanCommand::CMD_MAX_VEL, RAD2P(1)/1000.0));
        cmds_.push(BHCanCommand(2, BHCanCommand::CMD_MAX_VEL, RAD2P(1)/1000.0));
        cmds_.push(BHCanCommand(3, BHCanCommand::CMD_MAX_VEL, RAD2S(0.7)/1000.0));

        return true;
    }

    void stopHook()
    {
// this will not work anymore:
//        ctrl_->stopHand();
    }

    void readCan() {
        int cnt = iter_counter_;
        if (cnt == 0) {
            ctrl_->getPosition(0, p1_, jp1_);
        }
        else if (cnt == 1) {
            ctrl_->getPosition(1, p2_, jp2_);
        }
        else if (cnt == 2) {
            ctrl_->getPosition(2, p3_, jp3_);
        }
        else if (cnt == 3) {
            int32_t tmp;
            ctrl_->getPosition(3, s_, tmp);
        }
        else if (cnt == 4) {
            ctrl_->getStatus(0, mode_[0]);
        }
        else if (cnt == 5) {
            ctrl_->getStatus(1, mode_[1]);
        }
        else if (cnt == 6) {
            ctrl_->getStatus(2, mode_[2]);
        }
        else if (cnt == 7) {
            ctrl_->getStatus(3, mode_[3]);
        }
        else if (cnt == 8) {
            ctrl_->getCurrent(0, currents_[0]);
        }
        else if (cnt == 9) {
            ctrl_->getCurrent(1, currents_[1]);
        }
        else if (cnt == 10) {
            ctrl_->getCurrent(2, currents_[2]);
        }
        else if (cnt == 11) {
            ctrl_->getCurrent(3, currents_[3]);
        }
    }

    void writeCan() {
        int cnt = iter_counter_;
        if (cnt == 0) {
            ctrl_->sendGetPosition(0);
        }
        else if (cnt == 1) {
            ctrl_->sendGetPosition(1);
        }
        else if (cnt == 2) {
            ctrl_->sendGetPosition(2);
        }
        else if (cnt == 3) {
            ctrl_->sendGetPosition(3);
        }
        else if (cnt == 4) {
            ctrl_->sendGetStatus(0);
        }
        else if (cnt == 5) {
            ctrl_->sendGetStatus(1);
        }
        else if (cnt == 6) {
            ctrl_->sendGetStatus(2);
        }
        else if (cnt == 7) {
            ctrl_->sendGetStatus(3);
            if (status_read_seq_ == SEQ_CMD_SEND) {
                status_read_seq_ = SEQ_STATUS_RECV;
            }
        }
        else if (cnt == 8) {
            ctrl_->sendGetCurrent(0);
        }
        else if (cnt == 9) {
            ctrl_->sendGetCurrent(1);
        }
        else if (cnt == 10) {
            ctrl_->sendGetCurrent(2);
        }
        else if (cnt == 11) {
            ctrl_->sendGetCurrent(3);
        }
        else {
            BHCanCommand cmd;
            if (cmds_.pop(cmd)) {
                if (cmd.type_ == BHCanCommand::CMD_MAX_VEL) {
                    ctrl_->setMaxVel(cmd.id_, cmd.value_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_HOLD) {
                    ctrl_->setHoldPosition(cmd.id_, cmd.value_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_RESET) {
                    ctrl_->resetFinger(cmd.id_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_MAX_TORQUE) {
                    ctrl_->setMaxTorque(cmd.id_, cmd.value_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_TARGET_POS) {
                    ctrl_->setTargetPos(cmd.id_, cmd.value_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_STOP) {
                    ctrl_->stopFinger(cmd.id_);
                }
                else if (cmd.type_ == BHCanCommand::CMD_MOVE) {
                    RTT::log(RTT::Warning) << "sending move command " << cmd.id_ << RTT::endlog();
                    //ctrl_->moveAll();
                    ctrl_->move(cmd.id_);
                    if (cmd.id_ == 3 && status_read_seq_ == SEQ_BEFORE_CMD_SEND) {
                        status_read_seq_ = SEQ_CMD_SEND;
                    }
                }
            }
        }
    }

    // RTT update hook
    // This function runs every 1 ms (1000 Hz).
    // Temperature is published every 100 ms (10 Hz).
    void updateHook()
    {
        readCan();
        iter_counter_ = (iter_counter_+1)%20;

        if (port_reset_in_.read(reset_in_) == RTT::NewData) {
            resetFingersCounter_ = 3000;
        }

        if (resetFingersCounter_ > 0)
        {
            --resetFingersCounter_;
            if (resetFingersCounter_ == 2500)
            {
                cmds_.push(BHCanCommand(0, BHCanCommand::CMD_RESET, 0));
            }
            else if (resetFingersCounter_ == 2505)
            {
                cmds_.push(BHCanCommand(1, BHCanCommand::CMD_RESET, 0));
            }
            else if (resetFingersCounter_ == 2510)
            {
                cmds_.push(BHCanCommand(2, BHCanCommand::CMD_RESET, 0));
            }
            else if (resetFingersCounter_ == 2000) {
                cmds_.push(BHCanCommand(3, BHCanCommand::CMD_RESET, 0));
            }
            return;
        }

        bool move_hand = false;

        if (port_q_in_.read(q_in_) == RTT::NewData) {
            RTT::log(RTT::Info) << "move_hand" << RTT::endlog();
            
            move_hand = true;
        }

        if (port_v_in_.read(v_in_) == RTT::NewData) {
            if (v_in_.size() == BH_DOF) {
                cmds_.push(BHCanCommand(0, BHCanCommand::CMD_MAX_VEL, RAD2P(v_in_[0])/1000.0));
                cmds_.push(BHCanCommand(1, BHCanCommand::CMD_MAX_VEL, RAD2P(v_in_[1])/1000.0));
                cmds_.push(BHCanCommand(2, BHCanCommand::CMD_MAX_VEL, RAD2P(v_in_[2])/1000.0));
                cmds_.push(BHCanCommand(3, BHCanCommand::CMD_MAX_VEL, RAD2S(v_in_[3])/1000.0));
                //ctrl_->setMaxVel(0, RAD2P(v_in_[0])/1000.0);
                //ctrl_->setMaxVel(1, RAD2P(v_in_[1])/1000.0);
                //ctrl_->setMaxVel(2, RAD2P(v_in_[2])/1000.0);
                //ctrl_->setMaxVel(3, RAD2S(v_in_[3])/1000.0);
            } else {
                RTT::log(RTT::Warning) << "Size of " << port_v_in_.getName()
                << " not equal to " << BH_DOF << RTT::endlog();
            }
        }

        if (port_t_in_.read(t_in_) == RTT::NewData) {
            if (t_in_.size() == BH_DOF) {
            } else {
                RTT::log(RTT::Warning) << "Size of " << port_t_in_.getName()
                << " not equal to " << BH_DOF << RTT::endlog();
            }
        }

        port_mp_in_.read(mp_in_);
        if (port_hold_in_.read(hold_in_) == RTT::NewData) {
            holdEnabled_ = static_cast<bool>(hold_in_);
            cmds_.push(BHCanCommand(3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_));
            //ctrl_->setHoldPosition(3, holdEnabled_ && hold_);
        }

        if (move_hand) {
            status_out_ = 0;        // clear the status
            status_read_seq_ = SEQ_BEFORE_CMD_SEND;

            for (int i=0; i<BH_DOF; i++) {
                cmds_.push(BHCanCommand(i, BHCanCommand::CMD_MAX_TORQUE, maxStaticTorque_));
                //ctrl_->setMaxTorque(i, maxStaticTorque_);
            }
            torqueSwitch_ = 5;

            cmds_.push(BHCanCommand(0, BHCanCommand::CMD_TARGET_POS, RAD2P(q_in_[0])));
            cmds_.push(BHCanCommand(1, BHCanCommand::CMD_TARGET_POS, RAD2P(q_in_[1])));
            cmds_.push(BHCanCommand(2, BHCanCommand::CMD_TARGET_POS, RAD2P(q_in_[2])));
            cmds_.push(BHCanCommand(3, BHCanCommand::CMD_TARGET_POS, RAD2S(q_in_[3])));
            cmds_.push(BHCanCommand(0, BHCanCommand::CMD_MOVE, 0));
            cmds_.push(BHCanCommand(1, BHCanCommand::CMD_MOVE, 0));
            cmds_.push(BHCanCommand(2, BHCanCommand::CMD_MOVE, 0));
            cmds_.push(BHCanCommand(3, BHCanCommand::CMD_MOVE, 0));
            //ctrl_->setTargetPos(0, RAD2P(q_in_[0]));
            //ctrl_->setTargetPos(1, RAD2P(q_in_[1]));
            //ctrl_->setTargetPos(2, RAD2P(q_in_[2]));
            //ctrl_->setTargetPos(3, RAD2S(q_in_[3]));
            //ctrl_->moveAll();
        }

        if (torqueSwitch_>0)
        {
            --torqueSwitch_;
        } else if (torqueSwitch_ == 0)
        {
            for (int i=0; i<BH_DOF; i++) {
                cmds_.push(BHCanCommand(i, BHCanCommand::CMD_MAX_TORQUE, t_in_[i]));
                //ctrl_->setMaxTorque(i, t_in_[i]);
            }
            --torqueSwitch_;
        }

        // write current joint positions
//done
//        ctrl_->getPosition(0, p1, jp1);
//        ctrl_->getPositionAll(p1, p2, p3, jp1, jp2, jp3, s);
        q_out_[0] = static_cast<double>(s_) * M_PI/ 35840.0;
        q_out_[1] = 2.0*M_PI/4096.0*static_cast<double>(jp1_)/50.0;
        q_out_[2] = 2.0*M_PI/4096.0*static_cast<double>(p1_)*(1.0/125.0 + 1.0/375.0) - q_out_[1];
        q_out_[3] = static_cast<double>(s_) * M_PI/ 35840.0;
        q_out_[4] = 2.0*M_PI*static_cast<double>(jp2_)/4096.0/50.0;
        q_out_[5] = 2.0*M_PI/4096.0*(double)(p2_)*(1.0/125.0 + 1.0/375.0) - q_out_[4];
        q_out_[6] = 2.0*M_PI*static_cast<double>(jp3_)/4096.0/50.0;
        q_out_[7] = 2.0*M_PI/4096.0*static_cast<double>(p3_)*(1.0/125.0 + 1.0/375.0) - q_out_[6];
        port_q_out_.write(q_out_);

        if (getName() == "rHand") {
//            RTT::log(RTT::Info) << q_out_.transpose() << RTT::endlog();
            Eigen::Matrix<int, 4, 1 > status_idle;
            for (int i=0; i<4; ++i) {
                status_idle(i) = mode_[i];
            }
            //RTT::log(RTT::Info) << status_idle.transpose() << RTT::endlog();
        }

        // on 1, 101, 201, 301, 401, ... step
        if ( (loop_counter_%100) == 1)
        {
            int32_t temp[4] = {0,0,0,0}, therm[4] = {0,0,0,0};
            bool allTempOk = true;
            bool oneTempTooHigh = false;

            temp_out_.header.stamp = rtt_rosclock::host_now();

            for (int i=0; i<4; i++) {
//TODO:
//                ctrl_->getTemp(i, temp[i]);
//                ctrl_->getTherm(i, therm[i]);

                if (temp[i] > TEMP_MAX_HI || therm[i] > TEMP_MAX_HI) {
                    oneTempTooHigh = true;
                } else if (temp[i] >= TEMP_MAX_LO || therm[i] >= TEMP_MAX_LO) {
                    allTempOk = false;
                }
                temp_out_.temp[i] = temp[i];
            }

            if (hold_ && oneTempTooHigh) {
                hold_ = false;
                cmds_.push(BHCanCommand(3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_));
                //ctrl_->setHoldPosition(3, holdEnabled_ && hold_);
                RTT::log(RTT::Warning) << "Temperature is too high. Disabled spread hold." << RTT::endlog();
            } else if (!hold_ && allTempOk) {
                hold_ = true;
                cmds_.push(BHCanCommand(3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_));
                //ctrl_->setHoldPosition(3, holdEnabled_ && hold_);
                RTT::log(RTT::Warning) << "Temperature is lower. Enabled spread hold." << RTT::endlog();
            }

            port_temp_out_.write(temp_out_);
        }

        port_max_measured_pressure_in_.read(max_measured_pressure_in_);

//        std::cout << max_measured_pressure_in_.transpose() << "   " << mp_in_ << std::endl;
//        if (resetFingersCounter_ <= 0) {
//        }

//done:
//        ctrl_->getStatusAll(mode[0], mode[1], mode[2], mode[3]);

        // chack for torque switch activation
        if (fabs(q_out_[2]*3.0-q_out_[1]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH1;
        }
        if (fabs(q_out_[5]*3.0-q_out_[4]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH2;
        }
        if (fabs(q_out_[7]*3.0-q_out_[6]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH3;
        }

        if (mode_[0] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE1;
            if ((status_out_&STATUS_OVERPRESSURE1) == 0 && fabs(q_in_[0]-q_out_[1]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT1;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE1;
        }

        if (mode_[1] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE2;
            if ((status_out_&STATUS_OVERPRESSURE2) == 0 && fabs(q_in_[1]-q_out_[4]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT2;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE2;
        }

        if (mode_[2] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE3;
            if ((status_out_&STATUS_OVERPRESSURE3) == 0 && fabs(q_in_[2]-q_out_[6]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT3;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE3;
        }

        if (mode_[3] == 0 || (holdEnabled_ && fabs(q_in_[3]-q_out_[3]) < 0.05) && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE4;
            if (fabs(q_in_[3]-q_out_[3]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT4;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE4;
        }

        bool f1_stopped(false), f2_stopped(false), f3_stopped(false);
        for (int j=0; j<24; ++j) {
            if ( (status_out_&STATUS_IDLE1) == 0 && !f1_stopped) {
                if (max_measured_pressure_in_(0) > mp_in_) {
                    cmds_.push(BHCanCommand(0, BHCanCommand::CMD_STOP, 0));
                    //ctrl_->stopFinger(0);
                    status_out_ |= STATUS_OVERPRESSURE1;
                    f1_stopped = true;
                }
            }
            if ( (status_out_&STATUS_IDLE2) == 0 && !f2_stopped) {
                if (max_measured_pressure_in_(1) > mp_in_) {
                    cmds_.push(BHCanCommand(1, BHCanCommand::CMD_STOP, 0));
                    //ctrl_->stopFinger(1);
                    status_out_ |= STATUS_OVERPRESSURE2;
                    f2_stopped = true;
                }
            }
            if ( (status_out_&STATUS_IDLE3) == 0 && !f3_stopped) {
                if (max_measured_pressure_in_(2) > mp_in_) {
                    cmds_.push(BHCanCommand(2, BHCanCommand::CMD_STOP, 0));
                    //ctrl_->stopFinger(2);
                    status_out_ |= STATUS_OVERPRESSURE3;
                    f3_stopped = true;
                }
            }
        }

//done:
//        ctrl_->getCurrents(currents[0], currents[1], currents[2], currents[3]);
        t_out_[0] = currents_[3];
        t_out_[1] = currents_[0];
        t_out_[2] = currents_[0];
        t_out_[3] = currents_[3];
        t_out_[4] = currents_[1];
        t_out_[5] = currents_[1];
        t_out_[6] = currents_[2];
        t_out_[7] = currents_[2];

        port_t_out_.write(t_out_);
        port_status_out_.write(status_out_);

        loop_counter_ = (loop_counter_+1)%1000;
        writeCan();
    }
};
ORO_CREATE_COMPONENT(BarrettHand)

