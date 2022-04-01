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

#include <string>

#include <barrett_hand_msgs/CommandHand.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include "rtt_rosclock/rtt_rosclock.h"

#include "MotorController.h"

#include "fabric_logger/fabric_logger.h"

using fabric_logger::FabricLoggerInterfaceRtPtr;
using fabric_logger::FabricLogger;

using namespace RTT;

#define RAD2P(x) (int(((double)(x) * 180.0 * 199111.1) / (3.1416 * 140.0)))
#define RAD2S(x) (int((double)(x) * 35840.0 / M_PI))

#define VEL(x) ((double(x)) / 1000.0)

constexpr int BH_DOF = 4;
constexpr int BH_JOINTS = 8;
constexpr int TEMP_MAX_HI = 65;
constexpr int TEMP_MAX_LO = 60;
constexpr int STATIC_TORQUE_MAX = 4700;

using Joints4 = boost::array<double, 4>;
using Joints8 = boost::array<double, 8>;

struct BHCanCommand {
public:
    enum : uint8_t {
        CMD_MAX_VEL,
        CMD_HOLD,
        CMD_RESET,
        CMD_INIT,
        CMD_MAX_TORQUE,
        CMD_TARGET_POS,
        CMD_STOP,
        CMD_MOVE
    };

    uint8_t id;
    uint8_t type;
    int32_t value;
};

template <uint8_t SIZE>
class BHCanCommandFIFO {
public:
    BHCanCommandFIFO()
        : cmd_buf_len_(0)
        , cmd_buf_first_(0)
        , overloaded_(false)
    {}

    bool push(const BHCanCommand& cmd) {
        if (SIZE == cmd_buf_len_) {
            overloaded_ = true;
            return false;
        }

        cmd_buf_[(cmd_buf_first_+cmd_buf_len_) % SIZE] = cmd;
        ++cmd_buf_len_;
        return true;
    }

    bool pop(BHCanCommand& cmd) {
        if (0 == cmd_buf_len_) {
            return false;
        }

        cmd = cmd_buf_[cmd_buf_first_];
        cmd_buf_first_ = ((cmd_buf_first_+1) % SIZE);
        --cmd_buf_len_;
        return true;
    }

    void clear() {
        cmd_buf_len_ = 0;
    }

    bool wasOverloaded() {
        return overloaded_;
    }

    int size() const {
        return cmd_buf_len_;
    }

private:
    uint8_t cmd_buf_len_;
    uint8_t cmd_buf_first_;
    std::array<BHCanCommand, SIZE> cmd_buf_;
    bool overloaded_;
};

class BarrettHand : public RTT::TaskContext {
private:
    enum : uint8_t {
        SEQ_BEFORE_CMD_SEND,
        SEQ_CMD_SEND,
        SEQ_STATUS_RECV
    };

    enum : uint16_t {
        STATE_INIT_HAND,
        STATE_NORMAL_OP
    };

    enum : uint16_t {
        STATUS_OVERCURRENT1 = 0x0001,
        STATUS_OVERCURRENT2 = 0x0002,
        STATUS_OVERCURRENT3 = 0x0004,
        STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010,
        STATUS_OVERPRESSURE2 = 0x0020,
        STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100,
        STATUS_TORQUESWITCH2 = 0x0200,
        STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000,
        STATUS_IDLE2 = 0x2000,
        STATUS_IDLE3 = 0x4000,
        STATUS_IDLE4 = 0x8000
    };

    enum : uint16_t {
        PROP_STATUS,
        PROP_CURRENT
    };

    uint16_t current_state_;
    uint16_t expected_prop_type_;

    uint8_t status_read_seq_;

    enum : uint16_t {
        STATE_CURRENT_FSM_INIT,
        STATE_CURRENT_FSM_CURRENT_RECV,
        STATE_CURRENT_FSM_STATUS_RECV,
        STATE_CURRENT_FSM_POS_RECV,
    };
    uint16_t status_current_fsm_;
    bool recv_currents_[4];
    bool recv_status_[4];
    bool recv_pos_[4];

    //int16_t loop_counter_;
    std::unique_ptr<MotorController> ctrl_;

    int torqueSwitch_;

    bool hold_;
    bool holdEnabled_;

    // port variables
    uint16_t status_out_;
    Joints4 max_measured_pressure_in_;
    Joints8 q_out_;
    Joints8 t_out_;

    // OROCOS ports
    InputPort<barrett_hand_msgs::CommandHand> port_cmd_in_;
    InputPort<Joints4> port_max_measured_pressure_in_;
    InputPort<uint8_t> port_reset_in_;
    OutputPort<uint32_t> port_status_out_;
    OutputPort<Joints8> port_q_out_;
    OutputPort<Joints8> port_t_out_;

    // ROS parameters
    std::string dev_name_;
    int can_id_base_;

    // This vector may contain constant configuration for gripper, if required,
    // e.g. when a physical gripper is not connected
    std::vector<double> constant_configuration_;

    int32_t p1_, p2_, p3_, jp1_, jp2_, jp3_, jp4_, s_;
    double currents_[4];
    int32_t mode_[4];

    ros::Time initHandTime_;
    ros::Time sendGetTime_;
    bool initHandF1Sent_;
    bool initHandF2Sent_;
    bool initHandF3Sent_;
    bool initHandSpSent_;
    int normalOpIterCounter_;

    // circular buffer for commands
    BHCanCommandFIFO<100> cmds_;

    FabricLoggerInterfaceRtPtr m_fabric_logger;

public:
    explicit BarrettHand(const std::string& name)
        : TaskContext(name, PreOperational)
        //, loop_counter_(0)
        , current_state_(STATE_NORMAL_OP)
        , torqueSwitch_(-1)
        , p1_(0), p2_(0), p3_(0), jp1_(0), jp2_(0), jp3_(0), jp4_(0), s_(0)
        , can_id_base_(-1)
        , currents_{0, 0, 0, 0}
        , mode_{0, 0, 0, 0}
        , normalOpIterCounter_(0)
        , m_fabric_logger( FabricLogger::createNewInterfaceRt( name, 10000) )
    {
        sendGetTime_ = rtt_rosclock::host_now();

        holdEnabled_ = false;
        hold_ = true;
        status_out_ = 0;

        this->ports()->addPort("q_OUTPORT", port_q_out_);
        this->ports()->addPort("t_OUTPORT", port_t_out_);
        this->ports()->addPort("status_OUTPORT", port_status_out_);

        this->ports()->addPort("cmd_INPORT", port_cmd_in_);
        this->ports()->addPort("max_measured_pressure_INPORT", port_max_measured_pressure_in_);
        this->ports()->addPort("reset_INPORT", port_reset_in_);

        this->addProperty("device_name", dev_name_);
        this->addProperty("can_id_base", can_id_base_);
        this->addProperty("constant_configuration", constant_configuration_);
    }

    ~BarrettHand() = default;

    void cleanupHook() {
        ctrl_.reset();
    }

    // RTT configure hook
    bool configureHook() {
        RTT::Logger::In in(std::string("BarrettHand(") + getName() + ")::configureHook");
        if (can_id_base_ < 0) {
            Logger::log() << Logger::Error << "the parameter 'can_id_base' is not set " << Logger::endl;
            return false;
        }
        if (dev_name_.empty()) {
            Logger::log() << Logger::Error << "the parameter 'dev_name' is not set " << Logger::endl;
            return false;
        }
        if (constant_configuration_.size() != 0 && constant_configuration_.size() != 4) {
            Logger::log() << Logger::Error << "wrong size of constant_configuration ROS parameter: " << constant_configuration_.size() << Logger::endl;
            return false;
        }

        ctrl_.reset(new MotorController(this, dev_name_, can_id_base_));
        if (!ctrl_->isDevOpened()) {
            Logger::log() << Logger::Error << "could not open CAN bus" << Logger::endl;
            return false;
        }

        Logger::log() << Logger::Info << "constant_configuration size: " << constant_configuration_.size() << Logger::endl;
        if (constant_configuration_.size() == 4){
            Logger::log() << Logger::Info << "constant_configuration: " << constant_configuration_[0] << Logger::endl;
            Logger::log() << Logger::Info << "constant_configuration: " << constant_configuration_[1] << Logger::endl;
            Logger::log() << Logger::Info << "constant_configuration: " << constant_configuration_[2] << Logger::endl;
            Logger::log() << Logger::Info << "constant_configuration: " << constant_configuration_[3] << Logger::endl;
        }

        holdEnabled_ = false;
        hold_ = true;
        status_out_ = 0;
        normalOpIterCounter_ = 0;

        for (int i = 0; i < 4; ++i) {
            recv_status_[i] = false;
            recv_currents_[i] = false;
            recv_pos_[i] = false;
        }
        status_current_fsm_ = STATE_CURRENT_FSM_INIT;

        return true;
    }

    // RTT start hook
    bool startHook()
    {
        return true;
    }

    void stopHook() {}

    void readCan() {
        // Read all messages form the queue
        int32_t tmp;

        ros::Time now = rtt_rosclock::host_now();

        if (status_current_fsm_ == STATE_CURRENT_FSM_STATUS_RECV) {
            if (now > sendGetTime_ + ros::Duration(0.05)) {
                int reads[4] = {0, 0, 0, 0};
                for (int puck_id = 0; puck_id < 4; ++puck_id) {
                    for (int i=0; i < 20; ++i) {
                        ++reads[puck_id];
                        if (ctrl_->getStatus(puck_id, mode_[puck_id])) {
                            m_fabric_logger << "write can: getStatus " << puck_id << FabricLogger::End();
                            recv_status_[puck_id]=true;
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }
        else if (status_current_fsm_ == STATE_CURRENT_FSM_CURRENT_RECV) {
            if (now > sendGetTime_ + ros::Duration(0.05)) {
                int reads[4] = {0, 0, 0, 0};
                for (int puck_id = 0; puck_id < 4; ++puck_id) {
                    for (int i=0; i < 20; ++i) {
                        ++reads[puck_id];
                        if (ctrl_->getCurrent(puck_id, currents_[puck_id])) {
                            m_fabric_logger << "write can: getCurrent " << puck_id << FabricLogger::End();
                            //std::cout << " <- " << getName() << " " << puck_id << " " << i << std::endl;
                            recv_currents_[puck_id]=true;
                        }
                        else {
                            break;
                        }
                    }
                }
            //std::cout << "reads current: " << reads[0] << " " << reads[1] << " " << reads[2] << " " << reads[3] << std::endl;
            }
        }
        else if (status_current_fsm_ == STATE_CURRENT_FSM_POS_RECV) {
            for (int i=0; i < 20; ++i) {
                if (ctrl_->getPosition(0, p1_, jp1_)) {
                    m_fabric_logger << "write can: getPosition " << 0 << FabricLogger::End();
                    recv_pos_[0]=true;
                }
                else {
                    break;
                }
            }
            for (int i=0; i < 20; ++i) {
                if (ctrl_->getPosition(1, p2_, jp2_)) {
                    m_fabric_logger << "write can: getPosition " << 1 << FabricLogger::End();
                    recv_pos_[1]=true;
                }
                else {
                    break;
                }
            }
            for (int i=0; i < 20; ++i) {
                if (ctrl_->getPosition(2, p3_, jp3_)) {
                    m_fabric_logger << "write can: getPosition " << 2 << FabricLogger::End();
                    recv_pos_[2]=true;
                }
                else {
                    break;
                }
            }
            for (int i=0; i < 20; ++i) {
                if (ctrl_->getPosition(3, s_, tmp)) {
                    m_fabric_logger << "write can: getPosition " << 3 << FabricLogger::End();
                    recv_pos_[3]=true;
                }
                else {
                    break;
                }
            }
        }
    }

    void writeCanCommand(const BHCanCommand& cmd) {
        switch(cmd.type) {
            case BHCanCommand::CMD_MAX_VEL:
                ctrl_->setMaxVel(cmd.id, cmd.value);
                m_fabric_logger << "write can: setMaxVel " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_HOLD:
                ctrl_->setHoldPosition(cmd.id, cmd.value);
                m_fabric_logger << "write can: setHoldPosition " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_RESET:
                ctrl_->resetFinger(cmd.id);
                m_fabric_logger << "write can: resetFinger " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_INIT:
                ctrl_->initFinger(cmd.id);
                m_fabric_logger << "write can: initFinger " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_MAX_TORQUE:
                ctrl_->setMaxTorque(cmd.id, cmd.value);
                m_fabric_logger << "write can: setMaxTorque " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_TARGET_POS:
                ctrl_->setTargetPos(cmd.id, cmd.value);
                m_fabric_logger << "write can: setTargetPos " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_STOP:
                ctrl_->stopFinger(cmd.id);
                m_fabric_logger << "write can: stopFinger " << cmd.id << FabricLogger::End();
                break;

            case BHCanCommand::CMD_MOVE:
                ctrl_->move(cmd.id);
                m_fabric_logger << "write can: move " << cmd.id << FabricLogger::End();
                if (cmd.id == 3 && status_read_seq_ == SEQ_BEFORE_CMD_SEND) {
                    status_read_seq_ = SEQ_CMD_SEND;
                }
                break;

            default:
                Logger::log() << Logger::Warning << "Unknown command " << cmd.id << Logger::endl;
                m_fabric_logger << "WARNING: Unknown command for " << cmd.id << ": " << cmd.type
                                << FabricLogger::End();
                break;
        }
    }

    ros::Time last_read_time_;
    void writeNormaOpDataToCan() {
        // Use this magic with normalOpIterCounter_ to limit number of commands sent to the gripper
        // in every iteration
        ros::Time now = rtt_rosclock::host_now();
        switch(normalOpIterCounter_) {
            case 0:
            case 1:
            case 3:
                if (status_current_fsm_ == STATE_CURRENT_FSM_INIT || now > sendGetTime_ + ros::Duration(0.5)) {
                    status_current_fsm_ = STATE_CURRENT_FSM_STATUS_RECV;
                    for (int puck_id = 0; puck_id < 4; ++puck_id) {
                        recv_status_[puck_id] =false;
                        ctrl_->sendGetStatus(puck_id);
                        m_fabric_logger << "write can: sendGetStatus " << puck_id << FabricLogger::End();
                    }
                    //std::cout << getName() << " sendGetStatus init" << std::endl;
                    sendGetTime_ = now;
                }
                else if (status_current_fsm_ == STATE_CURRENT_FSM_STATUS_RECV) {
                    if (recv_status_[0] && recv_status_[1] && recv_status_[2] && recv_status_[3] && now > sendGetTime_ + ros::Duration(0.1)) {
                        status_current_fsm_ = STATE_CURRENT_FSM_CURRENT_RECV;
                        for (int puck_id = 0; puck_id < 4; ++puck_id) {
                            recv_currents_[puck_id] =false;
                            ctrl_->sendGetCurrent(puck_id);
                            m_fabric_logger << "write can: sendGetCurrent " << puck_id << FabricLogger::End();
                        }
                        //std::cout << getName() << " sendGetCurrent" << std::endl;
                        sendGetTime_ = now;
                    }
                }
                else if (status_current_fsm_ == STATE_CURRENT_FSM_CURRENT_RECV) {
                    if (recv_currents_[0] && recv_currents_[1] && recv_currents_[2] && recv_currents_[3] && recv_status_[3] && now > sendGetTime_ + ros::Duration(0.1)) {
                        status_current_fsm_ = STATE_CURRENT_FSM_POS_RECV;
                        for (int puck_id = 0; puck_id < 4; ++puck_id) {
                            recv_pos_[puck_id] =false;
                            ctrl_->sendGetPosition(puck_id);
                            m_fabric_logger << "write can: sendGetPosition " << puck_id << FabricLogger::End();
                        }
                        //std::cout << getName() << " sendGetPosition" << std::endl;
                        sendGetTime_ = now;
                    }
                }
                else if (status_current_fsm_ == STATE_CURRENT_FSM_POS_RECV) {
                    if (recv_pos_[0] && recv_pos_[1] && recv_pos_[2] && recv_pos_[3] && recv_status_[3] && now > sendGetTime_ + ros::Duration(0.1)) {
                        status_current_fsm_ = STATE_CURRENT_FSM_STATUS_RECV;
                        for (int puck_id = 0; puck_id < 4; ++puck_id) {
                            recv_status_[puck_id] =false;
                            ctrl_->sendGetStatus(puck_id);
                            m_fabric_logger << "write can: sendGetStatus " << puck_id << FabricLogger::End();
                        }
                        //std::cout << getName() << " sendGetStatus" << std::endl;
                        sendGetTime_ = now;
                    }
                }

                break;

            case 4:
                if (status_read_seq_ == SEQ_CMD_SEND) {
                    status_read_seq_ = SEQ_STATUS_RECV;
                }
                break;

            case 2:
            case 5:
                for (int i = 0; i < 3; ++i) {
                    BHCanCommand cmd;
                    if (cmds_.pop(cmd)) {
                        m_fabric_logger << "popped command" << FabricLogger::End();
                        writeCanCommand(cmd);
                        m_fabric_logger << "wrote command, size: " << cmds_.size() << FabricLogger::End();
                    }
                    else {
                        m_fabric_logger << "could not pop command, size: " << cmds_.size() << FabricLogger::End();
                        break;
                    }
                }
                break;
            default:
                Logger::log() << Logger::Warning << "writeNormaOpDataToCan: Unknown normalOpIterCounter_ " << normalOpIterCounter_ << Logger::endl;
                break;
        }
    }

    void switchToStateInitHand() {
        // Change the state
        current_state_ = STATE_INIT_HAND;

        // Save start time of the state
        initHandTime_ = rtt_rosclock::host_now();

        // Initialize variables that tell, if "init hand" command was sent to the gripper
        initHandF1Sent_ = false;
        initHandF2Sent_ = false;
        initHandF3Sent_ = false;
        initHandSpSent_ = false;

        // Clear all commands in the command buffer.
        // We cannot communicate with the gripper during the initialization procedure.
        cmds_.clear();
        m_fabric_logger << "cleared all commands" << FabricLogger::End();
        m_fabric_logger << "switchToStateInitHand" << FabricLogger::End();
    }

    void iterateStateInitHand() {
        ctrl_->read();

        m_fabric_logger << "input frames: " << ctrl_->getFramesCount() << FabricLogger::End();

        readCan();

        ros::Time now = rtt_rosclock::host_now();
        if (now > initHandTime_ + ros::Duration(0.1) && !initHandF1Sent_) {
            initHandF1Sent_ = true;
            cmds_.push(BHCanCommand{0, BHCanCommand::CMD_INIT, 0});
            m_fabric_logger << "added command 0 CMD_INIT" << FabricLogger::End();
        }
        else if (now > initHandTime_ + ros::Duration(0.2) && !initHandF2Sent_) {
            initHandF2Sent_ = true;
            cmds_.push(BHCanCommand{1, BHCanCommand::CMD_INIT, 0});
            m_fabric_logger << "added command 1 CMD_INIT" << FabricLogger::End();
        }
        else if (now > initHandTime_ + ros::Duration(0.3) && !initHandF3Sent_) {
            initHandF3Sent_ = true;
            cmds_.push(BHCanCommand{2, BHCanCommand::CMD_INIT, 0});
            m_fabric_logger << "added command 2 CMD_INIT" << FabricLogger::End();
        }
        else if (now > initHandTime_ + ros::Duration(1.5) && !initHandSpSent_) {
            initHandSpSent_ = true;
            cmds_.push(BHCanCommand{3, BHCanCommand::CMD_INIT, 0});
            m_fabric_logger << "added command 3 CMD_INIT" << FabricLogger::End();
        }
        else if (now > initHandTime_ + ros::Duration(3)) {
            switchToStateNormalOp();
        }

        // Add commands to queue
        BHCanCommand cmd;
        if (cmds_.pop(cmd)) {
            m_fabric_logger << "popped command" << FabricLogger::End();
            writeCanCommand(cmd);
            m_fabric_logger << "wrote command, size: " << cmds_.size() << FabricLogger::End();
        }
        else {
            m_fabric_logger << "could not pop command, size: " << cmds_.size() << FabricLogger::End();
        }
        // Write output ports
        calculateJointPositions();
        port_status_out_.write(status_out_);
        port_t_out_.write(t_out_);
        writeOutputPorts();
    }

    void switchToStateNormalOp() {
        current_state_ = STATE_NORMAL_OP;
        normalOpIterCounter_ = 0;
        m_fabric_logger << "switchToStateNormalOp" << FabricLogger::End();
    }

    //int iter_counter_print_;
    void iterateStateNormalOp() {
        ctrl_->read();

        //if (cmds_.wasOverloaded()) {
        //    if (iter_counter_print_ > 1000 || iter_counter_print_ < 0) {
        //        printf("%s: cmds queue was overloaded\n", getName().c_str());
        //        iter_counter_print_ = 0;
        //    }
        //    else {
        //        ++iter_counter_print_;
        //    }
        //}
        readCan();
        normalOpIterCounter_ = ((normalOpIterCounter_+1) % 6);

        auto cmd_in = barrett_hand_msgs::CommandHand();
        if (port_cmd_in_.read(cmd_in) == RTT::NewData) {
            handleCommand(cmd_in);
        }

        handleTorqueSwitch(cmd_in);
        calculateJointPositions();
        //handleFingersTemperature();
        handleStatus(cmd_in);
        handleCurrents();

        writeOutputPorts();

        writeNormaOpDataToCan();

        uint8_t reset_in = 0;
        if (port_reset_in_.read(reset_in) == RTT::NewData && reset_in == 1) {
            switchToStateInitHand();
            return;
        }
    }

    void handleCommand(const barrett_hand_msgs::CommandHand& cmd_in)
    {
        m_fabric_logger << "handleCommand: q=[" << cmd_in.q[0] << ", " << cmd_in.q[1] << ", "
                << cmd_in.q[2] << ", " << cmd_in.q[3] << "]"
                << ", dq=[" << cmd_in.dq[0] << ", " << cmd_in.dq[1] << ", "
                << cmd_in.dq[2] << ", " << cmd_in.dq[3] << "], "
                << FabricLogger::End();

        //Logger::log() << Logger::Info << "move_hand" << Logger::endl;
        cmds_.push(BHCanCommand{0, BHCanCommand::CMD_MAX_VEL, static_cast<int32_t>(VEL(RAD2P(cmd_in.dq[0])))});
        cmds_.push(BHCanCommand{1, BHCanCommand::CMD_MAX_VEL, static_cast<int32_t>(VEL(RAD2P(cmd_in.dq[1])))});
        cmds_.push(BHCanCommand{2, BHCanCommand::CMD_MAX_VEL, static_cast<int32_t>(VEL(RAD2P(cmd_in.dq[2])))});
        cmds_.push(BHCanCommand{3, BHCanCommand::CMD_MAX_VEL, static_cast<int32_t>(VEL(RAD2S(cmd_in.dq[3])))});
        m_fabric_logger << "added command: 0 CMD_MAX_VEL" << FabricLogger::End();
        m_fabric_logger << "added command: 1 CMD_MAX_VEL" << FabricLogger::End();
        m_fabric_logger << "added command: 2 CMD_MAX_VEL" << FabricLogger::End();
        m_fabric_logger << "added command: 3 CMD_MAX_VEL" << FabricLogger::End();

        status_out_ = 0;        // clear the status
        status_read_seq_ = SEQ_BEFORE_CMD_SEND;

        for (int i = 0; i < BH_DOF; i++) {
            cmds_.push(BHCanCommand{static_cast<uint8_t>(i), BHCanCommand::CMD_MAX_TORQUE, STATIC_TORQUE_MAX});
            m_fabric_logger << "added command: " << i << " CMD_MAX_TORQUE" << FabricLogger::End();
        }
        torqueSwitch_ = 5;

        holdEnabled_ = cmd_in.hold;
        cmds_.push(BHCanCommand{3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_});
        cmds_.push(BHCanCommand{0, BHCanCommand::CMD_TARGET_POS, RAD2P(cmd_in.q[0])});
        cmds_.push(BHCanCommand{1, BHCanCommand::CMD_TARGET_POS, RAD2P(cmd_in.q[1])});
        cmds_.push(BHCanCommand{2, BHCanCommand::CMD_TARGET_POS, RAD2P(cmd_in.q[2])});
        cmds_.push(BHCanCommand{3, BHCanCommand::CMD_TARGET_POS, RAD2S(cmd_in.q[3])});
        cmds_.push(BHCanCommand{0, BHCanCommand::CMD_MOVE, 0});
        cmds_.push(BHCanCommand{1, BHCanCommand::CMD_MOVE, 0});
        cmds_.push(BHCanCommand{2, BHCanCommand::CMD_MOVE, 0});
        cmds_.push(BHCanCommand{3, BHCanCommand::CMD_MOVE, 0});
        m_fabric_logger << "added command: 3 CMD_HOLD" << FabricLogger::End();
        m_fabric_logger << "added command: 0 CMD_TARGET_POS" << FabricLogger::End();
        m_fabric_logger << "added command: 1 CMD_TARGET_POS" << FabricLogger::End();
        m_fabric_logger << "added command: 2 CMD_TARGET_POS" << FabricLogger::End();
        m_fabric_logger << "added command: 3 CMD_TARGET_POS" << FabricLogger::End();
        m_fabric_logger << "added command: 0 CMD_MOVE" << FabricLogger::End();
        m_fabric_logger << "added command: 1 CMD_MOVE" << FabricLogger::End();
        m_fabric_logger << "added command: 2 CMD_MOVE" << FabricLogger::End();
        m_fabric_logger << "added command: 3 CMD_MOVE" << FabricLogger::End();
    }

    void handleTorqueSwitch(const barrett_hand_msgs::CommandHand& cmd_in)
    {
        if (torqueSwitch_ > 0) {
            --torqueSwitch_;
        }
        else if (torqueSwitch_ == 0) {
            for (int i = 0; i < BH_DOF; i++) {
                cmds_.push(BHCanCommand{static_cast<uint8_t>(i), BHCanCommand::CMD_MAX_TORQUE, static_cast<int32_t>(cmd_in.max_i[i])});
                m_fabric_logger << "added command: " << i << " CMD_MAX_TORQUE" << FabricLogger::End();
            }

            --torqueSwitch_;
        }

        // chack for torque switch activation
        if (fabs((q_out_[2] * 3.0) - q_out_[1]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH1;
        }
        if (fabs((q_out_[5] * 3.0) - q_out_[4]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH2;
        }
        if (fabs((q_out_[7] * 3.0) - q_out_[6]) > 0.03) {
            status_out_ |= STATUS_TORQUESWITCH3;
        }
    }

    int iter_counter_;
    void writeOutputPorts() {
        port_q_out_.write(q_out_);
        /*
        if (iter_counter_ < 0 || iter_counter_ > 1000) {
            iter_counter_ = 0;

            std::cout << getName() << " status: " <<
                    ((status_out_&STATUS_TORQUESWITCH1)?"torque_sw_1 " : "") <<
                    ((status_out_&STATUS_TORQUESWITCH2)?"torque_sw_2 " : "") <<
                    ((status_out_&STATUS_TORQUESWITCH3)?"torque_sw_3 " : "") <<
                    ((status_out_&STATUS_IDLE1)?"idle_1 " : "") <<
                    ((status_out_&STATUS_IDLE2)?"idle_2 " : "") <<
                    ((status_out_&STATUS_IDLE3)?"idle_3 " : "") <<
                    ((status_out_&STATUS_IDLE4)?"idle_4 " : "") <<
                    ((status_out_&STATUS_OVERCURRENT1)?"o_current_1 " : "") <<
                    ((status_out_&STATUS_OVERCURRENT2)?"o_current_2 " : "") <<
                    ((status_out_&STATUS_OVERCURRENT3)?"o_current_3 " : "") <<
                    ((status_out_&STATUS_OVERCURRENT4)?"o_current_4 " : "") <<
                    ((status_out_&STATUS_OVERPRESSURE1)?"o_pressure_1 " : "") <<
                    ((status_out_&STATUS_OVERPRESSURE2)?"o_pressure_2 " : "") <<
                    ((status_out_&STATUS_OVERPRESSURE3)?"o_pressure_3 " : "") <<
                    "mode: " << mode_[0] << ", " << mode_[1] << ", " << mode_[2] << ", " << mode_[3] <<
                    "  currents: " << currents_[0] << ", " << currents_[1] << ", " << currents_[2] << ", " << currents_[3] <<
                    "  pos: " << q_out_[0] << ", " << q_out_[1] << ", " << q_out_[4] << ", " << q_out_[6] <<
                    std::endl;
            //std::cout << getName() << " status: " << status_out_ << std::endl;
        }
        iter_counter_++;
        */
        port_status_out_.write(status_out_);
        port_t_out_.write(t_out_);
    }

    void calculateJointPositions()
    {
        if (constant_configuration_.size() == 4) {
            q_out_[0] = constant_configuration_[0];
            q_out_[3] = constant_configuration_[0];
            q_out_[1] = constant_configuration_[1];
            q_out_[2] = constant_configuration_[1] * 0.3333;
            q_out_[4] = constant_configuration_[2];
            q_out_[5] = constant_configuration_[2] * 0.3333;
            q_out_[6] = constant_configuration_[3];
            q_out_[7] = constant_configuration_[3] * 0.3333;
        }
        else {
            q_out_[0] = (double)(s_) * M_PI / 35840.0;
            q_out_[1] = 2.0 * M_PI / 4096.0 * (double)(jp1_) / 50.0;
            q_out_[2] = 2.0 * M_PI / 4096.0 * (double)(p1_) * (1.0/125.0 + 1.0/375.0) - q_out_[1];
            q_out_[3] = (double)(s_) * M_PI / 35840.0;
            q_out_[4] = 2.0 * M_PI * (double)(jp2_) / 4096.0 / 50.0;
            q_out_[5] = 2.0 * M_PI / 4096.0 * (double)(p2_) * (1.0/125.0 + 1.0/375.0) - q_out_[4];
            q_out_[6] = 2.0 * M_PI * (double)(jp3_) / 4096.0 / 50.0;
            q_out_[7] = 2.0 * M_PI / 4096.0 * (double)(p3_) * (1.0/125.0 + 1.0/375.0) - q_out_[6];
        }
    }
/*
    void handleFingersTemperature()
    {
        // on 1, 101, 201, 301, 401, ... step
        if ((loop_counter_ % 100) == 1)
        {
            int32_t temp[4] = {0, 0, 0, 0};
            int32_t therm[4] = {0, 0, 0, 0};
            bool allTempOk = true;
            bool oneTempTooHigh = false;

            for (int i = 0; i < 4; i++) {
                if (temp[i] > TEMP_MAX_HI || therm[i] > TEMP_MAX_HI) {
                    oneTempTooHigh = true;
                } else if (temp[i] >= TEMP_MAX_LO || therm[i] >= TEMP_MAX_LO) {
                    allTempOk = false;
                }
            }

            if (hold_ && oneTempTooHigh) {
                hold_ = false;
                cmds_.push(BHCanCommand{3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_});
            } else if (!hold_ && allTempOk) {
                hold_ = true;
                cmds_.push(BHCanCommand{3, BHCanCommand::CMD_HOLD, holdEnabled_ && hold_});
            }
        }
    }
*/
    void handleStatus(const barrett_hand_msgs::CommandHand& cmd_in)
    {
        if (mode_[0] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE1;
            if (((status_out_ & STATUS_OVERPRESSURE1) == 0) && (fabs(cmd_in.q[0] - q_out_[1]) > 0.03)) {
                status_out_ |= STATUS_OVERCURRENT1;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE1;
        }

        if (mode_[1] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE2;
            if ((status_out_&STATUS_OVERPRESSURE2) == 0 && fabs(cmd_in.q[1] - q_out_[4]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT2;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE2;
        }

        if (mode_[2] == 0 && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE3;
            if ((status_out_&STATUS_OVERPRESSURE3) == 0 && fabs(cmd_in.q[2] - q_out_[6]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT3;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE3;
        }

        if ((mode_[3] == 0 || (holdEnabled_ && fabs(cmd_in.q[3] - q_out_[3]) < 0.05)) && status_read_seq_ == SEQ_STATUS_RECV) {
            status_out_ |= STATUS_IDLE4;
            if (fabs(cmd_in.q[3]-q_out_[3]) > 0.03) {
                status_out_ |= STATUS_OVERCURRENT4;
            }
        }
        else {
            status_out_ &= ~STATUS_IDLE4;
        }

        port_max_measured_pressure_in_.read(max_measured_pressure_in_);
        // bool f1_stopped(false), f2_stopped(false), f3_stopped(false);
        // for (int j = 0; j < 24; ++j) {
        //     if ( (status_out_&STATUS_IDLE1) == 0 && !f1_stopped) {
        //         if (max_measured_pressure_in_[0] > cmd_in.max_p) {
        //             cmds_.push(BHCanCommand{0, BHCanCommand::CMD_STOP, 0});
        //             m_fabric_logger << "added command: " << 0 << " CMD_STOP" << FabricLogger::End();
        //             status_out_ |= STATUS_OVERPRESSURE1;
        //             f1_stopped = true;
        //         }
        //     }
        //     if ( (status_out_&STATUS_IDLE2) == 0 && !f2_stopped) {
        //         if (max_measured_pressure_in_[1] > cmd_in.max_p) {
        //             cmds_.push(BHCanCommand{1, BHCanCommand::CMD_STOP, 0});
        //             m_fabric_logger << "added command: " << 1 << " CMD_STOP" << FabricLogger::End();
        //             status_out_ |= STATUS_OVERPRESSURE2;
        //             f2_stopped = true;
        //         }
        //     }
        //     if ( (status_out_&STATUS_IDLE3) == 0 && !f3_stopped) {
        //         if (max_measured_pressure_in_[2] > cmd_in.max_p) {
        //             cmds_.push(BHCanCommand{2, BHCanCommand::CMD_STOP, 0});
        //             m_fabric_logger << "added command: " << 2 << " CMD_STOP" << FabricLogger::End();
        //             status_out_ |= STATUS_OVERPRESSURE3;
        //             f3_stopped = true;
        //         }
        //     }
        // }
    }

    void handleCurrents()
    {
        t_out_[0] = currents_[3];
        t_out_[1] = currents_[0];
        t_out_[2] = currents_[0];
        t_out_[3] = currents_[3];
        t_out_[4] = currents_[1];
        t_out_[5] = currents_[1];
        t_out_[6] = currents_[2];
        t_out_[7] = currents_[2];
    }

    // RTT update hook
    // This function runs every 2 ms (500 Hz).
    // Temperature is published every 100 ms (10 Hz).
    void updateHook()
    {
        if (current_state_ == STATE_NORMAL_OP) {
            iterateStateNormalOp();
        }
        else if (current_state_ == STATE_INIT_HAND) {
            iterateStateInitHand();
        }
        else {
            m_fabric_logger << "ERROR: Wrong internal state of the component: " << current_state_ << FabricLogger::End();
            Logger::log() << Logger::Error << "Wrong internal state of the component: " << current_state_ << Logger::endl;
            error();
        }
    }
};

ORO_CREATE_COMPONENT(BarrettHand)
