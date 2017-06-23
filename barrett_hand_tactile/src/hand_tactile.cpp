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

#include <barrett_hand_msgs/BHPressureState.h>
#include <barrett_hand_msgs/BHTemp.h>
#include <barrett_hand_msgs/BHPressureInfo.h>

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
#include "TactileInterface.h"
#include "tactile_geometry.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "Tactile.h"

#include "Eigen/Dense"


using std::string;
using RTT::InputPort;
using RTT::OutputPort;

class BarrettHandTactile : public RTT::TaskContext {
private:
    int32_t loop_counter_;
    TactileInterface *tact_;

    Tactile *ts_[4];

    // port variables
    barrett_hand_msgs::BHPressureState tactile_out_;
    std_msgs::Empty reset_in_;
    std_msgs::Empty calibrate_in_;
    std_msgs::Int32 filter_in_;
    barrett_hand_msgs::BHPressureInfo pressure_info_;
    Eigen::Vector4d max_pressure_out_;

    // OROCOS ports
    OutputPort<barrett_hand_msgs::BHPressureState> port_tactile_out_;
    InputPort<std_msgs::Empty> port_reset_in_;
    InputPort<std_msgs::Empty> port_calibrate_in_;
    InputPort<std_msgs::Int32> port_filter_in_;
    OutputPort<barrett_hand_msgs::BHPressureInfo> port_tactile_info_out_;
    OutputPort<Eigen::Vector4d > port_max_pressure_out_;

    // ROS parameters
    string dev_name_;
    string prefix_;

    int32_t median_filter_samples_, median_filter_max_samples_;

public:
    explicit BarrettHandTactile(const string& name):
        TaskContext(name, PreOperational),
        loop_counter_(0),
        tact_(NULL),
        median_filter_samples_(1),
        median_filter_max_samples_(8)
    {
        ts_[0] = new Tactile(median_filter_max_samples_);
        ts_[1] = new Tactile(median_filter_max_samples_);
        ts_[2] = new Tactile(median_filter_max_samples_);
        ts_[3] = new Tactile(median_filter_max_samples_);
        ts_[0]->setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[1]->setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[2]->setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[3]->setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

        this->ports()->addPort("BHPressureState", port_tactile_out_);
        this->ports()->addPort("calibrate_tactile_sensors", port_calibrate_in_);
        this->ports()->addPort("set_median_filter", port_filter_in_);
        this->ports()->addPort("tactile_info_out", port_tactile_info_out_);
        this->ports()->addPort("max_measured_pressure_out", port_max_pressure_out_);
        this->addProperty("device_name", dev_name_);
        this->addProperty("prefix", prefix_);
        max_pressure_out_.setZero();
    }

    ~BarrettHandTactile() {
    }

    void cleanupHook() {
        if (tact_ != NULL) {
            delete tact_;
        }
    }

    // RTT configure hook
    bool configureHook() {
        if (tact_ == NULL && !dev_name_.empty() && !prefix_.empty()) {
            tact_ = new TactileInterface(dev_name_);

            // tactile array info
            for (int id=0; id<4; ++id) {
                pressure_info_.sensor[id].frame_id = ts_[id]->getName();
                for (int i=0; i<24; ++i)
                {
                    pressure_info_.sensor[id].force_per_unit[i] = 1.0/256.0;
                }
            }

            for (int id=0; id<4; ++id)
            {
                for (int i=0; i<24; ++i)
                {
                    pressure_info_.sensor[id].center[i] = ts_[id]->getCenter(i);
                    pressure_info_.sensor[id].halfside1[i] = ts_[id]->getHalfside1(i);
                    pressure_info_.sensor[id].halfside2[i] = ts_[id]->getHalfside2(i);
                }
            }
            port_tactile_info_out_.setDataSample(pressure_info_);
            port_tactile_out_.setDataSample(tactile_out_);
            port_max_pressure_out_.setDataSample(max_pressure_out_);

            if (tact_->isDevOpened()) {
                return true;
            }
            return false;
        }
        return false;
    }

    // RTT start hook
    bool startHook()
    {
        return true;
    }

    void stopHook()
    {
    }

    // RTT update hook
    // This function runs every 1 ms (1000 Hz).
    // The i-th tactile data is published every 6 ms (166.66 Hz),
    // so all 4 tactiles' data is published every 25 ms (40 Hz).
    // Temperature is published every 100 ms (10 Hz).
    void updateHook()
    {
        if (port_calibrate_in_.read(calibrate_in_) == RTT::NewData) {
            for (int id=0; id<4; ++id)
            {
                ts_[id]->startCalibration();
            }
        }

        if (port_filter_in_.read(filter_in_) == RTT::NewData) {
            if (filter_in_.data >= 1 && filter_in_.data <= median_filter_max_samples_)
            {
                median_filter_samples_ = filter_in_.data;
            }
        }

        if ( (loop_counter_%10) == 1 ) {
            port_tactile_info_out_.write(pressure_info_);
        }

        if (loop_counter_ == 0)
        {
            TactileInterface::tact_array_t tact;
            tact_->getTactile(0, tact);
            ts_[0]->updatePressure(tact);
        } else if (loop_counter_ == 1)
        {
            TactileInterface::tact_array_t tact;
            tact_->getTactile(1, tact);
            ts_[1]->updatePressure(tact);
        } else if (loop_counter_ == 2)
        {
            TactileInterface::tact_array_t tact;
            tact_->getTactile(2, tact);
            ts_[2]->updatePressure(tact);
        } else if (loop_counter_ == 3)
        {
            TactileInterface::tact_array_t tact;
            tact_->getTactile(3, tact);
            ts_[3]->updatePressure(tact);
            tactile_out_.header.stamp = rtt_rosclock::host_now();

            max_pressure_out_.setZero();
            for (int i=0; i<24; ++i)
            {
                tactile_out_.finger1_tip[i] = ts_[0]->getPressure(i,median_filter_samples_);
                tactile_out_.finger2_tip[i] = ts_[1]->getPressure(i,median_filter_samples_);
                tactile_out_.finger3_tip[i] = ts_[2]->getPressure(i,median_filter_samples_);
                tactile_out_.palm_tip[i] = ts_[3]->getPressure(i,median_filter_samples_);
                for (int puck_id = 0; puck_id < 4; puck_id++) {
                    if (max_pressure_out_(puck_id) < ts_[puck_id]->getPressure(i,median_filter_samples_)) {
                        max_pressure_out_(puck_id) = ts_[puck_id]->getPressure(i,median_filter_samples_);
                    }
                }
            }

            port_max_pressure_out_.write(max_pressure_out_);
            port_tactile_out_.write(tactile_out_);
        }

        loop_counter_ = (loop_counter_+1)%4;
    }
};
ORO_CREATE_COMPONENT(BarrettHandTactile)

