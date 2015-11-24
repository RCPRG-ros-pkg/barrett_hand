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

#include "barrett_hand_controller_msgs/BHMoveAction.h"
#include "barrett_hand_controller_msgs/BHMoveGoal.h"

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>
#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "rtt_rosclock/rtt_rosclock.h"

#include <iostream>
#include <string>
#include <map>
#include <math.h>
#include "MotorController.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "Eigen/Dense"

using std::string;
using std::map;

using RTT::InputPort;
using RTT::OutputPort;

class BarrettHandMoveAction : public RTT::TaskContext {
private:
	const int BH_DOF;
	enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
		STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
		STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
		STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

	typedef actionlib::ServerGoalHandle<barrett_hand_controller_msgs::BHMoveAction> GoalHandle;
	typedef boost::shared_ptr<const barrett_hand_controller_msgs::BHMoveGoal> Goal;

	rtt_actionlib::RTTActionServer<barrett_hand_controller_msgs::BHMoveAction> as_;
	GoalHandle active_goal_;
	barrett_hand_controller_msgs::BHMoveFeedback feedback_;


	Eigen::VectorXd q_out_;
	Eigen::VectorXd v_out_;
	Eigen::VectorXd t_out_;
	double mp_out_;
	int32_t hold_out_;

	uint32_t status_in_;
	InputPort<uint32_t> port_status_in_;

	OutputPort<Eigen::VectorXd> port_q_out_;
	OutputPort<Eigen::VectorXd> port_v_out_;
	OutputPort<Eigen::VectorXd> port_t_out_;
	OutputPort<double> port_mp_out_;
	OutputPort<int32_t> port_hold_out_;

	string prefix_;
	map<string, int> dof_map;

	int action_start_counter_;

public:
	explicit BarrettHandMoveAction(const string& name):
		TaskContext(name, PreOperational),
		BH_DOF(4)
	{
		this->ports()->addPort("status_in", port_status_in_);
		this->ports()->addPort("q_out", port_q_out_);
		this->ports()->addPort("v_out", port_v_out_);
		this->ports()->addPort("t_out", port_t_out_);
		this->ports()->addPort("mp_out", port_mp_out_);
		this->ports()->addPort("hold_out", port_hold_out_);

		as_.addPorts(this->provides());
		as_.registerGoalCallback(boost::bind(&BarrettHandMoveAction::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&BarrettHandMoveAction::cancelCB, this, _1));

		this->addProperty("prefix", prefix_);
	}

	~BarrettHandMoveAction() {
	}

	void cleanupHook() {
	}

	// RTT configure hook
	bool configureHook() {
		if (!prefix_.empty()) {
			dof_map[prefix_ + "_HandFingerOneKnuckleTwoJoint"] = 0;
			dof_map[prefix_ + "_HandFingerTwoKnuckleTwoJoint"] = 1;
			dof_map[prefix_ + "_HandFingerThreeKnuckleTwoJoint"] = 2;
			dof_map[prefix_ + "_HandFingerOneKnuckleOneJoint"] = 3;

			q_out_.resize(BH_DOF);
			v_out_.resize(BH_DOF);
			t_out_.resize(BH_DOF);

			port_q_out_.setDataSample(q_out_);
			port_v_out_.setDataSample(v_out_);
			port_t_out_.setDataSample(t_out_);

			feedback_.name.resize(4);
			feedback_.name[0] = prefix_ + "_HandFingerOneKnuckleTwoJoint";
			feedback_.name[1] = prefix_ + "_HandFingerTwoKnuckleTwoJoint";
			feedback_.name[2] = prefix_ + "_HandFingerThreeKnuckleTwoJoint";
			feedback_.name[3] = prefix_ + "_HandFingerOneKnuckleOneJoint";
			feedback_.torque_switch.resize(4);
			feedback_.pressure_stop.resize(4);
			feedback_.current_stop.resize(4);
			feedback_.idle.resize(4);

			return true;
		}

		std::cout << "ERROR: BarrettHandMoveAction: no prefix" << std::endl;
		return false;
	}

	// RTT start hook
	bool startHook() {
		action_start_counter_ = 0;
		if (as_.ready()) {
			as_.start();
		} else {
			return false;
		}
		return true;
	}

	void stopHook() {
	}

	bool allPucksIdle(uint32_t status) {
		return ((status & STATUS_IDLE1) != 0) && ((status & STATUS_IDLE2) != 0) && ((status & STATUS_IDLE3) != 0) && ((status & STATUS_IDLE4) != 0);
	}

	void updateHook() {
		// ignore the status at the beginning of the movement
		action_start_counter_--;
		if (action_start_counter_ > 0)
			return;

		if (port_status_in_.readNewest(status_in_) == RTT::NewData && active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
			ros::Time now = rtt_rosclock::host_now();

			feedback_.header.stamp = now;
			feedback_.torque_switch[0] = ((status_in_ & STATUS_TORQUESWITCH1) != 0);
			feedback_.torque_switch[1] = ((status_in_ & STATUS_TORQUESWITCH2) != 0);
			feedback_.torque_switch[2] = ((status_in_ & STATUS_TORQUESWITCH3) != 0);
			feedback_.torque_switch[3] = 0;
			feedback_.pressure_stop[0] = ((status_in_ & STATUS_OVERPRESSURE1) != 0);
			feedback_.pressure_stop[1] = ((status_in_ & STATUS_OVERPRESSURE2) != 0);
			feedback_.pressure_stop[2] = ((status_in_ & STATUS_OVERPRESSURE3) != 0);
			feedback_.pressure_stop[3] = 0;
			feedback_.current_stop[0] = ((status_in_ & STATUS_OVERCURRENT1) != 0);
			feedback_.current_stop[1] = ((status_in_ & STATUS_OVERCURRENT2) != 0);
			feedback_.current_stop[2] = ((status_in_ & STATUS_OVERCURRENT3) != 0);
			feedback_.current_stop[3] = ((status_in_ & STATUS_OVERCURRENT4) != 0);
			feedback_.idle[0] = ((status_in_ & STATUS_IDLE1) != 0);
			feedback_.idle[1] = ((status_in_ & STATUS_IDLE2) != 0);
			feedback_.idle[2] = ((status_in_ & STATUS_IDLE3) != 0);
			feedback_.idle[3] = ((status_in_ & STATUS_IDLE4) != 0);

			active_goal_.publishFeedback(feedback_);

			if (allPucksIdle(status_in_)) {
				barrett_hand_controller_msgs::BHMoveResult res;
				res.error_code = barrett_hand_controller_msgs::BHMoveResult::SUCCESSFUL;
				res.torque_switch.resize(4);
				res.pressure_stop.resize(4);
				res.current_stop.resize(4);
				res.torque_switch[0] = ((status_in_ & STATUS_TORQUESWITCH1) != 0);
				res.torque_switch[1] = ((status_in_ & STATUS_TORQUESWITCH2) != 0);
				res.torque_switch[2] = ((status_in_ & STATUS_TORQUESWITCH3) != 0);
				res.torque_switch[3] = 0;
				res.pressure_stop[0] = ((status_in_ & STATUS_OVERPRESSURE1) != 0);
				res.pressure_stop[1] = ((status_in_ & STATUS_OVERPRESSURE2) != 0);
				res.pressure_stop[2] = ((status_in_ & STATUS_OVERPRESSURE3) != 0);
				res.pressure_stop[3] = 0;
				res.current_stop[0] = ((status_in_ & STATUS_OVERCURRENT1) != 0);
				res.current_stop[1] = ((status_in_ & STATUS_OVERCURRENT2) != 0);
				res.current_stop[2] = ((status_in_ & STATUS_OVERCURRENT3) != 0);
				res.current_stop[3] = ((status_in_ & STATUS_OVERCURRENT4) != 0);
				active_goal_.setSucceeded(res);
			}
		}
	}

private:
	void goalCB(GoalHandle gh) {
		// cancel active goal
		if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
			active_goal_.setCanceled();
		}

		Goal g = gh.getGoal();

		if (g->name.size() != BH_DOF || g->q.size() != BH_DOF || g->v.size() != BH_DOF || g->t.size() != BH_DOF)
		{
			barrett_hand_controller_msgs::BHMoveResult res;
			res.error_code = barrett_hand_controller_msgs::BHMoveResult::INVALID_GOAL;
			gh.setRejected(res);
			return;
		}

                for (int i = 0; i < g->name.size(); i++) {
			map<string, int>::iterator dof_idx_it = dof_map.find(g->name[i]);
			if (dof_idx_it == dof_map.end()) {
				barrett_hand_controller_msgs::BHMoveResult res;
				res.error_code = barrett_hand_controller_msgs::BHMoveResult::INVALID_DOF_NAME;
				gh.setRejected(res);
				return;
			}
			q_out_[dof_idx_it->second] = g->q[i];
			v_out_[dof_idx_it->second] = g->v[i];
			t_out_[dof_idx_it->second] = g->t[i];
		}

		mp_out_ = g->maxPressure;
		hold_out_ = g->hold;

		port_v_out_.write(v_out_);
		port_t_out_.write(t_out_);
		port_mp_out_.write(mp_out_);
		port_hold_out_.write(hold_out_);
		port_q_out_.write(q_out_);

		action_start_counter_ = 10;
		gh.setAccepted();
		active_goal_ = gh;
	}

	void cancelCB(GoalHandle gh) {
		if (active_goal_ == gh) {
			active_goal_.setCanceled();
		}
	}
};
ORO_CREATE_COMPONENT(BarrettHandMoveAction)

