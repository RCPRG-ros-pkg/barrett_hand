#include <barrett_hand_controller_srvs/BHCmd.h>
#include <barrett_hand_controller_srvs/BHPressureState.h>
#include <barrett_hand_controller_srvs/BHPressureInfo.h>
#include <barrett_hand_controller_srvs/BHPressureInfoElement.h>
#include <barrett_hand_controller_srvs/BHTemp.h>
#include <barrett_hand_controller_srvs/Empty.h>
#include <barrett_hand_controller_srvs/BHGetPressureInfo.h>
#include <barrett_hand_controller_srvs/BHMoveHand.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <ros/ros.h>

#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "rtt_rosclock/rtt_rosclock.h"

#include <iostream>
#include <math.h>
#include "MotorController.h"
#include "tactile_geometry.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "Tactile.h"

using namespace std;

#define JP2RAD(x) ((double)(x) * 1.0/4096.0 * 1.0/50.0 * 2 * M_PI)
#define P2RAD(x) ((double)(x) * 1.0/4096.0 * 1.0/(125*2.5) * 2 * M_PI)

#define RAD2P(x) ((double)(x) * 180.0/3.1416 / 140.0 * 199111.1)
#define RAD2S(x) ((double)(x) * 35840.0/M_PI)

using namespace RTT;

class BarrettHand : public RTT::TaskContext{
private:
	const int TEMP_MAX_HI;
	const int TEMP_MAX_LO;
	int32_t loop_counter_;
	MotorController *ctrl_;

	int32_t maxStaticTorque_;
	int32_t maxDynamicTorque_;
	int torqueSwitch_;

	Tactile ts_[4];
	barrett_hand_controller_srvs::BHCmd cmd_;
	barrett_hand_controller_srvs::BHTemp temp_;
	sensor_msgs::JointState joint_states_;
	barrett_hand_controller_srvs::BHPressureState pressure_states_;
	ros::Time last_tact_read_;
	bool hold_;
	bool holdEnabled_;

	InputPort<barrett_hand_controller_srvs::BHCmd>		cmd_in_;
	OutputPort<barrett_hand_controller_srvs::BHTemp>		temp_out_;
	OutputPort<sensor_msgs::JointState>			joint_out_;
	OutputPort<barrett_hand_controller_srvs::BHPressureState>	tactile_out_;
	string dev_name_;
	string prefix_;

	double sp_torque_;
	double f1_torque_;
	double f2_torque_;
	double f3_torque_;

	double move_hand_cmd_;
	double f1_target_;
	double f2_target_;
	double f3_target_;
	double sp_target_;

	double set_max_vel_;
	double f1_vel_;
	double f2_vel_;
	double f3_vel_;
	double sp_vel_;

	int32_t temp[4], therm[4];

	int resetFingersCounter_;
public:
	BarrettHand(const std::string& name):
		TaskContext(name, PreOperational),
		TEMP_MAX_HI(65),
		TEMP_MAX_LO(60),
		temp_out_("BHTemp"),
		joint_out_("joint_states"),
		tactile_out_("BHPressureState"),
		cmd_in_("BHCmd"),
		loop_counter_(0),
		resetFingersCounter_(0),
		maxStaticTorque_(4700),
		maxDynamicTorque_(700),
		sp_torque_(700),
		f1_torque_(700),
		f2_torque_(700),
		f3_torque_(700),
		torqueSwitch_(-1),
		move_hand_cmd_(false),
		set_max_vel_(false),
		ctrl_(NULL)
	{
		ts_[0].setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[1].setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[2].setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[3].setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

		temp[0] = 0;
		temp[1] = 0;
		temp[2] = 0;
		temp[3] = 0;
		therm[0] = 0;
		therm[1] = 0;
		therm[2] = 0;
		therm[3] = 0;
		
		pressure_states_.finger1_tip.resize(24);
		pressure_states_.finger2_tip.resize(24);
		pressure_states_.finger3_tip.resize(24);
		pressure_states_.palm_tip.resize(24);

		joint_states_.name.resize(8);
		joint_states_.position.resize(8);
		
		temp_.temp.resize(9);

		this->addPort(temp_out_).doc("Sends out BH temperature.");
		this->addPort(joint_out_).doc("Sends out BH joint states.");
		this->addPort(tactile_out_).doc("Sends out BH tactile data.");
		this->addPort(cmd_in_).doc("Input command.");

		this->provides()->addOperation("reset_fingers",&BarrettHand::resetFingers,this,RTT::OwnThread);
		this->provides()->addOperation("reset_fingers_ros",&BarrettHand::resetFingersRos,this,RTT::OwnThread);
		this->provides()->addOperation("calibrate",&BarrettHand::calibrateTactileSensors,this,RTT::OwnThread);
		this->provides()->addOperation("calibrate_ros",&BarrettHand::calibrateTactileSensorsRos,this,RTT::OwnThread);
		this->provides()->addOperation("get_pressure_info_ros",&BarrettHand::getPressureInfoRos,this,RTT::OwnThread);
		this->provides()->addOperation("set_max_torque",&BarrettHand::setMaxTorque,this,RTT::OwnThread);
		this->provides()->addOperation("set_max_vel",&BarrettHand::setMaxVel,this,RTT::OwnThread);
		this->provides()->addOperation("move_hand",&BarrettHand::moveHand,this,RTT::OwnThread);
		this->provides()->addOperation("move_hand_ros",&BarrettHand::moveHandRos,this,RTT::OwnThread);

		this->addProperty("device_name", dev_name_);
		this->addProperty("prefix", prefix_);
	}

	~BarrettHand()
	{
		ctrl_->stopHand();
	}

	void cleanupHook()
	{
		if (ctrl_ != NULL)
		{
			delete ctrl_;
		}
	}

	// RTT configure hook
	bool configureHook()
	{
		if (ctrl_ == NULL && !dev_name_.empty() && !prefix_.empty())
		{
			ctrl_ = new MotorController(dev_name_);

			joint_states_.name[0] = prefix_ + "_HandFingerOneKnuckleOneJoint";
			joint_states_.name[1] = prefix_ + "_HandFingerOneKnuckleTwoJoint";
			joint_states_.name[2] = prefix_ + "_HandFingerOneKnuckleThreeJoint";
		
			joint_states_.name[3] = prefix_ + "_HandFingerTwoKnuckleOneJoint";
			joint_states_.name[4] = prefix_ + "_HandFingerTwoKnuckleTwoJoint";
			joint_states_.name[5] = prefix_ + "_HandFingerTwoKnuckleThreeJoint";
		
			joint_states_.name[6] = prefix_ + "_HandFingerThreeKnuckleTwoJoint";
			joint_states_.name[7] = prefix_ + "_HandFingerThreeKnuckleThreeJoint";

			return ctrl_->isDevOpened();
		}
		return false;
	}

	// RTT start hook
	bool startHook()
	{
		holdEnabled_ = true;
		hold_ = true;
		ctrl_->setHoldPosition(0, false);
		ctrl_->setHoldPosition(1, false);
		ctrl_->setHoldPosition(2, false);
		ctrl_->setHoldPosition(3, holdEnabled_ && hold_);

		ctrl_->setMaxVel(0, RAD2P(1)/1000.0);
		ctrl_->setMaxVel(1, RAD2P(1)/1000.0);
		ctrl_->setMaxVel(2, RAD2P(1)/1000.0);
		ctrl_->setMaxVel(3, RAD2S(0.7)/1000.0);

		return true;
	}

	void stopHook()
	{
		ctrl_->stopHand();
	}

	// RTT update hook
	// This function runs every 1 ms (1000 Hz).
	// Joint state is published every 10 ms (100 Hz).
	// The i-th tactile data is published every 6 ms (166.66 Hz),
	// so all 4 tactiles' data is published every 25 ms (40 Hz).
	// Temperature is published every 100 ms (10 Hz).
	void updateHook()
	{
		int32_t p1, p2, p3, jp1, jp2, jp3, jp4, s, mode1, mode2, mode3, mode4;

		if (move_hand_cmd_)
		{
			move_hand_cmd_ = false;
			ctrl_->setMaxTorque(0, maxStaticTorque_);
			ctrl_->setMaxTorque(1, maxStaticTorque_);
			ctrl_->setMaxTorque(2, maxStaticTorque_);
			ctrl_->setMaxTorque(3, maxStaticTorque_);
			torqueSwitch_ = 5;

			ctrl_->setTargetPos(0, RAD2P(f1_target_));
			ctrl_->setTargetPos(1, RAD2P(f2_target_));
			ctrl_->setTargetPos(2, RAD2P(f3_target_));
			ctrl_->setTargetPos(3, RAD2S(sp_target_));
			ctrl_->moveAll();
		}

		if (set_max_vel_)
		{
			set_max_vel_ = false;
			ctrl_->setMaxVel(0, RAD2P(f1_vel_)/1000.0);
			ctrl_->setMaxVel(1, RAD2P(f2_vel_)/1000.0);
			ctrl_->setMaxVel(2, RAD2P(f3_vel_)/1000.0);
			ctrl_->setMaxVel(3, RAD2S(sp_vel_)/1000.0);
		}

		// on 0, 10, 20, 30, 40, ... step
		if ( (loop_counter_%10) == 0)
		{
			ctrl_->getPositionAll(p1, p2, p3, jp1, jp2, jp3, s);

			joint_states_.header.stamp = rtt_rosclock::host_rt_now();

			joint_states_.position[0] = (double)s * M_PI/ 35840.0;
			joint_states_.position[1] = JP2RAD(jp1);
			joint_states_.position[2] = P2RAD(p1);

			joint_states_.position[3] = (double)s * M_PI/ 35840.0;
			joint_states_.position[4] = JP2RAD(jp2);
			joint_states_.position[5] = P2RAD(p2);

			joint_states_.position[6] = JP2RAD(jp3);
			joint_states_.position[7] = P2RAD(p3);

			joint_out_.write(joint_states_);
		}

		// on 1, 101, 201, 301, 401, ... step
		if ( (loop_counter_%100) == 1)
		{
			ctrl_->getTemp(0, temp[0]);
			ctrl_->getTemp(1, temp[1]);
			ctrl_->getTemp(2, temp[2]);
			ctrl_->getTemp(3, temp[3]);
			ctrl_->getTherm(0, therm[0]);
			ctrl_->getTherm(1, therm[1]);
			ctrl_->getTherm(2, therm[2]);
			ctrl_->getTherm(3, therm[3]);

			if (	(temp[0] > TEMP_MAX_HI || temp[1] > TEMP_MAX_HI || temp[2] > TEMP_MAX_HI || temp[3] > TEMP_MAX_HI ||
				therm[0] > TEMP_MAX_HI || therm[1] > TEMP_MAX_HI || therm[2] > TEMP_MAX_HI || therm[3] > TEMP_MAX_HI) &&
				hold_ == true)
			{
				hold_ = false;
				ctrl_->setHoldPosition(3, holdEnabled_ && hold_);
				RTT::log(RTT::Warning) << "Temperature is too high. Disabled spread hold." << RTT::endlog();
			}

			if (	temp[0] < TEMP_MAX_LO && temp[1] < TEMP_MAX_LO && temp[2] < TEMP_MAX_LO && temp[3] < TEMP_MAX_LO &&
				therm[0] < TEMP_MAX_LO && therm[1] < TEMP_MAX_LO && therm[2] < TEMP_MAX_LO && therm[3] < TEMP_MAX_LO &&
				hold_ == false)
			{
				hold_ = true;
				ctrl_->setHoldPosition(3, holdEnabled_ && hold_);
				RTT::log(RTT::Warning) << "Temperature is lower. Enabled spread hold." << RTT::endlog();
			}

			temp_.header.stamp = rtt_rosclock::host_rt_now();
			temp_.temp[0] = temp[0];
			temp_.temp[1] = temp[1];
			temp_.temp[2] = temp[2];
			temp_.temp[3] = temp[3];
			temp_.temp[4] = therm[0];
			temp_.temp[5] = therm[1];
			temp_.temp[6] = therm[2];
			temp_.temp[7] = therm[3];
			temp_out_.write(temp_);
		}

		int i=((loop_counter_+2)%25);
		// on 2, 27, 52, 77, 102, ... step
		if ( i == 0)
		{
			MotorController::tact_array_t tact;
			ctrl_->getTactile(0, tact);
			ts_[0].updatePressure(tact);
		}
		else if (i==6)
		{
			MotorController::tact_array_t tact;
			ctrl_->getTactile(1, tact);
			ts_[1].updatePressure(tact);
		}
		else if (i==12)
		{
			MotorController::tact_array_t tact;
			ctrl_->getTactile(2, tact);
			ts_[2].updatePressure(tact);
		}
		else if (i==18)
		{
			MotorController::tact_array_t tact;
			ctrl_->getTactile(3, tact);
			ts_[3].updatePressure(tact);

			for (int i=0; i<24; ++i)
			{
				pressure_states_.finger1_tip[i] = ts_[0].getForce(i);
				pressure_states_.finger2_tip[i] = ts_[1].getForce(i);
				pressure_states_.finger3_tip[i] = ts_[2].getForce(i);
				pressure_states_.palm_tip[i] = ts_[3].getForce(i);
			}

			tactile_out_.write(pressure_states_);
		}

		if (torqueSwitch_>0)
		{
			--torqueSwitch_;
		}
		else if (torqueSwitch_ == 0)
		{
			ctrl_->setMaxTorque(0, f1_torque_);
			ctrl_->setMaxTorque(1, f2_torque_);
			ctrl_->setMaxTorque(2, f3_torque_);
			ctrl_->setMaxTorque(3, sp_torque_);
			--torqueSwitch_;
		}

		loop_counter_ = (loop_counter_+1)%1000;

		if (resetFingersCounter_ > 0)
		{
			--resetFingersCounter_;
			if (resetFingersCounter_ == 2500)
			{
				ctrl_->resetFinger(0);
				ctrl_->resetFinger(1);
				ctrl_->resetFinger(2);
			}
			if (resetFingersCounter_ == 2000)
				ctrl_->resetFinger(3);
		}
	}

	void resetFingers()
	{
		resetFingersCounter_ = 3000;
	}

	bool resetFingersRos(barrett_hand_controller_srvs::Empty::Request &req, barrett_hand_controller_srvs::Empty::Response &res)
	{
		resetFingers();
		return true;
	}

	bool getPressureInfoRos(barrett_hand_controller_srvs::BHGetPressureInfo::Request  &req,
	         barrett_hand_controller_srvs::BHGetPressureInfo::Response &res)
	{
		res.info.sensor.resize(4);
		for (int id=0; id<4; ++id)
		{
			res.info.sensor[id].frame_id = ts_[id].getName();
			res.info.sensor[id].center.resize(24);
			res.info.sensor[id].halfside1.resize(24);
			res.info.sensor[id].halfside2.resize(24);
			res.info.sensor[id].force_per_unit.resize(24);
			for (int i=0; i<24; ++i)
			{
				res.info.sensor[id].force_per_unit[i] = 1.0/256.0;
			}
		}

		for (int id=0; id<4; ++id)
		{
			for (int i=0; i<24; ++i)
			{
				res.info.sensor[id].center[i] = ts_[id].getCenter(i);
				res.info.sensor[id].halfside1[i] = ts_[id].getHalfside1(i);
				res.info.sensor[id].halfside2[i] = ts_[id].getHalfside2(i);
			}
		}

		return true;
	}

	void calibrateTactileSensors()
	{
		for (int id=0; id<4; ++id)
		{
			ts_[id].startCalibration();
		}
	}

	bool calibrateTactileSensorsRos(barrett_hand_controller_srvs::Empty::Request &req,
	         barrett_hand_controller_srvs::Empty::Response &res)
	{
		calibrateTactileSensors();

		return true;
	}

	bool setMaxTorque(double f1_t, double f2_t, double f3_t, double sp_t)
	{
		f1_torque_ = f1_t;
		f2_torque_ = f2_t;
		f3_torque_ = f3_t;
		sp_torque_ = sp_t;

		return true;
	}

	bool setMaxVel(double f1_s, double f2_s, double f3_s, double sp_s)
	{
		set_max_vel_ = true;
		f1_vel_ = f1_s;
		f2_vel_ = f2_s;
		f3_vel_ = f3_s;
		sp_vel_ = sp_s;

		return true;
	}

	bool moveHand(double f1, double f2, double f3, double sp)
	{
		move_hand_cmd_ = true;
		f1_target_ = f1;
		f2_target_ = f2;
		f3_target_ = f3;
		sp_target_ = sp;

		return true;
	}

	bool moveHandRos(barrett_hand_controller_srvs::BHMoveHand::Request  &req,
	         barrett_hand_controller_srvs::BHMoveHand::Response &res)
	{
		setMaxTorque(req.f1_torque, req.f2_torque, req.f3_torque, req.sp_torque);
		setMaxVel(req.f1_speed, req.f2_speed, req.f3_speed, req.sp_speed);		
		res.result = moveHand(req.f1, req.f2, req.f3, req.sp);
		return true;
	}

};
ORO_CREATE_COMPONENT(BarrettHand)

