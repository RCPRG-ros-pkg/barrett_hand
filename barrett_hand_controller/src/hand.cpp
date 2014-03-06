#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include <actionlib/server/simple_action_server.h>
#include <barrett_hand_controller/BHMoveAction.h>
#include <barrett_hand_controller/BHPressureState.h>
#include <barrett_hand_controller_srvs/BHPressureInfo.h>
#include <barrett_hand_controller_srvs/BHPressureInfoElement.h>
#include <barrett_hand_controller_srvs/BHResetFingers.h>
#include <barrett_hand_controller_srvs/BHGetPressureInfo.h>
#include <barrett_hand_controller_srvs/BHCalibrateTactileSensors.h>
#include <barrett_hand_controller/BHFingerVel.h>
#include <barrett_hand_controller/BHTemp.h>

#include <iostream>
#include <math.h>
#include "MotorController.h"
#include "tactile_geometry.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;

#define JP2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/50.0 * 2 * M_PI
#define P2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/(125*2.5) * 2 * M_PI

#define RAD2P(x) ((x) * 180.0/3.1416 / 140.0 * 199111.1)
#define RAD2S(x) (x * 35840.0/M_PI)

class Tactile
{
public:
	typedef double RawTactileData[24][3];

	Tactile() :
		calibration_counter_max_(40)
	{
		calibration_counter_ = calibration_counter_max_;
	}

	void setGeometry(string name, const RawTactileData &center, const RawTactileData &halfside1, const RawTactileData &halfside2, double scale = 1)
	{
		sensor_name_ = name;
		for (int i=0; i<24; ++i)
		{
			sensor_center_[i].x = center[i][0]*scale;	sensor_center_[i].y = center[i][1]*scale;	sensor_center_[i].z = center[i][2]*scale;
			sensor_halfside1_[i].x = halfside1[i][0]*scale;	sensor_halfside1_[i].y = halfside1[i][1]*scale;	sensor_halfside1_[i].z = halfside1[i][2]*scale;
			sensor_halfside2_[i].x = halfside2[i][0]*scale;	sensor_halfside2_[i].y = halfside2[i][1]*scale;	sensor_halfside2_[i].z = halfside2[i][2]*scale;
			field_[i] = 4 *	sqrt(sensor_halfside1_[i].x*sensor_halfside1_[i].x + sensor_halfside1_[i].y*sensor_halfside1_[i].y + sensor_halfside1_[i].z*sensor_halfside1_[i].z)*
					sqrt(sensor_halfside2_[i].x*sensor_halfside2_[i].x + sensor_halfside2_[i].y*sensor_halfside2_[i].y + sensor_halfside2_[i].z*sensor_halfside2_[i].z);
		}
	}

	void startCalibration()
	{
		calibration_counter_ = 0;
		for (int i=0; i<24; ++i)
		{
			offsets_[i] = 0.0;
		}
	}

	string getName()
	{
		return sensor_name_;
	}

	geometry_msgs::Vector3 getCenter(int i)
	{
		if (i>=0 && i<24)
			return sensor_center_[i];
		return geometry_msgs::Vector3();
	}

	geometry_msgs::Vector3 getHalfside1(int i)
	{
		if (i>=0 && i<24)
			return sensor_halfside1_[i];
		return geometry_msgs::Vector3();
	}

	geometry_msgs::Vector3 getHalfside2(int i)
	{
		if (i>=0 && i<24)
			return sensor_halfside2_[i];
		return geometry_msgs::Vector3();
	}

	void updatePressure(const MotorController::tact_array_t &tact)
	{
		memcpy(tact_, tact, sizeof(MotorController::tact_array_t));

		if (calibration_counter_ < calibration_counter_max_)
		{
			for (int i=0; i<24; ++i)
			{
				offsets_[i] += tact_[i];
			}
			++calibration_counter_;
			if (calibration_counter_ >= calibration_counter_max_)
			{
				for (int i=0; i<24; ++i)
				{
					offsets_[i] /= (double)calibration_counter_max_;
				}
			}
		}
	}

	int32_t getPressure(int i)
	{
		if (calibration_counter_ < calibration_counter_max_)
		{
			return 0;
		}

		double result = (double)tact_[i]-offsets_[i];
		if (result < 0)
			result = 0;
		return (int32_t)result;
	}

	int32_t getForce(int i)
	{
		if (calibration_counter_ < calibration_counter_max_)
		{
			return 0;
		}

		double result = (double)field_[i]*((double)tact_[i]-offsets_[i])*100000.0/5.0;
		if (result < 0)
			result = 0;
		return (int32_t)result;
	}

private:
	string sensor_name_;
	
	geometry_msgs::Vector3 sensor_center_[24];
	geometry_msgs::Vector3 sensor_halfside1_[24];
	geometry_msgs::Vector3 sensor_halfside2_[24];

	double field_[24];

	double offsets_[24];
	int32_t tact_[24];

	int32_t calibration_counter_;
	const int32_t calibration_counter_max_;
};

using namespace RTT;

class BarrettHand : public RTT::TaskContext{
private:
	ACTION_DEFINITION(barrett_hand_controller::BHMoveAction);

	const int TEMP_MAX_HI;
	const int TEMP_MAX_LO;
	int32_t loop_counter_;
	MotorController ctrl_;

	rtt_actionlib::RTTActionServer<barrett_hand_controller::BHMoveAction> as_;
	actionlib::ServerGoalHandle<barrett_hand_controller::BHMoveAction> gh_;
	Result result_;

	Tactile ts_[4];
	barrett_hand_controller::BHTemp temp_;
	sensor_msgs::JointState joint_states_;
	barrett_hand_controller::BHPressureState pressure_states_;
	ros::Time last_tact_read_;
	bool hold_;
	bool holdEnabled_;

	OutputPort<barrett_hand_controller::BHTemp>		temp_pub_;
	OutputPort<sensor_msgs::JointState>			joint_pub_;
	OutputPort<barrett_hand_controller::BHPressureState>	tactile_pub_;

	int resetFingersCounter_;
public:
	BarrettHand(const std::string& name):
		TaskContext(name),
		TEMP_MAX_HI(60),
		TEMP_MAX_LO(55),
		temp_pub_("BHTemp"),
		joint_pub_("joint_states"),
		tactile_pub_("BHPressureState"),
		ctrl_(string("can0")),
		loop_counter_(0),
		resetFingersCounter_(0)
	{
		ts_[0].setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[1].setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[2].setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
		ts_[3].setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

		pressure_states_.finger1_tip.resize(24);
		pressure_states_.finger2_tip.resize(24);
		pressure_states_.finger3_tip.resize(24);
		pressure_states_.palm_tip.resize(24);

		joint_states_.name.resize(8);
		joint_states_.position.resize(8);
		
		joint_states_.name[0] = "right_HandFingerOneKnuckleOneJoint";
		joint_states_.name[1] = "right_HandFingerOneKnuckleTwoJoint";
		joint_states_.name[2] = "right_HandFingerOneKnuckleThreeJoint";
		
		joint_states_.name[3] = "right_HandFingerTwoKnuckleOneJoint";
		joint_states_.name[4] = "right_HandFingerTwoKnuckleTwoJoint";
		joint_states_.name[5] = "right_HandFingerTwoKnuckleThreeJoint";
		
		joint_states_.name[6] = "right_HandFingerThreeKnuckleTwoJoint";
		joint_states_.name[7] = "right_HandFingerThreeKnuckleThreeJoint";

		// Bind action server goal and cancel callbacks (see below)
		as_.registerGoalCallback(boost::bind(&BarrettHand::goalCallback, this, _1));
		as_.registerCancelCallback(boost::bind(&BarrettHand::cancelCallback, this, _1));

		temp_.temp.resize(9);

		this->addPort(temp_pub_).doc("Sends out BH temperature.");
		this->addPort(joint_pub_).doc("Sends out BH joint states.");
		this->addPort(tactile_pub_).doc("Sends out BH tactile data.");

		this->provides()->addOperation("reset_fingers",&BarrettHand::resetFingers,this,RTT::OwnThread);
		this->provides()->addOperation("calibrate",&BarrettHand::calibrateTactileSensors,this,RTT::OwnThread);
		this->provides()->addOperation("get_pressure_info",&BarrettHand::getPressureInfo,this,RTT::OwnThread);

		// Add action server ports to this task's root service
		as_.addPorts(this->provides());
	}

	~BarrettHand()
	{
		ctrl_.stopHand();
	}

	// RTT configure hook
	bool configureHook()
	{
		return true;
	}

	// RTT start hook
	bool startHook()
	{
		holdEnabled_ = true;
		hold_ = true;
		ctrl_.setHoldPosition(0, false);
		ctrl_.setHoldPosition(1, false);
		ctrl_.setHoldPosition(2, false);
		ctrl_.setHoldPosition(3, holdEnabled_ && hold_);

/*		ctrl_.resetFinger(0);
		ctrl_.resetFinger(1);
		ctrl_.resetFinger(2);
	
		sleep(2);
	
		ctrl_.resetFinger(3);
	
		sleep(2);
*/
		// Start action server
		as_.start();
		return true;
	}

	void stopHook()
	{
		ctrl_.stopHand();
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

		// on 0, 10, 20, 30, 40, ... step
		if ( (loop_counter_%10) == 0)
		{
			//ctrl_.getPositionAll(p1, p2, p3, jp1, jp2, jp3, s);
			int32_t tmp;
			ctrl_.getPosition(0,p1,jp1);
			ctrl_.getPosition(1,p2,jp2);
			ctrl_.getPosition(2,p3,jp3);
			ctrl_.getPosition(3,s,tmp);
			joint_states_.header.stamp = ros::Time::now();

			joint_states_.position[0] = s * M_PI/ 35840.0;
			joint_states_.position[1] = JP2RAD(jp1);
			joint_states_.position[2] = P2RAD(p1);

			joint_states_.position[3] = s * M_PI/ 35840.0;
			joint_states_.position[4] = JP2RAD(jp2);
			joint_states_.position[5] = P2RAD(p2);

			joint_states_.position[6] = JP2RAD(jp3);
			joint_states_.position[7] = P2RAD(p3);
	
			joint_pub_.write(joint_states_);
		}

		// on 1, 101, 201, 301, 401, ... step
		if ( (loop_counter_%100) == 1)
		{
			int32_t temp[4], therm[4];
			ctrl_.getTemp(0, temp[0]);
			ctrl_.getTemp(1, temp[1]);
			ctrl_.getTemp(2, temp[2]);
			ctrl_.getTemp(3, temp[3]);
			ctrl_.getTherm(0, therm[0]);
			ctrl_.getTherm(1, therm[1]);
			ctrl_.getTherm(2, therm[2]);
			ctrl_.getTherm(3, therm[3]);

			if (	(temp[0] > TEMP_MAX_HI || temp[1] > TEMP_MAX_HI || temp[2] > TEMP_MAX_HI || temp[3] > TEMP_MAX_HI ||
				therm[0] > TEMP_MAX_HI || therm[1] > TEMP_MAX_HI || therm[2] > TEMP_MAX_HI || therm[3] > TEMP_MAX_HI) &&
				hold_ == true)
			{
				hold_ = false;
				ctrl_.setHoldPosition(3, holdEnabled_ && hold_);
				RTT::log(RTT::Warning) << "Temperature is too high. Disabled spread hold." << RTT::endlog();
			}

			if (	temp[0] < TEMP_MAX_LO && temp[1] < TEMP_MAX_LO && temp[2] < TEMP_MAX_LO && temp[3] < TEMP_MAX_LO &&
				therm[0] < TEMP_MAX_LO && therm[1] < TEMP_MAX_LO && therm[2] < TEMP_MAX_LO && therm[3] < TEMP_MAX_LO &&
				hold_ == false)
			{
				hold_ = true;
				ctrl_.setHoldPosition(3, holdEnabled_ && hold_);
				RTT::log(RTT::Warning) << "Temperature is lower. Enabled spread hold." << RTT::endlog();
			}

			temp_.header.stamp = ros::Time::now();
			temp_.temp[0] = temp[0];
			temp_.temp[1] = temp[1];
			temp_.temp[2] = temp[2];
			temp_.temp[3] = temp[3];
			temp_.temp[4] = therm[0];
			temp_.temp[5] = therm[1];
			temp_.temp[6] = therm[2];
			temp_.temp[7] = therm[3];
			temp_pub_.write(temp_);
		}

		int i=((loop_counter_+2)%25);
		// on 2, 27, 52, 77, 102, ... step
		if ( i == 0)
		{
			MotorController::tact_array_t tact;
			ctrl_.getTactile(0, tact);
			ts_[0].updatePressure(tact);
		}
		else if (i==6)
		{
			MotorController::tact_array_t tact;
			ctrl_.getTactile(1, tact);
			ts_[1].updatePressure(tact);
		}
		else if (i==12)
		{
			MotorController::tact_array_t tact;
			ctrl_.getTactile(2, tact);
			ts_[2].updatePressure(tact);
		}
		else if (i==18)
		{
			MotorController::tact_array_t tact;
			ctrl_.getTactile(3, tact);
			ts_[3].updatePressure(tact);

			for (int i=0; i<24; ++i)
			{
				pressure_states_.finger1_tip[i] = ts_[0].getForce(i);
				pressure_states_.finger2_tip[i] = ts_[1].getForce(i);
				pressure_states_.finger3_tip[i] = ts_[2].getForce(i);
				pressure_states_.palm_tip[i] = ts_[3].getForce(i);
			}

			tactile_pub_.write(pressure_states_);
		}

		// Pursue goal...
		if(gh_.isValid() && gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
		{
			ctrl_.getStatusAll(mode1, mode2, mode3, mode4);

			if((mode1 == 0) && (mode2 == 0) && (mode3 == 0) && (mode4 == 3))
				gh_.setSucceeded(result_);


		}

		loop_counter_ = (loop_counter_+1)%1000;

		if (resetFingersCounter_ > 0)
		{
			--resetFingersCounter_;
			if (resetFingersCounter_ == 2000)
			ctrl_.resetFinger(3);
		}
	}

	// Accept/reject goal requests here
	void goalCallback(actionlib::ServerGoalHandle<barrett_hand_controller::BHMoveAction> gh)
	{
		// Always preempt the current goal and accept the new one
		if(gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
		{
			// TODO: set the result
			//result_.actual_delay_time = rtt_ros::time::now() - current_gh_.getGoalID().stamp;
			gh_.setCanceled(result_);
		}
		gh.setAccepted();
		gh_ = gh;

		ctrl_.setMaxVel(0, RAD2P(gh_.getGoal()->fingerVel)/1000.0);
		ctrl_.setMaxVel(1, RAD2P(gh_.getGoal()->fingerVel)/1000.0);
		ctrl_.setMaxVel(2, RAD2P(gh_.getGoal()->fingerVel)/1000.0);
		ctrl_.setMaxVel(3, RAD2S(gh_.getGoal()->spreadVel)/1000.0);
				
//		cout << " : " << gh_.getGoal()->finger[0] << " - " << RAD2P(gh_.getGoal()->finger[0])
//							<< " : " << gh_.getGoal()->finger[1] << " - " << RAD2P(gh_.getGoal()->finger[1])
//							<< " : " << gh_.getGoal()->finger[2] << " - " << RAD2P(gh_.getGoal()->finger[2]) << endl;
			
		ctrl_.setTargetPos(0, RAD2P(gh_.getGoal()->finger[0]));
		ctrl_.setTargetPos(1, RAD2P(gh_.getGoal()->finger[1]));
		ctrl_.setTargetPos(2, RAD2P(gh_.getGoal()->finger[2]));
		ctrl_.setTargetPos(3, RAD2S(gh_.getGoal()->spread));

		ctrl_.moveAll();
	}

	// Handle preemption here
	void cancelCallback(actionlib::ServerGoalHandle<barrett_hand_controller::BHMoveAction> gh) {
		if(gh_ == gh && gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
			// TODO: set the result
			//result_.actual_delay_time = rtt_ros::time::now() - gh_.getGoalID().stamp;
			gh_.setCanceled(result_);
		}
	}

	bool resetFingers(barrett_hand_controller_srvs::BHResetFingers::Request  &req,
	         barrett_hand_controller_srvs::BHResetFingers::Response &res)
	{
		resetFingersCounter_ = 3000;
		ctrl_.resetFinger(0);
		ctrl_.resetFinger(1);
		ctrl_.resetFinger(2);
		return true;
	}

	bool getPressureInfo(barrett_hand_controller_srvs::BHGetPressureInfo::Request  &req,
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

	bool calibrateTactileSensors(barrett_hand_controller_srvs::BHCalibrateTactileSensors::Request  &req,
	         barrett_hand_controller_srvs::BHCalibrateTactileSensors::Response &res)
	{
		for (int id=0; id<4; ++id)
		{
			ts_[id].startCalibration();
		}
		return true;
	}


};
ORO_CREATE_COMPONENT(BarrettHand)


