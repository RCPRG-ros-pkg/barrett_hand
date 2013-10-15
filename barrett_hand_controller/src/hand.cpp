
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include <barrett_hand_controller/BHMoveAction.h>
#include <barrett_hand_controller/BHPressureState.h>
#include <barrett_hand_controller/BHPressureInfo.h>
#include <barrett_hand_controller/BHPressureInfoElement.h>
#include <barrett_hand_controller/BHGetPressureInfo.h>

#include <iostream>
#include <math.h>
#include "MotorController.h"

using namespace std;

#define JP2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/50.0 * 2 * M_PI
#define P2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/(125*2.5) * 2 * M_PI

#define RAD2P(x) ((x) * 180.0/3.1416 / 140.0 * 199111.1)
#define RAD2S(x) (x * 35840.0/M_PI)

class Hand {
public:
	Hand(string port) : 
			ctrl_(port),
			as_(nh_, "move_hand", false)
	{
		joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
		
		tactile_pub_ = nh_.advertise<barrett_hand_controller::BHPressureState>("barrett_hand_controller/BHPressureState", 1);
		
		pressure_info_service_ = nh_.advertiseService("barrett_hand_controller/get_pressure_info", getPressureInfo);

		as_.start();
		
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
	}
	
	~Hand() {
	
	}
	
	void run() {
	
		ctrl_.resetFinger(0);
		ctrl_.resetFinger(1);
		ctrl_.resetFinger(2);
	
		sleep(2);
	
		ctrl_.resetFinger(3);
	
		sleep(2);

		ros::Rate loop_rate(100);
		last_tact_read_ = ros::Time::now();

		int32_t loop_counter = 0;
		while(ros::ok()) {

			int32_t p1, p2, p3, jp1, jp2, jp3, jp4, s, mode1, mode2, mode3, mode4;
	
			ctrl_.getPositionAll(p1, p2, p3, jp1, jp2, jp3, s);
	
			joint_states_.header.stamp = ros::Time::now();
	
			joint_states_.position[0] = s * M_PI/ 35840.0;
			joint_states_.position[1] = JP2RAD(jp1);
			joint_states_.position[2] = P2RAD(p1);
	
			joint_states_.position[3] = s * M_PI/ 35840.0;
			joint_states_.position[4] = JP2RAD(jp2);
			joint_states_.position[5] = P2RAD(p2);
	
			joint_states_.position[6] = JP2RAD(jp3);
			joint_states_.position[7] = P2RAD(p3);
	
			joint_pub_.publish(joint_states_);
		
			ros::Duration time_diff = joint_states_.header.stamp - last_tact_read_;
			// tactile data is updated with 40Hz freq
			// we read tactile data with bigger freq (50Hz) to get all changes
			if (time_diff >= ros::Duration(0.020))
			{
				last_tact_read_ = joint_states_.header.stamp - (time_diff - ros::Duration(0.025));

				MotorController::tact_array_t tact[4];
				ctrl_.getTactile(0, tact[0]);
				ctrl_.getTactile(1, tact[1]);
				ctrl_.getTactile(2, tact[2]);
				ctrl_.getTactile(3, tact[3]);
				if (memcmp(&tact_, &tact, sizeof(MotorController::tact_array_t)*4) != 0)
				{
					memcpy(&tact_, &tact, sizeof(MotorController::tact_array_t)*4);

					pressure_states_.header.stamp = joint_states_.header.stamp;
					for (int i=0; i<24; ++i)
					{
						pressure_states_.finger1_tip[i] = tact[0][i];
						pressure_states_.finger2_tip[i] = tact[1][i];
						pressure_states_.finger3_tip[i] = tact[2][i];
						pressure_states_.palm_tip[i] = tact[3][i];
						//cout<<tact[i]<<" ";
					}
					tactile_pub_.publish(pressure_states_);
					//cout<<endl;
				}
			}

			ros::spinOnce();

			if(as_.isNewGoalAvailable()) {
				gh_ = as_.acceptNewGoal();
				
				ctrl_.setMaxVel(0, RAD2P(gh_->fingerVel)/1000.0);
				ctrl_.setMaxVel(1, RAD2P(gh_->fingerVel)/1000.0);
				ctrl_.setMaxVel(2, RAD2P(gh_->fingerVel)/1000.0);
				ctrl_.setMaxVel(3, RAD2S(gh_->spreadVel)/1000.0);
				
				cout << " : " << gh_->finger[0] << " - " << RAD2P(gh_->finger[0])
									<< " : " << gh_->finger[1] << " - " << RAD2P(gh_->finger[1])
									<< " : " << gh_->finger[2] << " - " << RAD2P(gh_->finger[2]) << endl;
			
				ctrl_.setTargetPos(0, RAD2P(gh_->finger[0]));
				ctrl_.setTargetPos(1, RAD2P(gh_->finger[1]));
				ctrl_.setTargetPos(2, RAD2P(gh_->finger[2]));
				ctrl_.setTargetPos(3, RAD2S(gh_->spread));
	
				ctrl_.moveAll();
			}
		
			ctrl_.getStatusAll(mode1, mode2, mode3, mode4);

			if(as_.isActive()) {
				if((mode1 == 0) && (mode2 == 0) && (mode3 == 0) && (mode4 == 3))
					as_.setSucceeded(result_);
			
				//cout << " : " << mode1 << " : " << mode2 << " : " << mode3 << " : " << mode4 <<endl;
			}
		
			loop_rate.sleep();
		}
	
		ctrl_.stopHand();
	}
	
	static bool getPressureInfo(barrett_hand_controller::BHGetPressureInfo::Request  &req,
	         barrett_hand_controller::BHGetPressureInfo::Response &res)
	{
		res.info.sensor.resize(4);
		for (int id=0; id<4; ++id)
		{
			res.info.sensor[id].frame_id = string(pressureSensorName_[id]);
			res.info.sensor[id].center.resize(24);
			res.info.sensor[id].halfside1.resize(24);
			res.info.sensor[id].halfside2.resize(24);
			res.info.sensor[id].force_per_unit.resize(24);
			for (int i=0; i<24; ++i)
			{
				res.info.sensor[id].force_per_unit[i] = 1.0/256.0;
			}
		}

		for (int id=0; id<3; ++id)
		{
			for (int i=0; i<24; ++i)
			{
				res.info.sensor[id].center[i].x = finger_sensor_center_[i][0]*0.001;
				res.info.sensor[id].center[i].y = finger_sensor_center_[i][1]*0.001;
				res.info.sensor[id].center[i].z = finger_sensor_center_[i][2]*0.001;
				res.info.sensor[id].halfside1[i].x = finger_sensor_halfside1_[i][0]*0.001;
				res.info.sensor[id].halfside1[i].y = finger_sensor_halfside1_[i][1]*0.001;
				res.info.sensor[id].halfside1[i].z = finger_sensor_halfside1_[i][2]*0.001;
				res.info.sensor[id].halfside2[i].x = finger_sensor_halfside2_[i][0]*0.001;
				res.info.sensor[id].halfside2[i].y = finger_sensor_halfside2_[i][1]*0.001;
				res.info.sensor[id].halfside2[i].z = finger_sensor_halfside2_[i][2]*0.001;
			}
		}

		for (int i=0; i<24; ++i)
		{
			res.info.sensor[3].center[i].x = palm_sensor_center_[i][0]*0.001;
			res.info.sensor[3].center[i].y = palm_sensor_center_[i][1]*0.001;
			res.info.sensor[3].center[i].z = palm_sensor_center_[i][2]*0.001;
			res.info.sensor[3].halfside1[i].x = palm_sensor_halfside1_[i][0]*0.001;
			res.info.sensor[3].halfside1[i].y = palm_sensor_halfside1_[i][1]*0.001;
			res.info.sensor[3].halfside1[i].z = palm_sensor_halfside1_[i][2]*0.001;
			res.info.sensor[3].halfside2[i].x = palm_sensor_halfside2_[i][0]*0.001;
			res.info.sensor[3].halfside2[i].y = palm_sensor_halfside2_[i][1]*0.001;
			res.info.sensor[3].halfside2[i].z = palm_sensor_halfside2_[i][2]*0.001;
		}

		return true;
	}

private:
	
	bool checkGoal(barrett_hand_controller::BHMoveGoal::ConstPtr g) {
		for(unsigned int i = 0; i < 3; i++) {
			if((g->finger[i] < 0.0) || (g->finger[i] > 2.4))
				return false;
		}
		return true;
	}

	static const string pressureSensorName_[4];
 	static const double palm_sensor_center_[24][3];
 	static const double palm_sensor_halfside1_[24][3];
 	static const double palm_sensor_halfside2_[24][3];
 	static const double finger_sensor_center_[24][3];
 	static const double finger_sensor_halfside1_[24][3];
 	static const double finger_sensor_halfside2_[24][3];

	MotorController ctrl_;
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<barrett_hand_controller::BHMoveAction> as_;
	ros::ServiceServer pressure_info_service_;
	barrett_hand_controller::BHMoveGoal::ConstPtr gh_;
	barrett_hand_controller::BHMoveResult result_;
	ros::Publisher joint_pub_;
	ros::Publisher tactile_pub_;
	sensor_msgs::JointState joint_states_;
	barrett_hand_controller::BHPressureState pressure_states_;
	ros::Time last_tact_read_;
	MotorController::tact_array_t tact_[4];
};

const string Hand::pressureSensorName_[4] =
{
	"finger1_tip_info",
	"finger2_tip_info",
	"finger3_tip_info",
	"palm_info",
};

const double Hand::palm_sensor_center_[24][3] = {	// in mm
	{ 22, 15.9, 77.5 },
	{ 11, 15.9, 77.5 },
	{ 0, 15.9, 77.5 },
	{ -11, 15.9, 77.5 },
	{ -22, 15.9, 77.5 },
	{ 33, 5.3, 77.5 },
	{ 22, 5.3, 77.5 },
	{ 11, 5.3, 77.5 },
	{ 0, 5.3, 77.5 },
	{ -11, 5.3, 77.5 },
	{ -22, 5.3, 77.5 },
	{ -33, 5.3, 77.5 },
	{ 33, -5.3, 77.5 },
	{ 22, -5.3, 77.5 },
	{ 11, -5.3, 77.5 },
	{ 0, -5.3, 77.5 },
	{ -11, -5.3, 77.5 },
	{ -22, -5.3, 77.5 },
	{ -33, -5.3, 77.5 },
	{ 22, -15.9, 77.5 },
	{ 11, -15.9, 77.5 },
	{ 0, -15.9, 77.5 },
	{ -11, -15.9, 77.5 },
	{ -22, -15.9, 77.5 }
};

const double Hand::palm_sensor_halfside1_[24][3] = {	// in mm
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 }
};

const double Hand::palm_sensor_halfside2_[24][3] = {	// in mm
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 }
};

const double Hand::finger_sensor_center_[24][3] = {	// in mm
	{ 22.25, -9.5, 5.2 },
	{ 22.25, -9.5, 0 },
	{ 22.25, -9.5, -5.2 },
	{ 28.25, -9.5, 5.2 },
	{ 28.25, -9.5, 0 },
	{ 28.25, -9.5, -5.2 },
	{ 34.2484, -9.41371, 5.2 },
	{ 34.2484, -9.41371, 0 },
	{ 34.2484, -9.41371, -5.2 },
	{ 40.2349, -9.05695, 5.2 },
	{ 40.2349, -9.05695, 0 },
	{ 40.2349, -9.05695, -5.2 },
	{ 46.1912, -8.35887, 5.2 },
	{ 46.1912, -8.35887, 0 },
	{ 46.1912, -8.35887, -5.2 },
	{ 51.0813, -7.1884, 5.2 },
	{ 51.0813, -7.1884, 0 },
	{ 51.0813, -7.1884, -5.2 },
	{ 53.8108, -5.14222, 5.2 },
	{ 53.8108, -5.14222, 0 },
	{ 53.8108, -5.14222, -5.2 },
	{ 55.4163, -2.13234, 5.2 },
	{ 55.4163, -2.13234, 0 },
	{ 55.4163, -2.13234, -5.2 }
};

const double Hand::finger_sensor_halfside1_[24][3] = {	// in mm
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.399575, 1.4367, 0 },
	{ 0.399575, 1.4367, 0 },
	{ 0.399575, 1.4367, 0 }
};

const double Hand::finger_sensor_halfside2_[24][3] = {	// in mm
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 }
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "hand");
	
	Hand hand(string("can0"));
	hand.run();

	return 0;
}
