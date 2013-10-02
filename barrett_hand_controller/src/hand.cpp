
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include <barrett_hand_controller/BHMoveAction.h>

#include <iostream>
#include <math.h>
#include "MotorController.h"

#define JP2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/50.0 * 2 * M_PI
#define P2RAD(x) (double)(x) * 1.0/4096.0 * 1.0/(125*2.5) * 2 * M_PI

#define RAD2P(x) ((x) * 180.0/3.1416 / 140.0 * 199111.1)
#define RAD2S(x) (x * 35840.0/M_PI)
class Hand {
public:
	Hand(std::string port) : 
			ctrl_(port),
			as_(nh_, "move_hand", false)
	{
		joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
		
		as_.start();
		
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
	
	while(ros::ok()) {
		
		int32_t p1, p2, p3, jp1, jp2, jp3, s, mode1, mode2, mode3, mode4;
		
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
		
		ros::spinOnce();
		
		if(as_.isNewGoalAvailable()) {
			gh_ = as_.acceptNewGoal();
			
			ctrl_.setMaxVel(0, RAD2P(gh_->fingerVel)/1000.0);
			ctrl_.setMaxVel(1, RAD2P(gh_->fingerVel)/1000.0);
			ctrl_.setMaxVel(2, RAD2P(gh_->fingerVel)/1000.0);
			ctrl_.setMaxVel(3, RAD2S(gh_->spreadVel)/1000.0);
			
			std::cout << " : " << gh_->finger[0] << " - " << RAD2P(gh_->finger[0])
								<< " : " << gh_->finger[1] << " - " << RAD2P(gh_->finger[1])
								<< " : " << gh_->finger[2] << " - " << RAD2P(gh_->finger[2]) << std::endl;
			
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
			
			//std::cout << " : " << mode1 << " : " << mode2 << " : " << mode3 << " : " << mode4 <<std::endl;
		}
		
		loop_rate.sleep();
	}
	
	ctrl_.stopHand();
	}
	
private:
	
	bool chackGoal(barrett_hand_controller::BHMoveGoal::ConstPtr g) {
		for(unsigned int i = 0; i < 3; i++) {
			if((g->finger[i] < 0.0) || (g->finger[i] > 2.4))
				return false;
		}
		return true;
	}
	
	MotorController ctrl_;
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<barrett_hand_controller::BHMoveAction> as_;
	barrett_hand_controller::BHMoveGoal::ConstPtr gh_;
	barrett_hand_controller::BHMoveResult result_;
	ros::Publisher joint_pub_;
	sensor_msgs::JointState joint_states_;
};


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "hand");
	
	Hand hand(std::string("can1"));
	hand.run();

	return 0;
}
