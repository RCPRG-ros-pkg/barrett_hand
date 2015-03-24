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

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_srvs/Empty.h"

#include "rtt_rosclock/rtt_rosclock.h"

#include "FtSensor.h"

//#include <fcntl.h>
#include <stdlib.h>
//#include <sys/types.h>
//#include <sys/stat.h>

using namespace std;

using namespace RTT;

class FtSensorComponent : public RTT::TaskContext{
private:

	BarrettFtSensor *ctrl_;

	geometry_msgs::Vector3Stamped accel_;
	geometry_msgs::WrenchStamped wrench_;
	bool tare_;

	OutputPort<geometry_msgs::Vector3Stamped>	accel_out_;
	OutputPort<geometry_msgs::WrenchStamped>	wrench_out_;
	string dev_name_;
	string prefix_;

	string frame_id;

public:
	FtSensorComponent(const std::string& name):
		TaskContext(name, PreOperational),
		accel_out_("accel"),
		wrench_out_("wrench"),
		ctrl_(NULL),
		tare_(false)
	{
		this->addPort(accel_out_).doc("Sends out the stamped acceleration");
		this->addPort(wrench_out_).doc("Sends out the WrenchStamped");

		this->provides()->addOperation("tare",&FtSensorComponent::tare,this,RTT::OwnThread);
		this->provides()->addOperation("tare_ros",&FtSensorComponent::tareRos,this,RTT::OwnThread);

		this->addProperty("device_name", dev_name_);
		this->addProperty("prefix", prefix_);
	}

	~FtSensorComponent()
	{
	}

	void cleanupHook()
	{
		if (ctrl_ != NULL)
		{
			delete ctrl_;
		}
	}

	bool configureHook()
	{
		if (ctrl_ == NULL && !dev_name_.empty() && !prefix_.empty())
		{
			frame_id = prefix_ + "_ft_sensor";
			ctrl_ = new BarrettFtSensor(dev_name_);
			if (ctrl_->isDevOpened())
			{
				int32_t status = ctrl_->getPuckStatus();
				if (status != 2)
				{
					ctrl_->setPuckStatus(2);
					status = ctrl_->getPuckStatus();
				}
				return status == 2;
			}
		}
		return false;
	}

	bool startHook()
	{
		return true;
	}

	void stopHook()
	{
	}

	void updateHook()
	{
		int16_t fx=0, fy=0, fz=0, tx=0, ty=0, tz=0;
		int16_t ax=0, ay=0, az=0;
		int32_t result_ft=0;
		int32_t result_ac=0;

		ros::Time time_now = rtt_rosclock::host_now();

		if (tare_)
		{
			ctrl_->tare();
			tare_ = false;
		}

		result_ac = ctrl_->readAcceleration(ax, ay, az);
		result_ft = ctrl_->readForceTorque(fx, fy, fz, tx, ty, tz);

		wrench_.header.stamp = time_now;
		wrench_.header.frame_id = frame_id;
		wrench_.wrench.force.x = static_cast<double>(fx)/256.0;
		wrench_.wrench.force.y = static_cast<double>(fy)/256.0;
		wrench_.wrench.force.z = static_cast<double>(fz)/256.0;
		wrench_.wrench.torque.x = static_cast<double>(tx)/4096.0;
		wrench_.wrench.torque.y = static_cast<double>(ty)/4096.0;
		wrench_.wrench.torque.z = static_cast<double>(tz)/4096.0;

		accel_.header.stamp = time_now;
		accel_.header.frame_id = frame_id;
                accel_.vector.x = static_cast<double>(ax)/1024.0;
                accel_.vector.y = static_cast<double>(ay)/1024.0;
                accel_.vector.z = static_cast<double>(az)/1024.0;

		accel_out_.write(accel_);
		wrench_out_.write(wrench_);
	}

	bool tare()
	{
		tare_ = true;
		return true;
	}

	bool tareRos(std_srvs::Empty::Request  &req,
	         std_srvs::Empty::Response &res)
	{
		tare_ = true;
		return true;
	}

};
ORO_CREATE_COMPONENT(FtSensorComponent)

