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

#include "Tactile.h"
#include "tactile_geometry.h"

Tactile::Tactile() :
	calibration_counter_max_(40)
{
	calibration_counter_ = calibration_counter_max_;
}

void Tactile::setGeometry(std::string name, const Tactile::RawTactileData &center, const Tactile::RawTactileData &halfside1, const Tactile::RawTactileData &halfside2, double scale = 1)
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

void Tactile::startCalibration()
{
	calibration_counter_ = 0;
	for (int i=0; i<24; ++i)
	{
		offsets_[i] = 0.0;
	}
}

std::string Tactile::getName()
{
	return sensor_name_;
}

geometry_msgs::Vector3 Tactile::getCenter(int i)
{
	if (i>=0 && i<24)
		return sensor_center_[i];
	return geometry_msgs::Vector3();
}

geometry_msgs::Vector3 Tactile::getHalfside1(int i)
{
	if (i>=0 && i<24)
		return sensor_halfside1_[i];
	return geometry_msgs::Vector3();
}

geometry_msgs::Vector3 Tactile::getHalfside2(int i)
{
	if (i>=0 && i<24)
		return sensor_halfside2_[i];
	return geometry_msgs::Vector3();
}

void Tactile::updatePressure(const MotorController::tact_array_t &tact)
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

int32_t Tactile::getPressure(int i)
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

int32_t Tactile::getForce(int i)
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

