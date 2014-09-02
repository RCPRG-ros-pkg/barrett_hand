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

#include "geometry_msgs/Vector3.h"
#include <string>
#include "MotorController.h"

class Tactile
{
public:
	typedef double RawTactileData[24][3];
        typedef int32_t TactileState[24];

	Tactile(int32_t median_filter_max_samples);
	~Tactile();
	void setGeometry(std::string name, const RawTactileData &center, const RawTactileData &halfside1, const RawTactileData &halfside2, double scale);
	void startCalibration();
	std::string getName();
	geometry_msgs::Vector3 getCenter(int i);
	geometry_msgs::Vector3 getHalfside1(int i);
	geometry_msgs::Vector3 getHalfside2(int i);
	void updatePressure(const MotorController::tact_array_t &tact);
	int selectionAlgorithm(int *tab, int left,int right,int kth);
	int32_t getPressure(int i, int median_filter_samples);

private:
	std::string sensor_name_;
	
	geometry_msgs::Vector3 sensor_center_[24];
	geometry_msgs::Vector3 sensor_halfside1_[24];
	geometry_msgs::Vector3 sensor_halfside2_[24];

	double field_[24];

	double offsets_[24];
	TactileState *tact_;
        int tact_index_;
	int32_t *tab_;

	int32_t calibration_counter_;
	const int32_t calibration_counter_max_;
        const int32_t median_filter_max_samples_;
};

