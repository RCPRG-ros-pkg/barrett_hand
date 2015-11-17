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

Tactile::Tactile(int32_t median_filter_max_samples) :
	calibration_counter_max_(40),
	median_filter_max_samples_(median_filter_max_samples),
	tact_index_(0)
{
	for (int i = 0; i < 24; i++) {
		offsets_[i] = 0.0;
		field_[i] = 1.0;
	}

	calibration_counter_ = calibration_counter_max_;
	tact_ = new TactileState[median_filter_max_samples_];
	tab_ = new int32_t[median_filter_max_samples_];
}

Tactile::~Tactile()
{
	delete[] tact_;
	delete[] tab_;
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

void Tactile::startCalibration() {
	calibration_counter_ = 0;
	for (int i = 0; i < 24; i++) {
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

void Tactile::updatePressure(const TactileInterface::tact_array_t &tact)
{
        ++tact_index_;
	if (tact_index_ >= median_filter_max_samples_)
		tact_index_ = 0;

	memcpy(tact_[tact_index_], tact, sizeof(TactileInterface::tact_array_t));

	if (calibration_counter_ < calibration_counter_max_)
	{
		for (int i=0; i<24; ++i)
		{
			if (tact_[tact_index_][i] > offsets_[i])
				offsets_[i] = tact_[tact_index_][i];
//			offsets_[i] += tact_[tact_index_][i];
		}
		++calibration_counter_;
		if (calibration_counter_ >= calibration_counter_max_)
		{
//			for (int i=0; i<24; ++i)
//			{
//				offsets_[i] /= (double)calibration_counter_max_;
//			}
		}
	}
}

int Tactile::selectionAlgorithm(int *tab, int left,int right,int kth)
{
	int pivotIndex = 0;
	for(;;)
	{
		{
			int p=left,r=right,x=tab[r],i=p-1;
			for(int j=p;j<=r-1;j++)
			{
				if (tab[j]<=x)
				{
					i=i+1;
					int tmp = tab[i];
					tab[i] = tab[j];
					tab[j] = tmp;
				}
			}
			int tmp = tab[i+1];
			tab[i+1] = tab[r];
			tab[r] = tmp;
			pivotIndex = i+1;
		}
		int len=pivotIndex-left+1;

		if(kth==len)
			return tab[pivotIndex];
		else if(kth<len)
			right=pivotIndex-1;
		else
		{
			kth=kth-len;
			left=pivotIndex+1;
		}
	}
}

int32_t Tactile::getPressure(int i, int median_filter_samples)
{
	if (calibration_counter_ < calibration_counter_max_)
	{
		return 0;
	}

	if (median_filter_samples<1 || median_filter_samples>median_filter_max_samples_)
	{
		return 0;
	}

	double result = 0.0;
	for (int k=0; k<median_filter_samples; ++k)
	{
		int index = (tact_index_-k+median_filter_max_samples_)%median_filter_max_samples_;
		tab_[k] = tact_[index][i];
	}
	if ((median_filter_samples%2) == 1)
	{
		result = (double)selectionAlgorithm(tab_, 0, median_filter_samples-1, (median_filter_samples+1)/2);
	}
	else
	{
		result = ((double)selectionAlgorithm(tab_, 0, median_filter_samples-1, median_filter_samples/2) +
			(double)selectionAlgorithm(tab_, 0, median_filter_samples-1, median_filter_samples/2+1))*0.5;
	}

	result -= offsets_[i];
	if (result < 0)
		result = 0;
	return (int32_t)result;
}

