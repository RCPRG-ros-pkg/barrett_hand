#include "geometry_msgs/Vector3.h"
#include <string>
#include "MotorController.h"

class Tactile
{
public:
	typedef double RawTactileData[24][3];

	Tactile();
	void setGeometry(std::string name, const RawTactileData &center, const RawTactileData &halfside1, const RawTactileData &halfside2, double scale);
	void startCalibration();
	std::string getName();
	geometry_msgs::Vector3 getCenter(int i);
	geometry_msgs::Vector3 getHalfside1(int i);
	geometry_msgs::Vector3 getHalfside2(int i);
	void updatePressure(const MotorController::tact_array_t &tact);
	int32_t getPressure(int i);
	int32_t getForce(int i);

private:
	std::string sensor_name_;
	
	geometry_msgs::Vector3 sensor_center_[24];
	geometry_msgs::Vector3 sensor_halfside1_[24];
	geometry_msgs::Vector3 sensor_halfside2_[24];

	double field_[24];

	double offsets_[24];
	int32_t tact_[24];

	int32_t calibration_counter_;
	const int32_t calibration_counter_max_;
};

