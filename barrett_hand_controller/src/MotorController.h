#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <inttypes.h>
#include "CANDev.h"

class MotorController {
public:
	MotorController(std::string dev_name = "can0");
	~MotorController();
	void initHand();
	void stopHand();
	void resetFinger(int id);
	void open(int id);
	void close(int id);
	void getPosition(int id, int32_t &p, int32_t &jp);
	void getPositionAll(int32_t &p1, int32_t &p2, int32_t &p3, int32_t &jp1, int32_t &jp2, int32_t &jp3, int32_t &s);
	void setMaxVel(int id, uint32_t vel);
	void setCloseTarget(int id, uint32_t ct);
	void setOpenTarget(int id, uint32_t ot);
	void setTargetPos(int id, int32_t pos);
	void moveAll();
	void getStatus(int id, int32_t &mode);
	void getStatusAll(int32_t &mode1, int32_t &mode2, int32_t &mode3, int32_t &mode4);
protected:
	void setProperty(int id, uint32_t property, int32_t value);
	void reqProperty(int id, uint32_t property);
	void recEncoder2(int id, int32_t &p, int32_t &jp);
	void recProperty(int id, int32_t &value);
private:
	CANDev dev;
};

#endif

