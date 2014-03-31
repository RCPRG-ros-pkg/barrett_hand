#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <inttypes.h>
#include "CANDev.h"

class MotorController {
public:
	typedef int32_t tact_array_t[24];
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
	void setMaxTorque(int id, uint32_t vel);
	void setCloseTarget(int id, uint32_t ct);
	void setOpenTarget(int id, uint32_t ot);
	void setTargetPos(int id, int32_t pos);
	void setTargetVel(int id, int32_t pos);
	void setHoldPosition(int id, bool hold);
	void moveAll();
	void moveAllVel();
	void getStatus(int id, int32_t &mode);
	void getStatusAll(int32_t &mode1, int32_t &mode2, int32_t &mode3, int32_t &mode4);
	void recTact(int id, int32_t &gr, int32_t &a, int32_t &b, int32_t &c, int32_t &d, int32_t &e);
	void getTactile(int id, tact_array_t &tact);
        void getTemp(int id, int32_t &temp);
        void getTherm(int id, int32_t &temp);
	bool isDevOpened();
protected:
	void setProperty(int id, uint32_t property, int32_t value);
	void reqProperty(int id, uint32_t property);
	void recEncoder2(int id, int32_t &p, int32_t &jp);
	void recProperty(int id, int32_t &value);
	int32_t getParameter(int32_t id, int32_t prop_id);
	void setParameter(int32_t id, int32_t prop_id, int32_t value, bool save);
private:
	CANDev dev;
};

#endif

