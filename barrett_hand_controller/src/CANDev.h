#ifndef _CAN_DEV_H_
#define _CAN_DEV_H_

#include <string>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>

class CANDev {
public:
  CANDev(std::string dev);
  ~CANDev();
  
  void send(uint32_t can_id, uint8_t len, const uint8_t *data);
  uint32_t waitForReply(uint32_t can_id, uint8_t *data);
protected:
private:

  int dev;
  std::vector<can_frame> frame_buf;
};

#endif

