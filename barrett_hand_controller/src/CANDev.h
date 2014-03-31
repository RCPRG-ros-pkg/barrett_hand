#ifndef _CAN_DEV_H_
#define _CAN_DEV_H_

#include <string>
#include <vector>

#if !defined(HAVE_RTNET)
#include <linux/can.h>
#include <linux/can/raw.h>
#else
#include <rtdm/rtcan.h> // Defines for the RT CAN socket
#endif

class CANDev {
public:
  CANDev(std::string dev);
  ~CANDev();
  
  void send(uint32_t can_id, uint8_t len, const uint8_t *data);
  uint32_t waitForReply(uint32_t can_id, uint8_t *data);
  bool isOpened();
protected:
private:

  int dev;
  std::vector<can_frame> frame_buf;
};

#endif

