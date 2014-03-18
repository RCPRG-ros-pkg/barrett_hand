#ifndef _CAN_DEV_H_
#define _CAN_DEV_H_

#include <string>
#include <vector>

//#include <linux/can.h>
//#include <linux/can/raw.h>

#include <rtdm/rtcan.h> // Defines for the RT CAN socket

//#define socket rt_dev_socket
//#define setsockopt rt_dev_setsockopt
//#define ioctl rt_dev_ioctl
//#define bind rt_dev_bind
//#define send rt_dev_send
//#define recv rt_dev_recv


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

