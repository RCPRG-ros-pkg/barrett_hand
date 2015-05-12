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

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <inttypes.h>

#include <iostream>
#include <cstring>
#include <string>

#include "CANDev.h"

#if !defined(HAVE_RTNET)
#define rt_dev_socket socket
#define rt_dev_ioctl ioctl
#define rt_dev_bind bind
#define rt_dev_close close
#endif

CANDev::CANDev(std::string dev_name) {
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  if((dev = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cout<< "Error while opening socket" << std::endl;
    dev = -1;
  }
  
  strcpy(ifr.ifr_name, dev_name.c_str());
  rt_dev_ioctl(dev, SIOCGIFINDEX, &ifr);
  
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
  
  if(rt_dev_bind(dev, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cout << "Error in socket bind" << std::endl;
    rt_dev_close(dev);
    dev = -1;
  }
}

CANDev::~CANDev() {
  if(dev > -1) {
    rt_dev_close(dev);
  }
}

void CANDev::send(const uint32_t can_id, const uint8_t len, const uint8_t *data) {
  struct can_frame frame;
  
  frame.can_id = can_id;
  frame.can_dlc = len;
  
  memcpy(frame.data, data, len);
#if !defined(HAVE_RTNET)
  write(dev, reinterpret_cast<void*>(&frame), sizeof(frame));
#else
  rt_dev_send(dev, reinterpret_cast<void*>(&frame), sizeof(frame), 0);
#endif
}

uint32_t CANDev::waitForReply(uint32_t can_id, uint8_t *data) {
  struct can_frame frame;
  
  // search frame buffer
  for(size_t i = 0; i < frame_buf.size(); i++) {

    if(frame_buf[i].can_id == can_id) {
      memcpy(data, frame_buf[i].data, frame_buf[i].can_dlc);
      frame_buf.erase(frame_buf.begin()+i);
      return frame_buf[i].can_dlc;
    }
  }
  
  // wait for new data
  while(1) {
#if !defined(HAVE_RTNET)
    size_t ret = read(dev, reinterpret_cast<void*>(&frame), sizeof(frame));
#else
    size_t ret = rt_dev_recv(dev, reinterpret_cast<void*>(&frame), sizeof(frame), 0);
#endif
    if(ret != sizeof(frame)) {
      continue;
    }
    

    if(frame.can_id == can_id) {
     memcpy(data, frame.data, frame.can_dlc);
      return frame.can_dlc;
    }
    
    frame_buf.push_back(frame);
  }
}

bool CANDev::isOpened() {
	return dev != -1;
}

