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

#include "TactileInterface.h"

#include <string>
#include <cstring>

// Tactile sensor array access (special)
#define PROP_TACT 106

#define TACTILE_FULL_GROUP 9

#define GROUP(from, to) 0x400 | ((from) << 5) | (to)
#define NGROUP(from, to) ((from) << 5) | (to)

TactileInterface::TactileInterface(const std::string &dev_name) {

    // 11                               000 0000 1011
    // 12                               000 0000 1100
    // 13                               000 0000 1101
    // 14                               000 0000 1110
    // GROUP(11, TACTILE_FULL_GROUP)    001 0110 1001
    // GROUP(12, TACTILE_FULL_GROUP)    001 1000 1001
    // GROUP(13, TACTILE_FULL_GROUP)    001 1010 1001
    // GROUP(14, TACTILE_FULL_GROUP)    001 1100 1001
    // mask:    0x07FF                  111 1111 1111

    std::vector<CANDev::FilterElement > filters;
    for (int puck_id = 0; puck_id < 4; puck_id++) {
//        filters.push_back( CANDev::FilterElement(11 + puck_id, 0x07FF) );
        filters.push_back( CANDev::FilterElement(GROUP(11 + puck_id, TACTILE_FULL_GROUP), 0x07FF) );
    }

    pdev_ = new CANDev(dev_name, "TactileInterface", filters);
}

TactileInterface::~TactileInterface() {
    delete pdev_;
}

void TactileInterface::setProperty(int can_id, uint32_t property, int32_t value) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	
	frame.can_id = NGROUP(9, can_id);
	frame.can_dlc = 6;
	
	frame.data[0] = 0x80 | property;
	frame.data[1] = 0;
	
	for(unsigned int i=2; i<6; i++){
        frame.data[i] = (uint8_t)(value & 0x000000FF);
        value >>= 8;
    }
	pdev_->send(frame.can_id, frame.can_dlc, frame.data);
}

void TactileInterface::recTact(int can_id, int32_t &gr, int32_t &a, int32_t &b, int32_t &c, int32_t &d, int32_t &e) {
	uint8_t data[8];
	pdev_->waitForReply(GROUP(can_id, TACTILE_FULL_GROUP), data);

	gr = (data[0]>>4)&0x0F;
	a = ((data[0]&0x0F)<<8) | data[1];
	b = (data[2]<<4) | ((data[3]>>4)&0x0F);
	c = ((data[3]&0x0F)<<8) | data[4];
	d = (data[5]<<4) | ((data[6]>>4)&0x0F);
	e = ((data[6]&0x0F)<<8) | data[7];
}

void TactileInterface::getTactile(int puck_id, tact_array_t &tact) {
	setProperty(11 + puck_id, PROP_TACT, 2);
	int gr;
	recTact(11 + puck_id, gr, tact[0], tact[1], tact[2], tact[3], tact[4]);
	recTact(11 + puck_id, gr, tact[5], tact[6], tact[7], tact[8], tact[9]);
	recTact(11 + puck_id, gr, tact[10], tact[11], tact[12], tact[13], tact[14]);
	recTact(11 + puck_id, gr, tact[15], tact[16], tact[17], tact[18], tact[19]);
	recTact(11 + puck_id, gr, tact[20], tact[21], tact[22], tact[23], tact[24]);
}

bool TactileInterface::isDevOpened() {
	return pdev_->isOpened();
}


