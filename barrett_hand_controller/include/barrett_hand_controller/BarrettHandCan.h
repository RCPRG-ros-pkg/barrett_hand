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

#ifndef BARRETT_HAND_CONTROLLER_BARRETT_HAND_CAN_H_
#define BARRETT_HAND_CONTROLLER_BARRETT_HAND_CAN_H_

// For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T 5
#define PROP_CMD 29
// 32-Bit Position. R=Act, W=Cmd
#define PROP_P 48
// Velocity (cts/ms). R=Act, W=Cmd
#define PROP_V 44
// Max velocity (cts/ms)
#define PROP_MV 45
// 32-Bit Close Target
#define PROP_CT 56
// 32-Bit Open Target
#define PROP_OT 54
// 32-Bit Endpoint
#define PROP_E 52
// Mode: 0=Idle, 2=Torque, 3=PID, 4=Vel, 5=Trap
#define PROP_MODE 8
// Temperature (puck internal)
#define PROP_TEMP 9
// Thermistor (motor) temperature
#define PROP_THERM 20
// Flag to hold position after move
#define PROP_HOLD 77
// Max torque
#define PROP_MT 43
// 32-Bit Counts per revolution
#define PROP_CTS 68
#define PROP_CTS2 69
// Motor current (2048+205/1A)
#define PROP_IMOTOR 22

//const int MODE_IDLE      = 0;
//const int MODE_TORQUE    = 2;
//const int MODE_PID       = 3;
//const int MODE_VELOCITY  = 4;
//const int MODE_TRAPEZOID = 5;

#define MODE_IDLE 0
#define MODE_TORQUE 2
#define MODE_PID 3
#define MODE_VELOCITY 4
#define MODE_TRAPEZOID 5

#define CMD_HI 13
#define CMD_TC 16
#define CMD_TO 17
#define CMD_CLOSE 18
#define CMD_OPEN 20
#define CMD_STOP 21

#define ALL_GROUP 0
#define PFEEDBACK_GROUP 3
#define HAND_GROUP 5

#define GROUP(from, to) (0x400 | ((from) << 5) | (to))

#endif  // BARRETT_HAND_CONTROLLER_BARRETT_HAND_CAN_H_

