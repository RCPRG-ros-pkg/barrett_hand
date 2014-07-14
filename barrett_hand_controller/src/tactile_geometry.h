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

#ifndef _TACTILE_GEOMETRY_H_
#define _TACTILE_GEOMETRY_H_

const double palm_sensor_center[24][3] = {	// in mm
	{ 22, 15.9, 77.5 },
	{ 11, 15.9, 77.5 },
	{ 0, 15.9, 77.5 },
	{ -11, 15.9, 77.5 },
	{ -22, 15.9, 77.5 },
	{ 33, 5.3, 77.5 },
	{ 22, 5.3, 77.5 },
	{ 11, 5.3, 77.5 },
	{ 0, 5.3, 77.5 },
	{ -11, 5.3, 77.5 },
	{ -22, 5.3, 77.5 },
	{ -33, 5.3, 77.5 },
	{ 33, -5.3, 77.5 },
	{ 22, -5.3, 77.5 },
	{ 11, -5.3, 77.5 },
	{ 0, -5.3, 77.5 },
	{ -11, -5.3, 77.5 },
	{ -22, -5.3, 77.5 },
	{ -33, -5.3, 77.5 },
	{ 22, -15.9, 77.5 },
	{ 11, -15.9, 77.5 },
	{ 0, -15.9, 77.5 },
	{ -11, -15.9, 77.5 },
	{ -22, -15.9, 77.5 }
};

const double palm_sensor_halfside1[24][3] = {	// in mm
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 },
	{ 5, 0, 0 }
};

const double palm_sensor_halfside2[24][3] = {	// in mm
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 },
	{ 0, 4.85, 0 }
};

const double finger_sensor_center[24][3] = {	// in mm
	{ 22.25, -9.5, 5.2 },
	{ 22.25, -9.5, 0 },
	{ 22.25, -9.5, -5.2 },
	{ 28.25, -9.5, 5.2 },
	{ 28.25, -9.5, 0 },
	{ 28.25, -9.5, -5.2 },
	{ 34.2484, -9.41371, 5.2 },
	{ 34.2484, -9.41371, 0 },
	{ 34.2484, -9.41371, -5.2 },
	{ 40.2349, -9.05695, 5.2 },
	{ 40.2349, -9.05695, 0 },
	{ 40.2349, -9.05695, -5.2 },
	{ 46.1912, -8.35887, 5.2 },
	{ 46.1912, -8.35887, 0 },
	{ 46.1912, -8.35887, -5.2 },
	{ 51.0813, -7.1884, 5.2 },
	{ 51.0813, -7.1884, 0 },
	{ 51.0813, -7.1884, -5.2 },
	{ 53.8108, -5.14222, 5.2 },
	{ 53.8108, -5.14222, 0 },
	{ 53.8108, -5.14222, -5.2 },
	{ 55.4163, -2.13234, 5.2 },
	{ 55.4163, -2.13234, 0 },
	{ 55.4163, -2.13234, -5.2 }
};

const double finger_sensor_halfside1[24][3] = {	// in mm
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.75, 0, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.74837, 0.085096, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.73902, 0.241919, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 2.72073, 0.397956, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 1.35885, 0.614231, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.970635, 1.13209, 0 },
	{ 0.399575, 1.4367, 0 },
	{ 0.399575, 1.4367, 0 },
	{ 0.399575, 1.4367, 0 }
};

const double finger_sensor_halfside2[24][3] = {	// in mm
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 },
	{ 0, 0, 2.35 }
};

#endif

