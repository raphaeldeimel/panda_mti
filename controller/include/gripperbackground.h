// Copyright (c) 2018 Raphael Deimel
#pragma once


#include <string>

namespace grippernonblocking {

void startGripperCommunicationInBackground(std::string& hostname);

void stopGripperCommunicationInBackground();

float getGripperPosition();

void setGripperDesired(float q, float tau);

const float threshold_close = 0.03;
const float threshold_open  = 0.05;
const float q_max = 0.12;
const float q_min = 0.00;

const float dq_max = 0.05;
const float dq_min = -0.05;


}
