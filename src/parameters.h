#pragma once

#include <string>

extern std::string ODOM_TOPIC;
extern std::string FEATURE_TOPIC;

//里程计测量噪声(标准差)
extern double ODOM_X_N;
extern double ODOM_Y_N;
extern double ODOM_TH_N;

//观测噪声
extern double OBS_R_N;
extern double OBS_TH_N;

extern int NUM_ITERATIONS;

const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;

