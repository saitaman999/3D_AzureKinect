#pragma once

#ifndef _GET_DATA_H_
#define _GET_DATA_H_

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <condition_variable>
#include <direct.h>
#include <stdlib.h>

using namespace std::chrono;
using std::chrono::system_clock;

//get cur time data
void fnGetTimeString(char* time_string);


#endif //_GET_DATA_H_
