#include "get_data.h"


void fnGetTimeString(char* time_string)
{
	int err = 0;
	//get time_now data
	system_clock::time_point t_now = system_clock::now();
	std::time_t t = system_clock::to_time_t(t_now);
#pragma warning(suppress : 4996)
	const std::tm* lt = std::localtime(&t);

	//get msec data
	milliseconds ms = duration_cast<milliseconds>(t_now.time_since_epoch());
	long long ll_ms = ms.count();
	seconds  s = duration_cast<seconds>(t_now.time_since_epoch());
	long long ll_s = s.count();
	long long ll_ms_data = ll_ms - (ll_s * 1000);

	//convert time data
	err = sprintf_s(time_string, 100, "%d_%d_%d_%d_%d_%d", lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
		lt->tm_hour, lt->tm_min, lt->tm_sec);
	if (err == -1)
		std::cout << "Can't get file name" << std::endl;

	return;
}

long long fnGetCurTimems(void) {
	auto time1 = std::chrono::system_clock::now();
	milliseconds s = duration_cast<milliseconds>(time1.time_since_epoch());
	return s.count();
}
