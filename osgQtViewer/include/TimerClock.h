/* Copyright© 2022 Jack721 */
#pragma once
#include <iostream>
#include <chrono>

using Min = std::chrono::minutes;
using Sec = std::chrono::seconds;
using Ms = std::chrono::milliseconds;
using Us = std::chrono::microseconds;

using SteadyClock = std::chrono::steady_clock;

class TimerClock {
 public:
	TimerClock() {}

	~TimerClock() {}

	// 开始or重新计时
	void start() {
		_start = SteadyClock::now();
	}

	// 获取时间，根据模板设置时间单位
	template <typename clockUnit = Sec>
	auto getTime() {
		_end = SteadyClock::now();
		return std::chrono::duration_cast<clockUnit>(_end - _start).count();
	}

 private:
	std::chrono::time_point<SteadyClock> _start;
	std::chrono::time_point<SteadyClock> _end;
};
