/* Copyright© 2022 Jack721 */
#pragma once
#include <iostream>
#include <chrono>

typedef std::chrono::minutes       Min;
typedef std::chrono::seconds       Sec;
typedef std::chrono::milliseconds  Ms;
typedef std::chrono::microseconds  Us;

class TimerClock {
 public:
	TimerClock() {}

	~TimerClock() {}

	// 开始or重新计时
	void start() {
		_start = std::chrono::steady_clock::now();
	}

	// 获取时间，根据模板设置时间单位
	template <typename clockUnit = Sec>
	auto getTime() {
		_end = std::chrono::steady_clock::now();
		return std::chrono::duration_cast<clockUnit>(_end - _start).count();
	}

 private:
	std::chrono::time_point<std::chrono::steady_clock> _start;
	std::chrono::time_point<std::chrono::steady_clock> _end;
};
