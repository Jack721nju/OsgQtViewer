#pragma once
#include <iostream>
#include <chrono>

using namespace std;

typedef chrono::minutes       Min;
typedef chrono::seconds       Sec;
typedef chrono::milliseconds  Ms;
typedef chrono::microseconds  Us;

class TimerClock {
public:
	TimerClock() {
	}

	~TimerClock(){
	}

	//��ʼ/���¼�ʱ
	void start(){
		_start = chrono::steady_clock::now();
	}

	//��ȡʱ�䣬����ģ������ʱ�䵥λ
	template <typename clockUnit = Sec>
	auto getTime(){
		_end = chrono::steady_clock::now();
		return chrono::duration_cast<clockUnit>(_end - _start).count();
	}

private:
	chrono::time_point<chrono::steady_clock> _start;
	chrono::time_point<chrono::steady_clock> _end;
};