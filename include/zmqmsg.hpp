#ifndef ZMQMSG_H
#define ZMQMSG_H

// 用于使用zmq发送和接收数据

#include <string>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include "export.h"

class BAL_CTRL_API Zmqmsg {

public:
	void init();
	std::vector<double> get_request();
	void send_msg(std::vector<double>& data);

	Zmqmsg();
	Zmqmsg(std::string p);
	~Zmqmsg();

private:
	struct Imp;
	std::unique_ptr<Imp> imp_;
};

#endif