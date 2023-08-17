#ifndef ZMQMSG_H
#define ZMQMSG_H

// 用于使用zmq发送和接收数据

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

#include "statcal_util.hpp"

class Zmqmsg {

public:
	void init();
	std::vector<double> get_request();
	void send_msg(std::vector<double>& data);

	Zmqmsg();
	Zmqmsg(std::string p);
	~Zmqmsg();

private:
	std::string port ;
	std::string socket_addr = "tcp://*:" + port;
	zmq::context_t context_;
	zmq::socket_t socket_;

};

#endif