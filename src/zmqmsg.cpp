/*
Zmqmsg is mainly used to sending data and receive data by zmq;

	There may be some problem where is: there is no len()>0 in get_request() 

*/

#include "zmqmsg.hpp"

const char* TERMINATE = "terminate";
const char* SHUTTINGDOWN = "shutting down";


void Zmqmsg::init() {
	//  Prepare our context and socket
	context_ = zmq::context_t(1);
	socket_ = zmq::socket_t(context_, ZMQ_REP);
	socket_.bind(socket_addr.c_str());
}

// Get end effector position and theta parameters
std::vector<double> Zmqmsg::get_request()
{
	zmq::message_t request;

	// Wait for next request from client
	socket_.recv(&request);
	//std::cout << " recv..." << std::endl;

	char* data_str = static_cast<char*>(request.data());
	data_str[request.size()] = '\0';

	int len = decode_header(data_str);

	// Check received data header to determine if it's a string or array of double values
	//std::cout << " len:" << len << std::endl;
	std::vector<double> data;
	decode_double_data(len, data_str, data);

	if (data.size() > 20) {
		std::cerr << "Data passed to statcalserver must be: prev_data, current_data, beta and current iteration number" << std::endl;
		std::exit(1);
	}

	return data;
}

void Zmqmsg::send_msg(std::vector<double>& data) {
	std::string reply_str;
	encode_double_data(data, reply_str);
	// Send reply back to client
	zmq::message_t reply(reply_str.size());
	memcpy(reply.data(), reply_str.c_str(), reply_str.size());
	socket_.send(reply);
}


// default construct function
Zmqmsg::Zmqmsg() {
	port = "8099";
	socket_addr = "tcp://*:" + port;
}

Zmqmsg::Zmqmsg(std::string p) {
	port = p;
	socket_addr = "tcp://*:" + port;
}

Zmqmsg::~Zmqmsg() = default;