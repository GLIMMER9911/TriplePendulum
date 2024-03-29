/*
Zmqmsg is mainly used to sending data and receive data by zmq;

	There may be some problem where is: there is no len()>0 in get_request() 

*/

#include "zmqmsg.hpp"
#include <zmq.hpp>
#include "statcal_util.hpp"


const char* TERMINATE = "terminate";
const char* SHUTTINGDOWN = "shutting down";

struct Zmqmsg::Imp {
	Zmqmsg* zmqmsg;
	std::string port;
	std::string socket_addr = "tcp://*:" + port;
	zmq::context_t context;
	zmq::socket_t socket;

	Imp(Zmqmsg* zmqmsg) : zmqmsg(zmqmsg) {

	}
};



void Zmqmsg::init() {
	//  Prepare our context and socket
	imp_->context = zmq::context_t(1);
	imp_->socket = zmq::socket_t(imp_->context, ZMQ_REP);
	imp_->socket.bind(imp_->socket_addr.c_str());
}

// Get end effector position and theta parameters
std::vector<double> Zmqmsg::get_request()
{
	zmq::message_t request;

	// Wait for next request from client
	imp_->socket.recv(&request);
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
	imp_->socket.send(reply);
}


// default construct function
Zmqmsg::Zmqmsg() : imp_(new Imp(this)) {
	imp_->port = "8099";
	imp_->socket_addr = "tcp://*:" + imp_->port;
}

Zmqmsg::Zmqmsg(std::string p) : imp_(new Imp(this)) {
	imp_->port = p;
	imp_->socket_addr = "tcp://*:" + imp_->port;
}

Zmqmsg::~Zmqmsg() = default;