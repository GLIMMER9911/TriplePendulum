#include <filesystem>
#include <iostream>
#include <aris.hpp>

#include "plan/plan.hpp"
#include "controller/model.hpp"
#include "zmq/zmqmsg.hpp"
#include "controller/control.hpp"
#include "controller/acrobot.hpp"
#include "utils/utils.hpp"

const double PI = 3.141592653589793;


int main(int argc, char* argv[])
{
	Zmqmsg zmqmsg;
	zmqmsg.init();

	// Initialize plan to get desired accelerate;
	// (0.2 0.8) weightxy 5,10, kx = 2
	//std::vector<double> targetxy{ 0.16, 1.0 };
	//std::vector<double> weightxy{ 5, 10, 2, 8 };

	triple::TripleModel tripleModel;

	acrobot::Acrobot AcrobotController;
	AcrobotController.setModel(tripleModel.createAcrobotModel().release());

	std::vector<double> lamba{0.0, 0.0, 0.0, 20};
	AcrobotController.setQPparameters(lamba);
	// 
	std::vector<double> targetxy{ 0.0, 1.0 };
	std::vector<double> weightxy{ 1, 2, 1, 1};
	triple::fixedPlan fixedPointPlan(targetxy, weightxy);

	std::vector<double> data{0};
	std::vector<double> desiredValue{0};
	std::vector<double> torque{0};

	while (true) {
		data = zmqmsg.get_request();

		//std::cout << "data: ";
		//for (int i = 0; i < data.size(); ++i) {
		//	std::cout << data[i] << " " ;
		//}
		//std::cout << std::endl;

		// data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
		AcrobotController.calcuForwardKinematics(data);

		// stateValue:  joint1, joint2, w1, w2, ax, ay, bz
		fixedPointPlan.setStateVal(data);

		// desiredValue: x desired acc, y desired acc
		desiredValue = fixedPointPlan.getStateVal();

		// calculate triple pendulum torque 
		AcrobotController.getStateVar(data, desiredValue);

		torque = AcrobotController.sendTorque();

		zmqmsg.send_msg(torque);

		// 验证计算是否正确
		AcrobotController.verifyAccelerate(data);

	}

	return 0;
}
