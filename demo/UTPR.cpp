/// 走x方向的轨迹
//-------------------------------------------------------------------------------------------------------//
//三个杆件，最后一根连杆上有一个质量
//模型示意：
//                                    
//                                      ** (body)
//                                      /
//                                     /  link3 (c)
//                                    o
//                                   /
//                                  /  link2(b)
//                                 o                           y
//                                /                            ^
//                               / link1(a)                    | 
//                              o                              *---> x
//                            ground
//
//-------------------------------------------------------------------------------------------------------//

#include <filesystem>
#include <iostream>
#include <aris.hpp>

#include "plan.hpp"
#include "model.hpp"
#include "zmqmsg.hpp"
#include "control.hpp"

const double PI = 3.141592653589793;


int main(int argc, char* argv[])
{

	Zmqmsg zmqmsg;
	zmqmsg.init();

	triple::TripleModel tripleModel;
	triple::Controller triplePendulumController(tripleModel.createUTPRModel().release());

	// Initialize plan to get desired accelerate;
	// (0.2 0.8) weightxy 5,10, kx = 2
	//std::vector<double> targetxy{ 0.16, 1.0 };
	//std::vector<double> weightxy{ 5, 10, 2, 8 };

	std::vector<double> targetxy{ 0.01, 0.725 };
	std::vector<double> weightxy{ 1, 10, 1, 10 };
	triple::fixedPlan fixedPointPlan(targetxy, weightxy);

	std::vector<double> data{0};
	std::vector<double> desiredValue{0};

	while (true) {

		data = zmqmsg.get_request();

		//std::cout << "data: ";
		//for (int i = 0; i < data.size(); ++i) {
		//	std::cout << data[i] << " " ;
		//}
		//std::cout << std::endl;

		// data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
		triplePendulumController.calcuForwardKinematics(data);

		// stateValue:  joint1, joint2, joint3, w1, w2, w3, ax, ay
		fixedPointPlan.setStateVal(data);

		// desiredValue: x desired acc, y desired acc
		desiredValue = fixedPointPlan.getStateVal();

		// calculate triple pendulum torque 
		triplePendulumController.getStateVar(data, desiredValue);

		std::vector<double> torque = triplePendulumController.sendTorque();

		zmqmsg.send_msg(torque);


		// 验证计算是否正确
		triplePendulumController.verifyAccelerate(data);

	}

	return 0;
}




























