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

#include "../../src/controller/control.hpp"
#include "../../src/controller/model.hpp"
#include "../../src/zmq/zmqmsg.hpp"
#include "../../src/plan/plan.hpp"
#include "../../src/utils/utils.hpp"

const double PI = 3.141592653589793;

int main(int argc, char* argv[])
{

	//triple::TripleModel tripleModel;

	//triple::Controller triplePendulumController(&tripleModel);

	//std::vector<double> targetxy{ 0.0, 0.725};
	//std::vector<double> weightxy{ 5, 10, 5, 10 };
	//triple::fixedPlan fixedPointPlan(targetxy, weightxy);
	//std::vector<double> data(18, 0);
	//data[0] = 0.0;
	//data[1] = -2.46347e-07;
	//data[2] = -2.09492e-05;
	//data[3] = 2.44787e-06;
	//data[4] = 0;
	//data[5] = 0;
	////std::vector<double> data{-0.316016	-1.46347e-07	-1.09492e-05	2.44787e-06	0	0	0.225319	0.689098	1.25477};
	//std::vector<double> desiredValue{0};
	//std::cout << data[6] << " " << data[7] << " " << data[8] << " " << data[9] << " " << data[10] << " " << data[11] << "\n" << std::endl;


	//// data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
	//tripleModel.calcuForwardKinematics(data);
	//std::cout << data[6] << " " << data[7] << " " << data[8] << " " << data[9] << " " << data[10] << " " << data[11] << std::endl;

	//// stateValue:  joint1, joint2, joint3, w1, w2, w3, ax, ay
	//fixedPointPlan.setStateVal(data);
	//// desiredValue: x desired acc, y desired acc
	//desiredValue = fixedPointPlan.getStateVal();
	//std::cout << desiredValue[0] << " " << desiredValue[1] << " " << std::endl;

	//// calculate triple pendulum torque 
	//triplePendulumController.getStateVar(data, desiredValue);
	//std::vector<double> torque = triplePendulumController.sendTorque();
	//std::cout << torque[0] << " " << torque[1] << std::endl;

	//return 0;


	triple::TripleModel tripleModel;

	triple::Controller triplePendulumController(tripleModel.createModel().release());

	Zmqmsg zmqmsg;
	zmqmsg.init();

	// Open the file which name is "data+time", and the address is out
	//openFile();

	// Initialize plan to get desired accelerate;
	// (0.2 0.8) weightxy 5,10, kx = 2
	//std::vector<double> targetxy{ 0.16, 1.0 };
	//std::vector<double> weightxy{ 5, 10, 2, 8 };

	std::vector<double> targetxy{ 0.01, 0.725 };
	std::vector<double> weightxy{ 5, 10, 2, 8 };
	triple::fixedPlan fixedPointPlan(targetxy, weightxy);

	std::vector<double> data{0};
	std::vector<double> desiredValue{0};

	std::vector<double> lambda{ 1e-5, 1e-5, 1e-1, 1e-1, 1000 };

	triplePendulumController.setQPparameters(lambda);

	while (true) {
		data = zmqmsg.get_request();

		//data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
		triplePendulumController.calcuForwardKinematics(data);
		
		//std::cout << "data: ";
		//for (int i = 0; i < data.size(); ++i) {
		//	std::cout << data[i] << " ";
		//}
		//std::cout << std::endl;

		// stateValue:  joint1, joint2, joint3, w1, w2, w3, ax, ay
		fixedPointPlan.setStateVal(data);
		// desiredValue: x desired acc, y desired acc
		desiredValue = fixedPointPlan.getStateVal();

		// calculate triple pendulum torque 
		triplePendulumController.getStateVar(data, desiredValue);
		std::vector<double> torque = triplePendulumController.sendTorque();

		zmqmsg.send_msg(torque);

		// 验证计算是否正确
		//triplePendulumController.verifyAccelerate(data);

	}

	return 0;
}
