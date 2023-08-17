/// 在Aris中进行计算，在MATLAB中进行仿真验证
// 通过运动学和动力学，得到关节的加速度

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
#include "pid.hpp"


double dt = 1;
const double PI = 3.141592653589793;

std::vector<double> calcustateval(std::vector<double>& inputdata, PIDController* xp,
	PIDController* yp, PIDController* thetap, PIDController* vx_pd, PIDController* vy_pd, 
	PIDController* w_pd, PIDController* q_pd, PIDController* aj1, PIDController* aj2, PIDController* aj3);

int main(int argc, char* argv[])
{

	TripleModel Tri;
	Zmqmsg zmqmsg;

	std::vector<double> targetxy{ 0.0, 1.2, PI/2 };
	std::vector<double> targetq{ 0 };
	std::vector<double> statexy;
	std::vector<double> data{ 0 };
	std::vector<double> statevalue{ 0 };
	std::vector<double> joint_angle(3);

	Tri.getmodel();
	zmqmsg.init();
	PIDController xp(1, 0, 0);
	PIDController yp(1, 0, 0);
	PIDController thetap(0.1, 0, 0);
	PIDController vx_pd(1, 0, 0.0);
	PIDController yx_pd(1, 0, 0.0);
	PIDController w_pd(0.01, 0, 0.0);
	PIDController q_pd(1,0,0);
	PIDController aj1(1, 0, 0);
	PIDController aj2(1, 0, 0);
	PIDController aj3(1, 0, 0);

	xp.setTarget(targetxy[0]);
	yp.setTarget(targetxy[1]);
	thetap.setTarget(targetxy[2]);
	q_pd.setTarget(targetq[0]);
	while (true) {
		
		data = zmqmsg.get_request();

		// data : joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz 
		for (int i = 0; i < joint_angle.size(); i++) {
			joint_angle[i] = data[i];
		}
		statexy = Tri.calcu_forwardKinematics(joint_angle);

		for (int i = 0; i < statexy.size(); i++) {
			data[i + 6] = statexy[i];
		}
		

		statevalue = calcustateval(data, &xp, &yp, &thetap, &vx_pd, &yx_pd, &w_pd, &q_pd, &aj1, &aj2, &aj3);

		Tri.getstate_var(statevalue);
		std::vector<double> torque = Tri.send_torque();

		zmqmsg.send_msg(torque);

		auto max_value = std::max(std::abs(data[12]), std::abs(data[13]));

		if (std::max({ std::abs(data[12] - Tri.desiredAcc()[0])
			, std::abs(data[13] - Tri.desiredAcc()[1]), 
			std::abs(data[15] - Tri.desiredAcc()[2]),
			std::abs(data[16] - Tri.desiredAcc()[3]), 
			std::abs(data[17] - Tri.desiredAcc()[4]) }) >
			1e-5 * std::max(max_value, 1.0)) {
			
			static int count_ = 0;
			if (count_ == 0)
				count_++;
			else {
				std::cout << "acc desired:";
				for (int i = 0; i < 5; i++) {
					std::cout << Tri.desiredAcc()[i] << " ";
				}
				std::cout << std::endl;

				std::cout << "data back: ";
				std::cout << data[12] << " " << data[13] << " " << data[15] << " "
					<< data[16] << " " << data[17] << " " << std::endl;
				std::cout << "---------------------------- " << std::endl;
				throw std::runtime_error("error : acc data not correct");
			}
		}

		
	}
	return 0;
}


// inputdata: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz 
// stateval:  x, y, theta, vx, vy, w, ax, ay, aj1, aj2, aj3;
std::vector<double> calcustateval(std::vector<double>& inputdata, PIDController* xp,
	PIDController* yp, PIDController* thetap,  PIDController* vx_pd, PIDController* vy_pd, PIDController* w_pd, PIDController* q_pd,
	PIDController* aj1, PIDController* aj2, PIDController* aj3) {

	std::vector<double> stateval;
	
	double vx_state = xp->calculateP(inputdata[6]);
	double vy_state = yp->calculateP(inputdata[7]);
	double w_state = thetap->calculateP(inputdata[8]);

	double vx_max = 0.1;
	vx_state = std::max(vx_state, -vx_max);
	vx_state = std::min(vx_state,  vx_max);

	double vy_max = 0.1;
	vy_state = std::max(vy_state, -vy_max);
	vy_state = std::min(vy_state, vy_max);



	// ax = 0 + K( ax_d - ax_s )
	vx_pd->setTarget(vx_state);
	double ax_d = vx_pd->calculateP(inputdata[9]);
	double ax_max = 10.0;
	ax_d = std::max(ax_d, -ax_max);
	ax_d = std::min(ax_d, ax_max);
	stateval.push_back(ax_d) ;
 
	vy_pd->setTarget(vy_state);
	double ay_d = vy_pd->calculateP(inputdata[10]);
	double ay_max = 100.0;
	ay_d = std::max(ay_d, -ay_max);
	ay_d = std::min(ay_d, ay_max);
	stateval.push_back(ay_d);




	w_pd->setTarget(w_state);
	double bz_d = w_pd->calculateP(inputdata[11]);
	double bz = 0.01* (bz_d - inputdata[14]);
	// stateval.push_back(bz_d);

	double qd = q_pd->calculateP(inputdata[0]);
	double q_dd = 0.01 * (qd - inputdata[3]);
	// stateval.push_back(q_dd);



	double vj1_max{ 0.1 }, vj2_max{ 0.2 }, vj3_max{ 0.2 };
	double vj1{ inputdata[3] }, vj2{ inputdata[4] }, vj3{ inputdata[5] };
	double aj1_v{ 0.0 }, aj2_v{ 0.0 }, aj3_v{ 0.0 };

	double kp_aj{ 10.0 };
	if (vj1 > vj1_max)
		aj1_v = -kp_aj * (vj1 - vj1_max);
	else if (vj1 < -vj1_max)
		aj1_v = -kp_aj * (vj1 + vj1_max);

	if (vj2 > vj2_max)
		aj2_v = -kp_aj * (vj2 - vj2_max);
	else if (vj2 < -vj2_max)
		aj2_v = -kp_aj * (vj2 + vj2_max);

	if (vj3 > vj3_max)
		aj3_v = -kp_aj * (vj3 - vj3_max);
	else if (vj3 < -vj3_max)
		aj3_v = -kp_aj * (vj3 + vj3_max);

	stateval.push_back(aj1_v);
	stateval.push_back(aj2_v);
	stateval.push_back(aj3_v);


	static int count_ = 0;
	if (++count_ % 10 == 0) {
		/*std::cout << "count " << count_ << " ---------------- - " << std::endl;
		std::cout << "v_state  : " << vx_state<< "  " << vy_state << "  " 
			<< "  " << vj1 << "  " << vj2 << "  " << vj3 << std::endl;
		std::cout << "a_desired: " << ax_d << "  " << ay_d << "  "
			<< "  " << aj1_v << "  " << aj2_v << "  " << aj3_v << std::endl;*/
	}



	auto start = inputdata.begin();
	auto end = inputdata.end() - 13 ;
	stateval.insert(stateval.begin(), start, end);

	return stateval;
}