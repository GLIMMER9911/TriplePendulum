#include "plan.hpp"
#include "pid.hpp"

namespace triple {
	
	// // 计算末端加速度所需要的变量；
	// struct fixedPlan::stateImp {
	// 	double targetPos;
	// 	double trajectoryVel;
	// 	double trajectoryAcc;
	// 	double statePos;
	// 	double stateVel;
	// 	double desiredVel;
	// 	double desiredAcc;
	// 	double weightKp;
	// 	double weightKv;
	// 	double maxVel;
	// 	double maxAcc;
	// 	int count = 0;

	// 	// initialize the struct parameters target, weightkp, weightKv and maxVel, maxAcc use default values
	// 	stateImp(double target, double kp, double kv) : targetPos{ target }, weightKp{ kp }, weightKv{ kv } {
	// 		this->maxVel = 2;
	// 		this->maxAcc = 5;
	// 		this->trajectoryVel = 0;
	// 		this->trajectoryAcc = 0;

	// 		if (target == 0) {
	// 			this->targetPos = 0.000000001;
	// 		}
	// 	};
		
	// 	// initialize the struct parameters target, weightkp, weightKv, maxVel, maxAcc
	// 	stateImp(double target, double kp, double kv, double maxv, double maxa) :
	// 		targetPos{ target }, weightKp{ kp }, weightKv{ kv }, maxVel{ maxv }, maxAcc{ maxa } {
	// 		this->trajectoryVel = 0;
	// 		this->trajectoryAcc = 0;
	// 		if (target == 0) {
	// 			this->targetPos = 0.000000001;
	// 		}
	// 	};

	// 	void getStatedata(double pos, double vel) {
	// 		this->statePos = pos;
	// 		this->stateVel = vel;
	// 	}

	// 	void calcuDesiredVel() {
	// 		if (trajectoryVel > 0) {
	// 			desiredVel = weightKp * (targetPos - statePos) + 2.0 * trajectoryVel;
	// 		}
	// 		else {
	// 			desiredVel = weightKp * (targetPos - statePos) + 2.0 * trajectoryVel;
	// 		}
	// 		desiredVel = std::max(desiredVel, -maxVel);
	// 		this->desiredVel = std::min(desiredVel, maxVel);
	// 	}

	// 	void calcuDesiredAcc() {
	// 		this->calcuDesiredVel();
			
	// 		if (trajectoryAcc > 0) {
	// 			desiredAcc = weightKv * (desiredVel - stateVel) + 2.0 * trajectoryAcc;
	// 		}
	// 		else {
	// 			desiredAcc = weightKv * (desiredVel - stateVel) + 2.0 * trajectoryAcc;
	// 		}

	// 		desiredAcc = std::max(desiredAcc, -maxAcc);
	// 		this->desiredAcc = std::min(desiredAcc, maxAcc);
	// 	}

	// };

	// inputdata: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
	// stateval:  joint1, joint2, joint3, w1, w2, w3, ax, ay ;
	auto fixedPlan::calcuStateVal() ->std::vector<double> {
		
		std::vector<double> desiredval{};
		static int count = 0;
		static double targetpos_x = stateVar_[0].targetPos;
		static double targetpos_y = stateVar_[1].targetPos;
		static double omiga = 2.0 * aris::PI / 5000.0;
		static double Amp = 0.1;
		static double constant_speed = 0.1;
		static double trajectory_begin_y = 0.6;
		static double Amp_x = -0.02;

		count++;


		// Use polar coordinates to go to a track
		//stateVar_[0].targetPos = comX + row * std::cos(aris::PI / 2 - theta * (count - 5000) / 5000.0);
		//stateVar_[1].targetPos = comY + row * std::sin(aris::PI / 2 - theta * (count - 5000) / 5000.0);
		// x direction to follow a trajectory
		//stateVar_[0].targetPos = targetpos_x + (-0.05) * (1 - std::cos((count / 2500.0) * 2 * aris::PI);
		// y direction to follow a trajectory
		//stateVar_[1].targetPos = targetpos_y + (-0.08) * (1 - std::cos((count / 2500.0) * 2 * aris::PI));


		//if (count <= 3000) {
		//	stateVar_[1].targetPos = targetpos_y;
		//	stateVar_[1].trajectoryVel = 0;
		//	stateVar_[1].trajectoryAcc = 0;
		//}
		//else if ((count > 3000) && (count <= 4000)) {
		//	// y direction follow a straight path of constant speed
		//	stateVar_[1].targetPos = targetpos_y + (-constant_speed) / 1000.0 * (count - 3000);
		//	stateVar_[1].trajectoryVel = 5 * (-constant_speed) / 1000.0;
		//	stateVar_[1].trajectoryAcc = 0;
		//	stateVar_[1].weightKp = 10;
		//	stateVar_[1].weightKv = 20;
		//}
		//else if ((count > 4000) && (count <= 5000)) {
		//	stateVar_[1].targetPos = trajectory_begin_y;
		//	stateVar_[1].trajectoryVel = 0;
		//}
		//else if ((count > 5000) && (count <= 12500)) {
		//	stateVar_[1].targetPos = trajectory_begin_y + Amp * (1 - std::cos((count - 5000) * omiga));
		//	stateVar_[1].trajectoryVel = Amp * std::sin((count - 5000) * omiga);
		//	stateVar_[1].trajectoryAcc = Amp * std::cos((count - 5000) * omiga);
		//}
		//else if ((count > 12500) && (count <= 13000)) {
		//	stateVar_[1].targetPos = 0.6;
		//	stateVar_[1].trajectoryVel = 0;
		//	stateVar_[1].trajectoryAcc = 0;
		//}
		//else if ((count > 13000) && (count <= 13500)) {
		//	stateVar_[1].targetPos = 0.6 + (constant_speed) / 500.0 * (count - 12970);
		//	stateVar_[1].trajectoryVel = 5*(constant_speed) / 500.0;
		//	stateVar_[1].trajectoryAcc = 0;
		//	stateVar_[1].weightKp = 20;
		//	stateVar_[1].weightKv = 50;
		//}
		//else {
		//	stateVar_[1].targetPos = targetpos_y;
		//	stateVar_[1].trajectoryVel = 0;
		//	stateVar_[1].trajectoryAcc = 0;
		//	stateVar_[0].weightKp = 10;
		//	stateVar_[0].weightKv = 20;
		//}
		
		//// ʵ����룬��ȥ�����㣬Ȼ���ٵ�ƽ���
		//if (count <= 4000) {
		//	stateVar_[0].targetPos = 0.3;
		//	stateVar_[1].targetPos = 0.7;
		//}
		//else if ((count > 4000)&& (count <= 8000)) {
		//	stateVar_[0].targetPos = 0.2;
		//	stateVar_[1].targetPos = 0.8;
		//}
		//else if ((count > 8000) && (count <= 12000)) {
		//	stateVar_[0].targetPos = -0.2;
		//	stateVar_[1].targetPos = 0.8;
		//}
		//else if ((count > 12000) && (count <= 16000)) {
		//	stateVar_[0].targetPos = -0.2;
		//	stateVar_[1].targetPos = 0.9;
		//}
		//else {
		//	stateVar_[0].targetPos = 0.0;
		//	stateVar_[1].targetPos = 1.2;
		//}
		
		 
		// set state variable position and velocity
		stateVar_[0].getStatedata(input_data_[6], input_data_[9]);
		stateVar_[1].getStatedata(input_data_[7], input_data_[10]);
		stateVar_[0].calcuDesiredAcc();
		stateVar_[1].calcuDesiredAcc();
		//std::cout << "stateVar_[1].trajectoryVel: " << stateVar_[1].trajectoryVel << std::endl;
		//std::cout << "stateVar_[1].trajectoryAcc: " << stateVar_[1].trajectoryAcc << std::endl;
		desiredval.push_back(stateVar_[0].desiredAcc);
		desiredval.push_back(stateVar_[1].desiredAcc);

		return desiredval;
	}
	 
	auto fixedPlan::getStateVal() -> std::vector<double>
	{ 
		return this->calcuStateVal();
	}
	// ����Ŀ��λ�ú�Ȩ�أ�
	fixedPlan::fixedPlan(std::vector<double> targetdata, std::vector<double> weightdata) {
		
		// Determine whether the number of weights is twice the number of target values
		// calculate state Acc need two 
		if (!(2 * targetdata.size() == weightdata.size()))
			throw std::runtime_error("The number of target values does not match the number of weights");

		stateVar_.push_back(stateImp(targetdata[0], weightdata[0], weightdata[1]));
		stateVar_.push_back(stateImp(targetdata[1], weightdata[2], weightdata[3]));

	}

	fixedPlan::~fixedPlan()  = default;


}











