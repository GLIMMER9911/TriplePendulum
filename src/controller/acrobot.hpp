#ifndef ACROBOT_H
#define ACROBOT_H

#include <aris.hpp>
#include <iostream>
#include <fstream>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../utils/utils.hpp"
#include "../solver/math.hpp"
#include "../solver/qpsolver.hpp"

namespace acrobot {

	using namespace aris::dynamic;

	class  Acrobot
	{
	public:
		void getStateVar(std::vector<double>& data, std::vector<double>& acc);
		void setQPparameters(std::vector<double>& lambda_data);
		void estimateState();
		void calculateCoMJacobian();
		void cptModelCm();
		void cptModelAngularMoment();
		void cptdesiredCoMAcc();
		void calculateTorque();
		void calcAandBColumn(double* force);
		void calculateAandB();
		void checkrealAcc(int m, int n);
		auto sendTorque() -> std::vector<double>;
		void verifyAccelerate(std::vector<double>& data);
		void calcuForwardKinematics(std::vector<double>& data);
		void setModel(Model* model);
		void test();

		Acrobot();
		~Acrobot();

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}

#endif // ! ACROBOT_H