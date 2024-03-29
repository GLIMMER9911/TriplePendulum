#ifndef CONTROL_H
#define CONTROL_H

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>




namespace triple {

	using namespace aris::dynamic;
	using namespace triple;

	class  Controller
	{
	public:
		void init(Model* model);
		void getStateVar(std::vector<double>& data, std::vector<double>& acc);
		void setQPparameters(std::vector<double>& lambda_data);
		void estimateState();
		void cptModelCm();
		void cptModelAngularMoment();
		void calcAandBColumn(double* force);
		void calculateAandB();
		void cptdesiredCoMAcc();
		void calculateTorque();
		void arisCalcuTorque();
		auto getlastRealAcc() -> double* { return lastrealAcc.data(); };
		void sendDesiredAcc(std::vector<double>& desireddata);
		auto sendTorque() -> std::vector<double>;
		void verifyAccelerate(std::vector<double>& data);
		void calculateCoMJacobian();		
		void checkrealAcc(int m, int n);
		void calcuForwardKinematics(std::vector<double>& data);
		void dspComputingInformation(int period = 100);
		void cptdesired7Acc();
		void osqpMatrix7x2test();
		void arisMatrix7x2test();

		Controller(const Controller& other);
		explicit Controller(int outputSize = 2, int intputSize = 3);
		explicit Controller(Model* model, int outputSize = 2, int intputSize = 3);
		~Controller();

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
		std::vector<double> lastTorque;
		std::vector<double> lastrealAcc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	};


}







#endif // !CONTROL_H
