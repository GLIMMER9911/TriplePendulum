#ifndef CONTROL_H
#define CONTROL_H

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <cmath>

#include "export.h"


namespace triple {

	using namespace aris::dynamic;
	using namespace triple;

	class BAL_CTRL_API Controller
	{
	public:
		void init(Model* model);
		void getStateVar(std::vector<double>& data, std::vector<double>& acc);
		void setQPparameters(double* lambda_data, int lambda_size);
		void estimateState();
		void cptModelCm();
		void cptModelAngularMoment();
		void calcAandBColumn(double* force);
		void calculateAandB();
		void cptdesiredCoMAcc();
		void calculateTorque();
		auto getDesiredAcc() -> double* { return lastrealAcc.data(); };
		auto sendTorque() -> std::vector<double>;
		void verifyAccelerate(std::vector<double>& data);
		void calculateCoMJacobian();		
		void checkrealAcc(int m, int n);
		void calcuForwardKinematics(std::vector<double>& data);


		Controller(const Controller& other);
		explicit Controller(int outputSize = 2, int intputSize = 3);
		explicit Controller(Model* model, int outputSize = 2, int intputSize = 3);
		~Controller();

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
		std::vector<double> lastTorque;
		std::vector<double> lastrealAcc{0.0, 0.0, 0.0 };

	};


}







#endif // !CONTROL_H
