// model.h: 标准系统包含文件的包含文件

#ifndef MODEL_H
#define MODEL_H

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>


using namespace aris::dynamic;

class TripleModel : public aris::dynamic::Model
{
public:
	auto CreateModel() -> std::unique_ptr<Model>;
	void getstate_var(std::vector<double>& data);
	void getmodel() { m_ = CreateModel(); }
	std::vector<double> send_torque();
	void calcu_torque();
	auto desiredAcc() -> double* { return desired_acc_.data(); }
	std::vector<double> calcu_forwardKinematics(std::vector<double>& joint_angle);
	TripleModel();
	~TripleModel();

private:
	std::unique_ptr<Model> m_;
	std::vector<double> state_var;
	std::vector<double> torque, last_toq_{0.0, 0.0};
	std::vector<double> desired_acc_{0.0, 0.0, 0.0, 0.0, 0.0};
};



#endif // !MODEL_H