#include "control.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "../utils/utils.hpp"
#include "./../solver/math.hpp"
#include "./../solver/qpsolver.hpp"

namespace triple {

	// calculate Controller implementation
	struct Controller::Imp {
		int count_ = 0;
		Controller* controller_;
		std::shared_ptr<aris::dynamic::Model> m_;
		double cm_pos[3]{ 0.0, 0.0, 0.0 };
		double cm_vel[3]{ 0.0, 0.0, 0.0 };
		double cm_acc[3]{ 0.0, 0.0, 0.0 };
		double moment[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double lambda[5]{ 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::vector<double> stateVar;
		std::vector<double> desiredAcc;
		std::vector<double> torque;
		Eigen::MatrixXd A;
		Eigen::MatrixXd B;
		Eigen::MatrixXd COMJacobian;
		Eigen::VectorXd COMCf;

		Imp(Controller* controller) : controller_(controller) {
			A = Eigen::MatrixXd::Zero(3, 2);
			B = Eigen::MatrixXd::Zero(3, 1);
		}
	};

	// data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
	void Controller::getStateVar(std::vector<double>& data, std::vector<double>& acc) {

		//std::cout << "data: ";
		//std::cout << data.size() << std::endl;
		//for (int i = 0; i < data.size(); ++i) {
		//	std::cout << data[i] << " ";
		//}
		//std::cout << std::endl;

		std::copy(data.begin(), data.begin() + 6, imp_->stateVar.begin());
		std::copy(data.end() - 7, data.end() - 1, imp_->stateVar.end() - 6);

		std::copy(acc.begin(), acc.end(), imp_->desiredAcc.begin());
		//std::cout << imp_->desiredAcc.size() << std::endl;
		//for (int i = 0; i < imp_->desiredAcc.size(); ++i) {
		//	std::cout << imp_->desiredAcc[i] << " ";
		//}
		//std::cout << std::endl;
	}

	// set QP parameters 
	void Controller::setQPparameters(double* lambda_data, int lambda_size) {
		for (int i = 0; i < lambda_size; i++) {
			imp_->lambda[i] = lambda_data[i];
		}
	}

	// 进行状态估计 
	// stateVar: joint1, joint2, joint3, w1, w2, w3, ax, ay, bz, ja1, ja2, ja3
	void Controller::estimateState() {
		// 
		double joint_pos[3][6] = { {0, 0, 0, 0, 0, imp_->stateVar[0]},
								   {0, 0, 0, 0, 0, imp_->stateVar[1]},
								   {0, 0, 0, 0, 0, imp_->stateVar[2]},
		};
		double joint_velocity[3][6] = { {0, 0, 0, 0, 0, imp_->stateVar[3]},
										{0, 0, 0, 0, 0, imp_->stateVar[4]},
										{0, 0, 0, 0, 0, imp_->stateVar[5]}
		};
		double joint_accelerate[3][6] = { {0, 0, 0, 0, 0, imp_->stateVar[9]},
										  {0, 0, 0, 0, 0, imp_->stateVar[10]},
										  {0, 0, 0, 0, 0, imp_->stateVar[11]}
		};


		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		// calculate the forward position
		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->setPe(*imp_->m_->jointPool().at(i).makJ(), joint_pos[i], "123");
		}
		for (auto& m : imp_->m_->motionPool()) m.updP();
		ee.updP();

		// calculate the forward velocity
		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setVs(*imp_->m_->jointPool().at(i).makJ(), joint_velocity[i]);
		}
		for (auto& m : imp_->m_->motionPool()) m.updV();
		ee.updV();

		// calculate the forward accelerate
		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), joint_accelerate[i]);
		}
		for (auto& m : imp_->m_->motionPool()) m.updA();
		ee.updA();

		this->cptModelCm();
		this->cptModelAngularMoment();
		this->calculateCoMJacobian();

	}

	void Controller::calculateCoMJacobian() {
		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		auto& forward_kinematic_solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&> (imp_->m_->solverPool().at(1));

		////////////////////////////////////////////////////////////////// COM Accelerate test ///////////////////////////////////////////////////////////////////////////
		this->cptModelCm();
		double cx_acc = imp_->cm_acc[0];


		Eigen::VectorXd qdd(3);

		//[q1dd xdd ydd] =  Jacobian*qdd + Cf 
		Eigen::VectorXd Cf(3);
		Eigen::MatrixXd Jacobian(3, 3);
		Jacobian.setZero();

		// cx_acc = CoMJacobian(1,3)*[qdd1, xdd, ydd ] + CoMCf
		Eigen::MatrixXd CoMJacobian(1, 3);
		Eigen::VectorXd CoMCf(1);

		// �����ؽڵļ��ٶ�
		qdd << imp_->stateVar[9], imp_->stateVar[10], imp_->stateVar[11];

		// ���Ա���
		double test0[3]{ 0, 0, 0 };
		double test1[3]{ 1, 0, 0 };
		double test2[3]{ 0, 1, 0 };
		double test3[3]{ 0, 0, 1 };
		double ee_Maa[6]{ 0, 0, 0, 0, 0, 0 };

		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMa(test0[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		cptModelCm();
		ee.getMaa(ee_Maa);
		Cf << 0, ee_Maa[0], ee_Maa[1];
		CoMCf(0, 0) = imp_->cm_acc[0];


		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMa(test1[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		cptModelCm();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 0, 3, 1) << 1, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 0) = (imp_->cm_acc[0] - CoMCf[0]);

		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMa(test2[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		cptModelCm();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 1, 3, 1) << 0, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 1) = (imp_->cm_acc[0] - CoMCf[0]);
		//std::cout << std::endl;
		//std::cout << " Jacobian1: " << Jacobian << std::endl;


		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMa(test3[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		cptModelCm();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 2, 3, 1) << 0, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 2) = (imp_->cm_acc[0] - CoMCf[0]);

		//Eigen::Vector<double, 3> answer;
		//answer = Jacobian * qdd + Cf;

		//Eigen::Vector<double, 1> com_answer;
		//com_answer = CoMJacobian * qdd + CoMCf;

		//Eigen::Vector<double, 1> com_answer_;
		CoMJacobian = CoMJacobian * computePeseudoInverse(Jacobian);
		CoMCf = -CoMJacobian * Cf + CoMCf;
		//com_answer_ = CoMJacobian * answer + CoMCf;

		imp_->COMJacobian = CoMJacobian;
		imp_->COMCf = CoMCf;

		//std::cout << " q1_accelerate: " << qdd(0) << "  x acc: " << imp_->stateVar[6] << "  y acc: " << imp_->stateVar[7] << std::endl;
		//std::cout << " the answer is: " << answer(0) << "  x acc: " << answer(1) << "  y acc: " << answer(2) << std::endl;

		//std::cout << " CoM x acc is : " << cx_acc << std::endl;
		//std::cout << " the answer is: " << com_answer(0, 0) << std::endl;
		//std::cout << " the answer_ is: " << com_answer_(0, 0) << std::endl;

		////////////////////////////////////////////////////////////////// COM Accelerate test ///////////////////////////////////////////////////////////////////////////

		//Eigen::VectorXd Qdd(2);
		//Qdd << imp_->stateVar[10], imp_->stateVar[11];

		//cptModelCm();
		//double cxa = imp_->cm_acc[0];

		//double test0[3]{ 0.0, 0.0, 0.0 };
		//double test1[3]{ 1.0, 0.0, 0.0 };
		//double test2[3]{ 0.0, 1.0, 0.0 };
		////double test3[3]{ 0.0, 0.0, 1.0 };

		//double ee_Maa[6]{ 0, 0, 0, 0, 0, 0 };

		//Eigen::VectorXd Cf(3);
		//Eigen::MatrixXd Jacobian(3, 2);
		//Cf.setZero();
		//Jacobian.setZero();

		//imp_->m_->motionPool().at(0).setMa(imp_->stateVar[9]);
		//for (int i = 1; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test0[i-1]);
		//}
		//forward_kinematic_solver.dynAccAndFce();
		//cptModelCm();
		//ee.getMaa(ee_Maa);
		//Cf << ee_Maa[0], ee_Maa[1], imp_->cm_acc[0];

		//Jacobian.setZero();

		//for (int i = 1; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test1[i-1]);
		//}
		//forward_kinematic_solver.dynAccAndFce();
		//cptModelCm();
		//ee.getMaa(ee_Maa);
		//Jacobian.block(0, 0, 3, 1) << (ee_Maa[0] - Cf(0, 0)), (ee_Maa[1] - Cf(1, 0)), (imp_->cm_acc[0] - Cf(2, 0));
		////std::cout << std::endl;
		////std::cout << "Jacobian2: " << Jacobian << std::endl;


		//////imp_->m_->motionPool().at(0).setMa(imp_->stateVar[9]);
		//for (int i = 1; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test2[i-1]);
		//}
		//forward_kinematic_solver.dynAccAndFce();
		//cptModelCm();
		//ee.getMaa(ee_Maa);
		//Jacobian.block(0, 1, 3, 1) << (ee_Maa[0] - Cf(0, 0)), (ee_Maa[1] - Cf(1, 0)), (imp_->cm_acc[0] - Cf(2, 0));
		////std::cout << std::endl;
		////std::cout << "Jacobian2: " << Jacobian << std::endl;

		////for (int i = 0; i < 3; ++i) {
		////	imp_->m_->motionPool().at(i).setMa(test3[i]);
		////}
		////forward_kinematic_solver.dynAccAndFce();
		////cptModelCm();
		////ee.getMaa(ee_Maa);
		////Jacobian.block(0, 2, 3, 1) << (ee_Maa[0] - Cf(0, 0)), (ee_Maa[1] - Cf(1, 0)), (imp_->cm_acc[0] - Cf(2, 0));
		////std::cout << "Jacobian: " << Jacobian << std::endl;


		//Eigen::VectorXd test_acc(3);
		//test_acc = Jacobian * Qdd + Cf;

		//std::cout << "Cf:" << Cf << std::endl;
		//std::cout << "Jacobian: " << Jacobian << std::endl;
		//std::cout << "test Acc: " << test_acc(0, 0) << " " << test_acc(1, 0) << " " << test_acc(2, 0) << " " << std::endl;
		//std::cout << "before Acc: " << imp_->stateVar[6] << " " << imp_->stateVar[7] << " " << cxa << " " << std::endl;


		////////////////////////////////////////////////////////////////// COM Velocity test ///////////////////////////////////////////////////////////////////////////
		/*cptModelCm();
		double com_x_vel = imp_->cm_vel[0];

		double test0[3]{ 0, 0, 0 };
		double test1[3]{ 1, 0, 0 };
		double test2[3]{ 0, 1, 0 };
		double test3[3]{ 0, 0, 1 };

		Eigen::MatrixXd Mv(3, 1);
		Mv << imp_->stateVar[3], imp_->stateVar[4], imp_->stateVar[5];

		Eigen::MatrixXd HV(1, 3);

		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMv(test1[i]);
		}
		imp_->m_->solverPool().at(1).kinVel();
		cptModelCm();
		HV(0, 0) = imp_->cm_vel[0];

		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMv(test2[i]);
		}
		imp_->m_->solverPool().at(1).kinVel();
		cptModelCm();
		HV(0, 1) = imp_->cm_vel[0];

		for (int i = 0; i < 3; ++i) {
			imp_->m_->motionPool().at(i).setMv(test3[i]);
		}
		imp_->m_->solverPool().at(1).kinVel();
		cptModelCm();
		HV(0, 2) = imp_->cm_vel[0];

		auto cx_vel = HV * Mv;

		std::cout << " Hv : " << HV << std::endl;

		std::cout << " com x velocity: " << com_x_vel << " ?= " << cx_vel(0, 0) << " ?= " << (cx_vel(0,0) / com_x_vel) << std::endl;*/




		//double joint_velocity[3][6] = { {0, 0, 0, 0, 0, imp_->stateVar[3]},
		//								{0, 0, 0, 0, 0, imp_->stateVar[4]},
		//								{0, 0, 0, 0, 0, imp_->stateVar[5]}
		//};
		//
		//double x1[3], x2[3], x3[3];

		//imp_->m_->jointPool().at(0).makI()->getPp(x1);
		////std::cout << " x1 : " << x1[0] << " " << x1[1] << " " << x1[2] << " " << std::endl;

		//imp_->m_->jointPool().at(1).makI()->getPp(x2);
		////std::cout << " x2 : " << x2[0] << " " << x2[1] << " " << x2[2] << " " << std::endl;

		//imp_->m_->jointPool().at(2).makI()->getPp(x3);
		////std::cout << " x3 : " << x3[0] << " " << x3[1] << " " << x3[2] << " " << std::endl;

		//double cx1[3], cx2[3], cx3[3];
		//for (int i = 0; i < 3; ++i) {
		//	cx1[i] = x1[i] - imp_->cm_pos[i];
		//	cx2[i] = x2[i] - imp_->cm_pos[i];
		//	cx3[i] = x3[i] - imp_->cm_pos[i];
		//}

		//double vx1[3], vx2[3], vx3[3];
		//s_vs2vp(joint_velocity[0], cx1, vx1);
		//s_vs2vp(joint_velocity[1], cx2, vx2);
		//s_vs2vp(joint_velocity[2], cx3, vx3);
		//
		////double cx_vel = vx1[0] + vx2[0] + vx3[0];

		//std::cout << " joint velocity: " << imp_->stateVar[3] << " " << imp_->stateVar[4] << " " << imp_->stateVar[5] << " " << std::endl;
		//std::cout << " Com distance 1: " << cx1[0] << " " << cx1[1] << " " << cx1[2] << " " << std::endl;
		//std::cout << " Com distance 2: " << cx2[0] << " " << cx2[1] << " " << cx2[2] << " " << std::endl;
		//std::cout << " Com distance 3: " << cx3[0] << " " << cx3[1] << " " << cx3[2] << " " << std::endl;




		//////////////////////////////////////////////////////////////////// COM Accelerate test1 ///////////////////////////////////////////////////////////////////////////

		//cptModelCm();
		//double com_x_acc = imp_->cm_acc[0];
		////double test0[3]{ 0, 0, 0 };
		////double test1[3]{ 1, 0, 0 };
		////double test2[3]{ 0, 1, 0 };
		////double test3[3]{ 0, 0, 1 };
		//Eigen::VectorXd Ma(3, 1);

		//double ma1 = imp_->stateVar[9];
		//double ma2 = imp_->stateVar[10];
		//double ma3 = imp_->stateVar[11];
		//Ma << ma1, ma2, ma3;


		//Eigen::MatrixXd H(1, 3);
		//Eigen::VectorXd b(1);
		//
		//
		//imp_->m_->motionPool().at(0).setMa(0.0);
		//imp_->m_->generialMotion

		//imp_->m_->solverPool().at(1).dynAccAndFce();
		//cptModelCm();
		//b(0) = imp_->cm_acc[0];

		//for (int i = 0; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test1[i]);
		//}
		//imp_->m_->solverPool().at(1).dynAccAndFce();
		//cptModelCm();
		//H(0, 0) = imp_->cm_acc[0];


		//for (int i = 0; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test2[i]);
		//}
		//imp_->m_->solverPool().at(1).dynAccAndFce();
		//cptModelCm();
		//H(0, 1) = imp_->cm_acc[0];

		//for (int i = 0; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test3[i]);
		//}
		//imp_->m_->solverPool().at(1).dynAccAndFce();
		//cptModelCm();
		//H(0, 2) = imp_->cm_acc[0];
		//auto testXa = H * Ma + b ;

		//std::cout << " Ha: " << H << std::endl;
		//std::cout << " COM X Accelerate is : " << com_x_acc << " ?=  " << testXa(0,0) << "" << std::endl;

	}

	// 计算质心位置：
	// c  = (m1*c1 + m2*c2 + m3*c3)/(m1+m2+m3);
	// vc = (m1*vc1 + m2*vc2 + m3*vc3)/(m1+m2+m3);
	// ac = (m1*ac1 + m2*ac2 + m3*ac3)/(m1+m2+m3);
	void Controller::cptModelCm() {
		double im1[36], im2[36], im3[36];
		imp_->m_->partPool()[1].cptGlbIm(im1, 6);
		imp_->m_->partPool()[2].cptGlbIm(im2, 6);
		imp_->m_->partPool()[3].cptGlbIm(im3, 6);
		double m1{ im1[0] }, m2{ im2[0] }, m3{ im3[0] };

		double x1[3]{ im1[11] / m1, im1[15] / m1 ,im1[4] / m1 };
		double x2[3]{ im2[11] / m2, im2[15] / m2 ,im2[4] / m2 };
		double x3[3]{ im3[11] / m3, im3[15] / m3 ,im3[4] / m3 };

		double vx1[3], vx2[3], vx3[3], ax1[3], ax2[3], ax3[3];

		s_as2ap(imp_->m_->partPool()[1].vs(), imp_->m_->partPool()[1].as(), x1, ax1, vx1);
		s_as2ap(imp_->m_->partPool()[2].vs(), imp_->m_->partPool()[2].as(), x2, ax2, vx2);
		s_as2ap(imp_->m_->partPool()[3].vs(), imp_->m_->partPool()[3].as(), x3, ax3, vx3);

		imp_->cm_pos[0] = (x1[0] * m1 + x2[0] * m2 + x3[0] * m3) / (m1 + m2 + m3);
		imp_->cm_pos[1] = (x1[1] * m1 + x2[1] * m2 + x3[1] * m3) / (m1 + m2 + m3);
		imp_->cm_pos[2] = (x1[2] * m1 + x2[2] * m2 + x3[2] * m3) / (m1 + m2 + m3);

		imp_->cm_vel[0] = (vx1[0] * m1 + vx2[0] * m2 + vx3[0] * m3) / (m1 + m2 + m3);
		imp_->cm_vel[1] = (vx1[1] * m1 + vx2[1] * m2 + vx3[1] * m3) / (m1 + m2 + m3);
		imp_->cm_vel[2] = (vx1[2] * m1 + vx2[2] * m2 + vx3[2] * m3) / (m1 + m2 + m3);

		imp_->cm_acc[0] = (ax1[0] * m1 + ax2[0] * m2 + ax3[0] * m3) / (m1 + m2 + m3);
		imp_->cm_acc[1] = (ax1[1] * m1 + ax2[1] * m2 + ax3[1] * m3) / (m1 + m2 + m3);
		imp_->cm_acc[2] = (ax1[2] * m1 + ax2[2] * m2 + ax3[2] * m3) / (m1 + m2 + m3);
	}

	// calculate the Model Angular Moment
	// Spatial Inertia
	// | 1 0 0    0        z   -y      | 
	// | 0 1 0   -z        0    x      |
	// | 0 0 1    y       -x    0      |
	// | 0 -z y  y^2+z^2 -xy   -xz     |
	// | z 0 -x  -xy   x^2+z^2 -yz     |
	// |-y x 0   -xz     -yz  x^2+y^2  |
	void Controller::cptModelAngularMoment() {
		double im1[36], im2[36], im3[36];
		imp_->m_->partPool()[1].cptGlbIm(im1, 6);
		imp_->m_->partPool()[2].cptGlbIm(im2, 6);
		imp_->m_->partPool()[3].cptGlbIm(im3, 6);

		s_mm(6, 1, 6, im1, imp_->m_->partPool()[1].vs(), imp_->moment);
		// s_mma 
		s_mma(6, 1, 6, im2, imp_->m_->partPool()[2].vs(), imp_->moment);
		s_mma(6, 1, 6, im3, imp_->m_->partPool()[3].vs(), imp_->moment);

		//std::cout << "�����Լ��Ƕ����� " << imp_->moment[0] << " " << imp_->moment[1] << " "  << imp_->moment[5] << " " << std::endl;

	}

	// compute desired CoM accelerate
	void Controller::cptdesiredCoMAcc() {
		////////////////////////////////// 质心处的加速度 /////////////////////////////////////////////
		// 质心加速度 x 方向, 经过测试质心处的最大加速度为 6, 最好应该小于 5 ，系统才能控制得住。
		static double max_acc = 0;
		double cx_d = imp_->moment[5] * 0.05;
		double kp_cx = 10;
		double kp_vcx = 50;
		double vx_d = kp_cx * (cx_d - imp_->cm_pos[0]);
		vx_d = std::max(vx_d, -0.1);
		vx_d = std::min(vx_d, 0.1);
		double ax_d = kp_vcx * (vx_d - imp_->cm_vel[0]);
		ax_d = std::max(ax_d, -1.0);
		ax_d = std::min(ax_d, 1.0);
		imp_->desiredAcc[2] = ax_d + 0 * (imp_->COMJacobian(0, 1) * imp_->desiredAcc[0] );
		 
		// std::cout << "cm_vel: " << imp_->cm_vel[0] << std::endl; 
		// std::cout << "ax_d: " << ax_d << std::endl; 
		// std::cout << "imp_->desiredAcc[2]: " << imp_->desiredAcc[2] << std::endl;
	}

	// calculate A and B column
	void Controller::calcAandBColumn(double* force) {

		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		auto& force1 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(0));
		auto& force2 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(1));
		auto& forward_dynamic_solver = dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(imp_->m_->solverPool().at(3));

		force1.setFce(force[0]);
		force2.setFce(force[1]);
		forward_dynamic_solver.dynAccAndFce();

		// 得到末端位置
		double pp[6]{ 0.0, 0.0, 0.0 }, vp[3]{ 0.0, 0.0, 0.0 }, ap[3]{ 0.0, 0.0, 0.0 };
		ee.getMpe(pp);
		aris::dynamic::s_as2ap(imp_->m_->partPool().back().vs(), imp_->m_->partPool().back().as(), pp, ap, vp);

		// 计算质心加速度
		this->cptModelCm();

		// 电机端, 每个关节的加速度；
		double aj1 = 0.0;
		double as[6];
		for (auto& m : imp_->m_->motionPool()) m.updA();
		imp_->m_->jointPool()[0].makI()->getAs(*imp_->m_->jointPool()[0].makJ(), as);
		aj1 = as[5];

		if (imp_->count_ < 3000) {
			if ((force[0] == 0) && (force[1] == 0)) {
				// get b
				imp_->B(0, 0) = ap[0];
				imp_->B(1, 0) = ap[1];
				imp_->B(2, 0) = imp_->cm_acc[0];
			}
			else if ((force[0] == 1) && (force[1] == 0)) {
				imp_->A(0, 0) = ap[0] - imp_->B(0, 0);
				imp_->A(1, 0) = ap[1] - imp_->B(1, 0);
				imp_->A(2, 0) = imp_->cm_acc[0] - imp_->B(2, 0);
			}
			else if ((force[0] == 0) && (force[1] == 1)) {
				imp_->A(0, 1) = ap[0] - imp_->B(0, 0);
				imp_->A(1, 1) = ap[1] - imp_->B(1, 0);
				imp_->A(2, 1) = imp_->cm_acc[0] - imp_->B(2, 0);
			}
			else {
				std::cout << " ... ... " << std::endl;
				std::cerr << "force error!" << std::endl;
				std::cout << force[0] << " " << force[1] << " " << std::endl;
				exit(1);
			}
		}
		else {
			if ((force[0] == 0) && (force[1] == 0)) {
				// get b
				imp_->B(0, 0) = ap[0];
				imp_->B(1, 0) = ap[1];
				//imp_->B(2, 0) = imp_->cm_acc[0];
			}
			else if ((force[0] == 1) && (force[1] == 0)) {
				imp_->A(0, 0) = ap[0] - imp_->B(0, 0);
				imp_->A(1, 0) = ap[1] - imp_->B(1, 0);
				//imp_->A(2, 0) = imp_->cm_acc[0] - imp_->B(2, 0);

			}
			else if ((force[0] == 0) && (force[1] == 1)) {
				imp_->A(0, 1) = ap[0] - imp_->B(0, 0);
				imp_->A(1, 1) = ap[1] - imp_->B(1, 0);
				//imp_->A(2, 1) = imp_->cm_acc[0] - imp_->B(2, 0);

			}
			else {
				std::cout << " ... ... " << std::endl;
				std::cerr << "force error!" << std::endl;
				std::cout << force[0] << " " << force[1] << " " << std::endl;
				exit(1);
			}
		}

	}

	// 1.  [ddx, ddy]^T = A * torque + b
	// 2.  t = A^(-1) * [ddx, ddy]^T -  A^(-1)*b
	void Controller::calculateAandB() {

		double force[2]{ 0.0, 0.0 };

		force[0] = 0.0;
		force[1] = 0.0;
		calcAandBColumn(force);

		force[0] = 1.0;
		force[1] = 0.0;
		calcAandBColumn(force);

		force[0] = 0.0;
		force[1] = 1.0;
		calcAandBColumn(force);

	}
	
	// Because the communication has ONE STEP ERROR, we need to check  whether the calculate Acc is as same as the real Acc. 
	void Controller::checkrealAcc(int m, int n) {

		Eigen::MatrixXd torq = Eigen::MatrixXd::Zero(n, 1);
		Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(m, 1);

		torq(0, 0) = lastTorque[0];
		torq(1, 0) = lastTorque[1];
		acc = imp_->A * torq + imp_->B;
		lastrealAcc[0] = acc(0, 0);
		lastrealAcc[1] = acc(1, 0);
		lastrealAcc[2] = acc(2, 0);

	}

	// calculate torque
	void Controller::calculateTorque() {
		// size of input Accelerate
		int ipt_size = 3;
		// size of output torque
		int opt_size = 2;

		this->estimateState();
		this->cptdesiredCoMAcc();
		this->calculateAandB();

		// Because the communication has ONE STEP ERROR, we need to check  whether the calculate Acc is as same as the real Acc. 
		this->checkrealAcc(ipt_size, opt_size);

		//this->inverseMethodsolveTorque();

		///////////////////////////////// QP ���������� ///////////////////////////////////////////////

		///////////////////// Set NEXT desired Acc //////////////////////
		Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(ipt_size, 1);
		Eigen::MatrixXd error(3, 1);
		double comAcc_error = 0;
		double abs_comAcc = std::abs(imp_->desiredAcc[2]);

		// ������������CAM����ƿ���CAM���������κ���
		if ((abs_comAcc > 0) && (abs_comAcc < 0.2)) {
			comAcc_error = 0.2;
		}
		else if((abs_comAcc > 0.2) && (abs_comAcc < 0.6)) {
			comAcc_error = 0.2 - 0.5 * (abs_comAcc - 0.2);
		}
		else {
			comAcc_error = 0.0;
		}

		error << 0, 0.0, 0;// comAcc_error;

		// desired Acc stateVar[6] stateVar[7] is calculated by PID, ax_d is the accelerate of CoM x positon.  
		Acc(0, 0) = imp_->desiredAcc[0];
		Acc(1, 0) = imp_->desiredAcc[1];
		Acc(2, 0) = imp_->desiredAcc[2];

		// m1, m2, ax, ay, acx
		double lambda[5]{ 0.01, 0.01, 1, 1, 10};
		//if (imp_->count_ > 3000 ) {
		//	lambda[2] = 1;
		//	lambda[3] = 1;
		//	lambda[4] = 2;
		//}

		const int n = opt_size;
		const int m = ipt_size;

		// P
		const int size_P = m + n;
		Eigen::MatrixXd P(size_P, size_P);
		P.setZero();
		P.diagonal() << lambda[0], lambda[1], lambda[2], lambda[3], lambda[4];

		// q
		Eigen::MatrixXd q(n + m, 1);
		q.setZero();

		// A 
		//Eigen::Matrix3d weight;
		//weight.setZero();
		//weight.diagonal() << 1, 1, 1 ;
		Eigen::MatrixXd Ad = imp_->A;
		Eigen::MatrixXd QP_A(m + n, m + n);
		QP_A.setZero();
		QP_A.block(0, 0, m, n) = Ad;
		QP_A.block(0, n, m, m) = (-1) * Eigen::MatrixXd(m, m).setIdentity();
		QP_A.block(m, 0, n, n) = Eigen::MatrixXd(n, n).setIdentity();

		// l
		Eigen::MatrixXd l = Eigen::MatrixXd::Zero(n + m, 1);
		l.block(0, 0, m, 1) = (Acc - imp_->B - error);
		l.block(m, 0, n, 1) << -100, -100;

		// u
		Eigen::MatrixXd u = Eigen::MatrixXd::Zero(n + m, 1);
		u.block(0, 0, m, 1) = (Acc - imp_->B + error);
		u.block(m, 0, n, 1) << 100, 100;

		// x 
		Eigen::MatrixXd x = Eigen::MatrixXd::Zero(n + m, 1);
		x = qpSolver(P, q, QP_A, l, u);

		//std::cout << "qp solver: " << x << std::endl;


		/* using aris::dynamic::s_quadprog

		Eigen::MatrixXd torque = x.block(0, 0, n, 1);

		const aris::Size nG = 5, nCE = 3, nCI = 4;

		double aris_G[nG * nG]{ lambda[0], 0, 0, 0, 0,
								0, lambda[1], 0, 0, 0,
								0, 0, lambda[2], 0, 0,
								0, 0, 0, lambda[3], 0,
								0, 0, 0, 0, lambda[4],
		};

		double aris_g[nG]{ 0.0, 0.0, 0.0, 0.0, 0.0 };
		double aris_CE[nCE * nG]{ imp_->A(0, 0), imp_->A(0, 1), -1, 0.0, 0.0,
								  imp_->A(1, 0), imp_->A(1, 1), 0.0, -1, 0.0,
								  imp_->A(2, 0), imp_->A(2, 1), 0.0, 0.0, -1,
		};

		double aris_ce[nCE]{ce(0, 0), ce(1, 0), ce(2, 0)};

		double aris_CI[nCI * nG]{ 1, 0, 0, 0, 0,
								  0, 1, 0, 0, 0,
								  -1, 0, 0, 0, 0,
								  0, -1, 0, 0, 0
		};

		double aris_ci[nCI]{ 100, 100, 100, 100 };

		std::vector<double> result(nG), mem(nG * nG * 2 + 8 * (nCE + nCI) + 3 * nG);
		auto r = aris::dynamic::s_quadprog(nG, nCE, nCI, aris_G, aris_g, aris_CE, aris_ce, aris_CI, aris_ci, result.data(), mem.data());
		*/

		Eigen::MatrixXd torque = x.block(0, 0, n, 1);

		auto max_fce = 1000.0;
		imp_->torque[0] = std::min(torque(0, 0), max_fce);
		imp_->torque[0] = std::max(torque(0, 0), -max_fce);
		imp_->torque[1] = std::min(torque(1, 0), max_fce);
		imp_->torque[1] = std::max(torque(1, 0), -max_fce);

		if (imp_->count_ % 500 == 0) {
				std::cout << "imp_->count_ " << imp_->count_ << " ---------------------------------------------------- " << std::endl;
				std::cout << " " << std::endl;
				std::cout << "imp_->A: \n" << imp_->A << std::endl;
				std::cout << "Acc: " << Acc(0, 0) << " " << Acc(1, 0) << " " << Acc(2, 0) << std::endl;
				std::cout << "torque: " << torque(0, 0) << "  " << torque(1, 0) << " " << std::endl;
				std::cout << "CoM x com: " << imp_->cm_pos[0] << " " << imp_->cm_pos[1] << " " << std::endl;
			//	/*std::cout << " Acc:  " << x(2, 0) << " " << x(3, 0) << " " << x(4, 0) << " " << std::endl;
				Eigen::MatrixXd accAfter = imp_->A * torque + imp_->B;
				std::cout << " Acc After: " << accAfter(0, 0) << "  " << accAfter(1, 0) << "  "
					<< accAfter(2, 0) << std::endl;
		}

		//std::cout << torque[0] << "  " << torque[1] << " " << std::endl;

		std::vector<double> acc_(2);
		acc_[0] = lastrealAcc[0];
		acc_[1] = lastrealAcc[1];

		//std::string str = "state var";
		//saveDataTofile(imp_->stateVar, str, 0);

		//str = "torque";
		//saveDataTofile(imp_->torque, str, 0);

		//str = "accelerate";
		//saveDataTofile(acc_, str, true);

		//std::ofstream outputFile("CoM_P.txt", std::ios::app);
		//if (outputFile.is_open()) {
		//	outputFile << imp_->cm_pos[0] << "  " << imp_->cm_pos[1] << "  " << std::endl;
		//};
		//outputFile.close();

		lastTorque = imp_->torque;

		//std::cout << "imp_->torque[0]: " << imp_->torque[0] << "imp_->torque[1]: " << imp_->torque[1] << std::endl;
	}

	// send torque
	auto Controller::sendTorque()->std::vector<double> {
		imp_->count_++;
		this->calculateTorque();
		return imp_->torque;
	}

	// verify the model accelerate 
	// data : joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
	void Controller::verifyAccelerate(std::vector<double>& data) {

		double error = 0;

		for (int i = 0; i < lastrealAcc.size() - 1; ++i) {
			error = std::max(std::abs(data[12 + i] - lastrealAcc[i]), error);
		}

		if (error > 1e-1 * 1.0)
		{
			std::cout << "acc desired:";
			for (int i = 0; i < lastrealAcc.size(); i++) {
				std::cout << lastrealAcc[i] << " ";
			}

			std::cout << "data back: " << std::endl;
			std::cout << data[12] << " " << data[13] << " " << data[15] << " "
				<< data[16] << " " << data[17] << " " << std::endl;
			std::cout << "---------------------------- " << std::endl;
			throw std::runtime_error("error : acc data not correct");
		}
	}

	void Controller::calcuForwardKinematics(std::vector<double>& data) {
		double Joint_pos[3][6] = { {0, 0, 0, 0, 0, data[0]},
								   {0, 0, 0, 0, 0, data[1]},
								   {0, 0, 0, 0, 0, data[2]} };
		double joint_velocity[3][6] = { {0, 0, 0, 0, 0, data[3]},
								   {0, 0, 0, 0, 0, data[4]},
								   {0, 0, 0, 0, 0, data[5]} };

		for (int i = 0; i < 3; i++) {
			imp_->m_->jointPool().at(i).makI()->setPe(*imp_->m_->jointPool().at(i).makJ(), Joint_pos[i], "123");
		}
		for (auto& m : imp_->m_->motionPool()) m.updP();
		imp_->m_->generalMotionPool()[0].updP();

		// calculate the forward velocity
		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setVs(*imp_->m_->jointPool().at(i).makJ(), joint_velocity[i]);
		}
		for (auto& m : imp_->m_->motionPool()) m.updV();
		imp_->m_->generalMotionPool()[0].updV();

		double ee_position[6] = { 0,0,0,0,0,0 };
		imp_->m_->getOutputPos(ee_position);
		ee_position[3] = std::fmod(ee_position[3], 2 * aris::PI);

		double ee_velocity[6] = { 0,0,0,0,0,0 };
		dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool()[0]).getMva(ee_velocity);

		data[6] = ee_position[0];
		data[7] = ee_position[1];
		data[8] = ee_position[3];

		data[9] = ee_velocity[0];
		data[10] = ee_velocity[1];
		data[11] = ee_velocity[5];
	}

	void Controller::init(Model* model) {
		imp_->m_.reset(model);
	}

	Controller::Controller(const Controller& other) : imp_(new Imp(*other.imp_)) {
		lastTorque = other.lastTorque;
		lastrealAcc = other.lastrealAcc;
	}

	Controller::Controller(int outputSize, int intputSize) : imp_(new Imp(this)) {
		imp_->torque.resize(outputSize);
		lastTorque.resize(outputSize);
		imp_->stateVar.resize(12);
		imp_->desiredAcc.resize(4);
	}

	//  Constructor Function
	Controller::Controller(Model* model, int outputSize, int intputSize) : imp_(new Imp(this)) {
		imp_->torque.resize(outputSize);
		lastTorque.resize(outputSize);
		imp_->m_.reset(model);		
		imp_->stateVar.resize(12);
		imp_->desiredAcc.resize(4);
	}
	Controller::~Controller() = default;

}



