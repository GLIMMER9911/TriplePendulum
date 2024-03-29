///  建立三连杆模型
#include "acrobot.hpp"

const double PI = 3.141592653589793;

namespace acrobot{

	// calculate Acrobot implementation
	struct Acrobot::Imp {
		int count_ = 0;
		Acrobot* acrobot_;
		std::unique_ptr<aris::dynamic::Model> m_;
		double cm_pos[3]{ 0.0, 0.0, 0.0 };
		double cm_vel[3]{ 0.0, 0.0, 0.0 };
		double cm_acc[3]{ 0.0, 0.0, 0.0 };
		double moment[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		// torque1, ax, ay, aCoM
		double lambda[4]{ 0.0, 0.0, 0.0, 0.0};

		// q1, q2, w1, w2, ja1, ja2, x, y, angle, vx, vy, w, ax, ay, bz
		std::vector<double> stateVar;
		// torque1
		std::vector<double> torque;
		// torque1
		std::vector<double> lastTorque;
		// ax, ay, acm
		std::vector<double> desiredAcc;

		std::vector<double> lastrealAcc{0.0, 0.0, 0.0 };

		Eigen::MatrixXd A;
		Eigen::MatrixXd B;
		Eigen::MatrixXd COMJacobian;
		Eigen::VectorXd COMCf;


		Imp(Acrobot* acrobot) : acrobot_(acrobot) {
			A = Eigen::MatrixXd::Zero(3, 1);
			B = Eigen::MatrixXd::Zero(3, 1);
		}
	};

	// data: q1, q2, w1, w2, ja1, ja2, x, y, angle, vx, vy, w, ax, ay, bz
	void Acrobot::getStateVar(std::vector<double>& data, std::vector<double>& acc) {

		std::copy(data.begin(), data.end() - 1, imp_->stateVar.begin());

		std::copy(acc.begin(), acc.end(), imp_->desiredAcc.begin());
	}

	// set QP parameters 
	void Acrobot::setQPparameters(std::vector<double>& lambda_data) {
		for (int i = 0; i < lambda_data.size(); i++) {
			imp_->lambda[i] = lambda_data[i];
			if (imp_->lambda[i] < 1.0 * 1e-5) {
				imp_->lambda[i] = 1.0 * 1e-5;
			}
		}
	}

	// 进行状态估计 
	// stateVar: q1, q2, w1, w2, ja1, ja2, x, y, angle, vx, vy, w, ax, ay, bz
	void Acrobot::estimateState() {
		// 
		//double joint_pos[2][6] = { {0, 0, 0, 0, 0, imp_->stateVar[0]},
		//						   {0, 0, 0, 0, 0, imp_->stateVar[1]}
		//};
		//double joint_velocity[2][6] = { {0, 0, 0, 0, 0, imp_->stateVar[2]},
		//								{0, 0, 0, 0, 0, imp_->stateVar[3]}
		//};
		//double joint_accelerate[2][6] = { {0, 0, 0, 0, 0, imp_->stateVar[4]},
		//								  {0, 0, 0, 0, 0, imp_->stateVar[5]}
		//};

		//auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		//// calculate the forward position
		//for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
		//	imp_->m_->jointPool().at(i).makI()->setPe(*imp_->m_->jointPool().at(i).makJ(), joint_pos[i], "123");
		//}
		//for (auto& m : imp_->m_->motionPool()) m.updP();
		//ee.updP();

		//// calculate the forward velocity
		//for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
		//	imp_->m_->jointPool().at(i).makI()->fatherPart().setVs(*imp_->m_->jointPool().at(i).makJ(), joint_velocity[i]);
		//}
		//for (auto& m : imp_->m_->motionPool()) m.updV();
		//ee.updV();

		// calculate the forward accelerate
		//for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
		//	imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), joint_accelerate[i]);
		//}
		//for (auto& m : imp_->m_->motionPool()) m.updA();
		//ee.updA();

		this->cptModelCm();
		this->cptModelAngularMoment();
		this->calculateCoMJacobian();
		
	}

	// calculatue CoM Jacobian
	// cx_acc = COMJacobian * joint
	void Acrobot::calculateCoMJacobian() {
		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		auto& forward_kinematic_solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&> (imp_->m_->solverPool().at(1));

		////////////////////////////////////////////////////////////////// COM Accelerate test ///////////////////////////////////////////////////////////////////////////
		this->cptModelCm();
		double cx_acc = imp_->cm_acc[0];

		// [q1dd xdd ydd]^T =  Jacobian * qdd + Cf 
		Eigen::VectorXd Cf(3);
		Eigen::MatrixXd Jacobian(3, 2);
		Jacobian.setZero();

		// cx_acc = CoMJacobian(1, 2) * [ qdd1, qdd2] + b
		// CoMJacobian(1, 2) * computePeseudoInverse(Jacobian)(2, 3)
		// cx_acc = CoMJacobian(1, 3) * [ qdd1, xdd, ydd ] + CoMCf
		Eigen::MatrixXd CoMJacobian(1, 2);
		Eigen::VectorXd CoMCf(1);

		// ja1, ja2
		Eigen::VectorXd qdd(2);
		qdd << imp_->stateVar[4], imp_->stateVar[5];

		double test0[3]{ 0, 0, 0 };
		double test1[3]{ 1, 0, 0 };
		double test2[3]{ 0, 1, 0 };
		//double test3[3]{ 0, 0, 1 };
		double ee_Maa[6]{ 0, 0, 0, 0, 0, 0 };

		for (int i = 0; i < 2; ++i) {
			imp_->m_->motionPool().at(i).setMa(test0[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		this->cptModelCm();
		ee.getMaa(ee_Maa);
		Cf << 0, ee_Maa[0], ee_Maa[1];
		CoMCf(0, 0) = imp_->cm_acc[0];


		for (int i = 0; i < 2; ++i) {
			imp_->m_->motionPool().at(i).setMa(test1[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		this->cptModelCm();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 0, 3, 1) << 1, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 0) = (imp_->cm_acc[0] - CoMCf(0, 0));

		for (int i = 0; i < 2; ++i) {
			imp_->m_->motionPool().at(i).setMa(test2[i]);
		}
		forward_kinematic_solver.dynAccAndFce();
		cptModelCm();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 1, 3, 1) << 0, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 1) = (imp_->cm_acc[0] - CoMCf(0, 0));
		//std::cout << std::endl;
		//std::cout << " Jacobian1: " << Jacobian << std::endl;


		//for (int i = 0; i < 3; ++i) {
		//	imp_->m_->motionPool().at(i).setMa(test3[i]);
		//}
		//forward_kinematic_solver.dynAccAndFce();
		//cptModelCm();
		//ee.getMaa(ee_Maa);
		//Jacobian.block(0, 2, 3, 1) << 0, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		//CoMJacobian(0, 2) = (imp_->cm_acc[0] - CoMCf[0]);

		//Eigen::Vector<double, 3> answer;
		//answer = Jacobian * qdd + Cf;

		//Eigen::Vector<double, 1> com_answer;
		//com_answer = CoMJacobian * qdd + CoMCf;

		//Eigen::Vector<double, 1> com_answer_;
		
		Eigen::MatrixXd CoMJacobian_(1, 3);
		CoMJacobian_ = CoMJacobian * computePeseudoInverse(Jacobian);

		CoMCf = - CoMJacobian_ * Cf + CoMCf;
		//com_answer_ = CoMJacobian * answer + CoMCf;

		imp_->COMJacobian = CoMJacobian_;
		
		imp_->COMCf = CoMCf;
	}

	// 计算质心位置：
	// c  = (m1*c1 + m2*c2 + m3*c3)/(m1+m2+m3);
	// vc = (m1*vc1 + m2*vc2 + m3*vc3)/(m1+m2+m3);
	// ac = (m1*ac1 + m2*ac2 + m3*ac3)/(m1+m2+m3);
	void Acrobot::cptModelCm() {
		double im1[36], im2[36], im3[36];
		imp_->m_->partPool()[1].cptGlbIm(im1, 6);
		imp_->m_->partPool()[2].cptGlbIm(im2, 6);
		//imp_->m_->partPool()[3].cptGlbIm(im3, 6);
		double m1{ im1[0] }, m2{ im2[0] }, m3{ im3[0] };

		double x1[3]{ im1[11] / m1, im1[15] / m1 ,im1[4] / m1 };
		double x2[3]{ im2[11] / m2, im2[15] / m2 ,im2[4] / m2 };
		//double x3[3]{ im3[11] / m3, im3[15] / m3 ,im3[4] / m3 };

		double vx1[3], vx2[3], vx3[3], ax1[3], ax2[3], ax3[3];

		s_as2ap(imp_->m_->partPool()[1].vs(), imp_->m_->partPool()[1].as(), x1, ax1, vx1);
		s_as2ap(imp_->m_->partPool()[2].vs(), imp_->m_->partPool()[2].as(), x2, ax2, vx2);
		//s_as2ap(imp_->m_->partPool()[3].vs(), imp_->m_->partPool()[3].as(), x3, ax3, vx3);

		imp_->cm_pos[0] = (x1[0] * m1 + x2[0] * m2) / (m1 + m2);
		imp_->cm_pos[1] = (x1[1] * m1 + x2[1] * m2) / (m1 + m2);
		imp_->cm_pos[2] = (x1[2] * m1 + x2[2] * m2) / (m1 + m2);

		imp_->cm_vel[0] = (vx1[0] * m1 + vx2[0] * m2) / (m1 + m2);
		imp_->cm_vel[1] = (vx1[1] * m1 + vx2[1] * m2) / (m1 + m2);
		imp_->cm_vel[2] = (vx1[2] * m1 + vx2[2] * m2) / (m1 + m2);

		imp_->cm_acc[0] = (ax1[0] * m1 + ax2[0] * m2) / (m1 + m2);
		imp_->cm_acc[1] = (ax1[1] * m1 + ax2[1] * m2) / (m1 + m2);


	}

	// calculate the Model Angular Moment
	// Spatial Inertia
	// | 1 0 0    0        z   -y      | 
	// | 0 1 0   -z        0    x      |
	// | 0 0 1    y       -x    0      |
	// | 0 -z y  y^2+z^2 -xy   -xz     |
	// | z 0 -x  -xy   x^2+z^2 -yz     |
	// |-y x 0   -xz     -yz  x^2+y^2  |
	void Acrobot::cptModelAngularMoment() {
		double im1[36], im2[36], im3[36];
		imp_->m_->partPool()[1].cptGlbIm(im1, 6);
		imp_->m_->partPool()[2].cptGlbIm(im2, 6);
		//imp_->m_->partPool()[3].cptGlbIm(im3, 6);

		s_mm(6, 1, 6, im1, imp_->m_->partPool()[1].vs(), imp_->moment);
		// s_mma 
		s_mma(6, 1, 6, im2, imp_->m_->partPool()[2].vs(), imp_->moment);
		//s_mma(6, 1, 6, im3, imp_->m_->partPool()[3].vs(), imp_->moment);

		//std::cout << "imp_->moment[0]: " << imp_->moment[0] << " " << imp_->moment[1] << " "  << imp_->moment[5] << " " << std::endl;

	}

	// compute desired CoM accelerate
	void Acrobot::cptdesiredCoMAcc() {
		////////////////////////////////// 质心处的加速度 /////////////////////////////////////////////
		// 质心加速度 x 方向, 经过测试质心处的最大加速度为 6, 最好应该小于 5 ，系统才能控制得住。
		static double max_acc = 0;
		double cx_d = imp_->moment[5] * 0.1;
		double kp_cx = 1;
		double kp_vcx = 5;
		double vx_d = kp_cx * (cx_d - imp_->cm_pos[0]);
		vx_d = std::max(vx_d, -0.1);
		vx_d = std::min(vx_d, 0.1);
		double ax_d = kp_vcx * (vx_d - imp_->cm_vel[0]);
		ax_d = std::max(ax_d, -0.1);
		ax_d = std::min(ax_d, 0.1);
		imp_->desiredAcc[2] = ax_d + 0.0 * (imp_->COMJacobian(0, 1) * imp_->desiredAcc[0]);

		// std::cout << "cm_vel: " << imp_->cm_vel[0] << std::endl; 
		// std::cout << "ax_d: " << ax_d << std::endl; 
		// std::cout << "imp_->desiredAcc[2]: " << imp_->desiredAcc[2] << std::endl;
	}

	// calculate A and B column
	void Acrobot::calcAandBColumn(double* force) {

		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		auto& force1 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(0));
		//auto& force2 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(1));
		auto& forward_dynamic_solver = dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(imp_->m_->solverPool().at(3));

		force1.setFce(force[0]);
		//force2.setFce(force[1]);
		forward_dynamic_solver.dynAccAndFce();

		// end-effector position
		double pp[6]{ 0.0, 0.0, 0.0 }, vp[3]{ 0.0, 0.0, 0.0 }, ap[3]{ 0.0, 0.0, 0.0 };
		ee.getMpe(pp);
		aris::dynamic::s_as2ap(imp_->m_->partPool().back().vs(), imp_->m_->partPool().back().as(), pp, ap, vp);

		// calculate CoM acceleration
		this->cptModelCm();

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
			//else if ((force[0] == 0) && (force[1] == 1)) {
			//	imp_->A(0, 1) = ap[0] - imp_->B(0, 0);
			//	imp_->A(1, 1) = ap[1] - imp_->B(1, 0);
			//	imp_->A(2, 1) = imp_->cm_acc[0] - imp_->B(2, 0);
			//}
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
				imp_->B(2, 0) = imp_->cm_acc[0];
			}
			else if ((force[0] == 1) && (force[1] == 0)) {
				imp_->A(0, 0) = ap[0] - imp_->B(0, 0);
				imp_->A(1, 0) = ap[1] - imp_->B(1, 0);
				imp_->A(2, 0) = imp_->cm_acc[0] - imp_->B(2, 0);

			}
			//else if ((force[0] == 0) && (force[1] == 1)) {
			//	imp_->A(0, 1) = ap[0] - imp_->B(0, 0);
			//	imp_->A(1, 1) = ap[1] - imp_->B(1, 0);
			//	//imp_->A(2, 1) = imp_->cm_acc[0] - imp_->B(2, 0);
			//}
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
	void Acrobot::calculateAandB() {

		double force[2]{ 0.0, 0.0 };

		force[0] = 0.0;
		force[1] = 0.0;
		calcAandBColumn(force);

		force[0] = 1.0;
		force[1] = 0.0;
		calcAandBColumn(force);

		//force[0] = 0.0;
		//force[1] = 1.0;
		//calcAandBColumn(force);

	}

	// calculate torque
	void Acrobot::calculateTorque() {
		// size of input Accelerate
		int ipt_size = 3;
		// size of output torque
		int opt_size = 1;
		this->estimateState();
		this->cptdesiredCoMAcc();
		this->calculateAandB();

		// Because the communication has ONE STEP ERROR, we need to check  whether the calculate Acc is as same as the real Acc. 
		this->checkrealAcc(ipt_size, opt_size);

		///////////////////////////////// QP parameters Value  ///////////////////////////////////////////////

		///////////////////// Set NEXT desired Acc //////////////////////
		Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(ipt_size, 1);
		Eigen::MatrixXd error(3, 1);
		double comAcc_error = 0;
		double abs_comAcc = std::abs(imp_->desiredAcc[2]);

		// 
		if ((abs_comAcc > 0) && (abs_comAcc < 0.2)) {
			comAcc_error = 0.2;
		}
		else if ((abs_comAcc > 0.2) && (abs_comAcc < 0.6)) {
			comAcc_error = 0.2 - 0.5 * (abs_comAcc - 0.2);
		}
		else {
			comAcc_error = 0.0;
		}

		error << 0.01, 0.01, 0.001;// comAcc_error;

		// desired Acc stateVar[6] stateVar[7] is calculated by PID, ax_d is the accelerate of CoM x positon.  
		Acc(0, 0) = imp_->desiredAcc[0];
		Acc(1, 0) = imp_->desiredAcc[1];
		Acc(2, 0) = imp_->desiredAcc[2];

		const int m = ipt_size;
		const int n = opt_size;

		//// P
		//const int size_P = m + n;
		//Eigen::MatrixXd P(size_P, size_P);
		//P.setZero();
		//P.diagonal() << imp_->lambda[0], imp_->lambda[1], imp_->lambda[2], imp_->lambda[3];

		//// q
		//Eigen::MatrixXd q(n + m, 1);
		//q.setZero();

		//// A 
		//Eigen::MatrixXd Ad = imp_->A;
		//Eigen::MatrixXd QP_A(m + n, m + n);
		//QP_A.setZero();
		//QP_A.block(0, 0, m, n) = Ad;
		//QP_A.block(0, n, m, m) = (-1) * Eigen::MatrixXd(m, m).setIdentity();
		//QP_A.block(m, 0, n, n) = Eigen::MatrixXd(n, n).setIdentity();

		//// l
		//Eigen::MatrixXd l = Eigen::MatrixXd::Zero(n + m, 1);
		//l.block(0, 0, m, 1) = (Acc - imp_->B);
		//l.block(m, 0, n, 1) << -100;

		//// u
		//Eigen::MatrixXd u = Eigen::MatrixXd::Zero(n + m, 1);
		//u.block(0, 0, m, 1) = (Acc - imp_->B + error);
		//u.block(m, 0, n, 1) << 100;

		//// x
		//Eigen::MatrixXd x = Eigen::MatrixXd::Zero(n + m, 1);
		//x = qpSolver(P, q, QP_A, l, u, 1, 3);

		//using aris::dynamic::s_quadprog

		const aris::Size nG = 4, nCE = 3, nCI = 2;



		double aris_G[nG * nG]{ imp_->lambda[0], 0, 0, 0, 
								0, imp_->lambda[1], 0, 0, 
								0, 0, imp_->lambda[2], 0, 
								0, 0, 0, imp_->lambda[3], 
		};

		double aris_g[nG]{ 0.0, 0.0, 0.0, 0.0 };
		double aris_CE[nCE * nG]{ imp_->A(0, 0), -1, 0.0, 0.0,
								  imp_->A(1, 0), 0.0, -1, 0.0,
								  imp_->A(2, 0), 0.0, 0.0, -1,
		};

		Eigen::MatrixXd ce = Acc - imp_->B;

		double aris_ce[nCE]{ -ce(0, 0), -ce(1, 0), -ce(2, 0)};

		double aris_CI[nCI * nG]{ 1, 0, 0, 0,
								  -1, 0, 0, 0,
		};

		double aris_ci[nCI]{10, 10};

		std::vector<double> result(nG), mem(nG * nG * 2 + 8 * (nCE + nCI) + 3 * nG);
		auto r = aris::dynamic::s_quadprog(nG, nCE, nCI, aris_G, aris_g, aris_CE, aris_ce, aris_CI, aris_ci, result.data(), mem.data());

		double x = ce(2, 0) / imp_->A(2, 0);

		auto max_fce = 10.0;
		imp_->torque[0] = std::min(x, max_fce);
		imp_->torque[0] = std::max(x, -max_fce);
		//imp_->torque[1] = std::min(torque(1, 0), max_fce);
		//imp_->torque[1] = std::max(torque(1, 0), -max_fce);

		if (imp_->count_ % 500 == 0) {
			//std::cout << "A: \n" << QP_A << std::endl;
			//std::cout << "P: \n" << P << std::endl;
			//std::cout << "l: \n" << l << std::endl;
			//std::cout << "u: \n " << u << std::endl;
			std::cout << "imp_->count_ " << imp_->count_ << " ---------------------------------------------------- " << std::endl;
			std::cout << "imp_->lambda: " << imp_->lambda[0] << " " << imp_->lambda[1] << " " << imp_->lambda[2] << " " << imp_->lambda[3] << std::endl;
			std::cout << "imp_->A: \n" << imp_->A << std::endl;
			std::cout << "imp_->B: \n" << imp_->B << std::endl;
			std::cout << "Acc before: " << Acc(0, 0) << " " << Acc(1, 0) << " " << Acc(2, 0) << std::endl;
			std::cout << "torque: " << imp_->torque[0] << std::endl;
			std::cout << "CoM x com: " << imp_->cm_pos[0] << " " << imp_->cm_pos[1] << " " << std::endl;
			Eigen::MatrixXd accAfter = imp_->A * imp_->torque[0] + imp_->B;
			std::cout << "Acc After: " << accAfter(0, 0) << "  " << accAfter(1, 0) << "  "
					<< accAfter(2, 0) << std::endl;
			std::cout << "imp_->COMJacobian: " << imp_->COMJacobian << std::endl;
			std::cout << std::endl;
		}

		//std::cout << torque[0] << "  " << torque[1] << " " << std::endl;

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

		imp_->lastTorque = imp_->torque;
		this->test();

		//std::cout << "imp_->torque[0]: " << imp_->torque[0] << "imp_->torque[1]: " << imp_->torque[1] << std::endl;
	}

	void Acrobot::test() {
	/*using inverse dynamics to calculate output torque */
		auto& forward_kinematic_solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&> (imp_->m_->solverPool().at(1));
		auto& inverse_dynamic_solver = dynamic_cast<aris::dynamic::InverseDynamicSolver&> (imp_->m_->solverPool().at(2));
		
		int mJf = 0;
		int nJf = 0;
		forward_kinematic_solver.cptJacobi();
		mJf = forward_kinematic_solver.mJf();
		nJf = forward_kinematic_solver.nJf();

		auto Jf = forward_kinematic_solver.Jf();
		auto cf = forward_kinematic_solver.cf();
		
		//// [q1dd, ax, ay]^{T} = jacobian * [q1dd, q2dd] + [0, cf]
		// jacobian
		Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 2);
		jacobian(0, 0) = 1;
		jacobian(1, 0) = Jf[0];
		jacobian(1, 1) = Jf[1];
		jacobian(2, 0) = Jf[2];
		jacobian(2, 1) = Jf[3];

		// didq
		Eigen::VectorXd djdq = Eigen::VectorXd::Zero(3);
		djdq(1) = cf[0];
		djdq(2) = cf[1];

		int nM = 0;
		inverse_dynamic_solver.cptGeneralInverseDynamicMatrix();
		nM = inverse_dynamic_solver.nM();

		const double* arisM = inverse_dynamic_solver.M();
		const double* arish = inverse_dynamic_solver.h();

		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nM, nM);
		M(0, 0) = arisM[0];
		M(0, 1) = arisM[1];
		M(1, 0) = arisM[2];
		M(1, 1) = arisM[3];

		Eigen::VectorXd h = Eigen::VectorXd::Zero(nM);
		h(0) = h[0];
		h(1) = h[1];

		Eigen::MatrixXd invM = Eigen::MatrixXd::Zero(3, 3);
		invM.block(0, 0, 2, 3) = M * computePeseudoInverse(jacobian);
		invM.block(2, 0, 1, 3) = imp_->COMJacobian;

		Eigen::VectorXd invh = Eigen::VectorXd::Zero(3);
		invh.block(0, 0, 2, 1) = -invM.block(0, 0, 2, 3) * djdq + h;
		invh.block(2, 0, 1, 1) = imp_->COMCf;

		Eigen::MatrixXd eqM = Eigen::MatrixXd::Zero(3, 3);
		eqM.block(0, 0, 3, 2) = invM.block(0, 0, 3, 2);
		eqM(1, 2) = -1;

		Eigen::VectorXd eq(3);
		eq(0) = -invM(0, 2) * imp_->desiredAcc[1] - invh(0);
		eq(1) = -invM(1, 2) * imp_->desiredAcc[1] - invh(1);
		eq(2) = imp_->desiredAcc[2] - invM(2, 2) * imp_->desiredAcc[1] - invh(2);

		Eigen::MatrixXd answer = computePeseudoInverse(eqM) * eq;

		if (imp_->count_ % 500 == 0) {
			std::cout << "invM: " << std::endl;
			std::cout << invM << std::endl;
			std::cout << "invh: " << std::endl;
			std::cout << invh << std::endl;
			std::cout << "eqM: " << std::endl;
			std::cout << eqM << std::endl;
			std::cout << "eq: " << std::endl;
			std::cout << eq << std::endl;
			std::cout << "answer " << std::endl;
			std::cout << answer << std::endl;
		}

		imp_->torque[0] = answer(2, 0);
		imp_->lastTorque = imp_->torque;

	}


	// send torque
	auto Acrobot::sendTorque()->std::vector<double> {
		this->calculateTorque();
		std::vector<double> send_data {imp_->torque[0], 0 };
		// std::cout << "send_data: " << send_data[0] << " " << send_data[1] << " " << std::endl;
		imp_->count_++;

		return imp_->torque;
	}

	// Because the communication has ONE STEP ERROR, we need to check  whether the calculate Acc is as same as the real Acc. 
	void Acrobot::checkrealAcc(int m, int n) {

		Eigen::MatrixXd torq = Eigen::MatrixXd::Zero(n, 1);
		Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(m, 1);

		for (int i = 0; i < n; ++i) {
			torq(i, 0) = imp_->lastTorque[i];
		}

		acc = imp_->A * torq + imp_->B;

		for (int i = 0; i < m; ++i) {
			imp_->lastrealAcc[i] = acc(i, 0);
		}

	}


	// verify the model accelerate 
	// data : joint1, joint2, w1, w2, ja1, ja2, x, y, angle, vx, vy, wz, ax, ay, bz
	void Acrobot::verifyAccelerate(std::vector<double>& data) {

		double error = 0;
		static double lasttorque = 0;

		for (int i = 0; i < imp_->lastrealAcc.size() - 1; ++i) {
			error = std::max(std::abs(data[12 + i] - imp_->lastrealAcc[i]), error);
		}

		if (error > 1e-2 * 1.0)
		{
			std::cout << "imp_->count_: " << imp_->count_ << "  " << std::endl;
			std::cout << "torque: " << imp_->torque[0] << " " << std::endl;
			std::cout << "last torque: " << lasttorque << " " << std::endl;
			std::cout << "imp_->A: " << imp_->A << std::endl;
			std::cout << "imp_->B: " << imp_->B << std::endl;
			std::cout << "acc desired:";
			for (int i = 0; i < imp_->lastrealAcc.size(); i++) {
				std::cout << imp_->lastrealAcc[i] << " ";
			}
			std::cout << std::endl;

			std::cout << "data back: " << std::endl;
			std::cout << data[12] << " " << data[13] << " " << std::endl;
			std::cout << "---------------------------- " << std::endl;
			throw std::runtime_error("error : acc data not correct");
		}
		lasttorque = imp_->lastTorque[0];
	}

	// calculate Forward Kinematics
	// data: joint1, joint2, w1, w2, ja1, ja2,  x, y, angle, vx, vy, wz, ax, ay, bz
	void Acrobot::calcuForwardKinematics(std::vector<double>& data) {

		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		double Joint_pos[2][6] = { {0, 0, 0, 0, 0, data[0]},
								   {0, 0, 0, 0, 0, data[1]}	};

		double joint_velocity[2][6] = { {0, 0, 0, 0, 0, data[2]},
								   {0, 0, 0, 0, 0, data[3]} };

		double joint_accelerate[2][6] = { {0, 0, 0, 0, 0, data[4]},
								  {0, 0, 0, 0, 0, data[5]} };

		for (int i = 0; i < 2; ++i) {
			imp_->m_->jointPool().at(i).makI()->setPe(*imp_->m_->jointPool().at(i).makJ(), Joint_pos[i], "123");
		}
		for (auto& m : imp_->m_->motionPool()) m.updP();
		imp_->m_->generalMotionPool()[0].updP();

		// calculate the forward velocity
		for (int i = 0; i < 2; ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setVs(*imp_->m_->jointPool().at(i).makJ(), joint_velocity[i]);
		}
		for (auto& m : imp_->m_->motionPool()) m.updV();
		imp_->m_->generalMotionPool()[0].updV();

		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), joint_accelerate[i]);
		}
		for (auto& m : imp_->m_->motionPool()) m.updA();
		imp_->m_->generalMotionPool()[0].updA();

		double ee_position[6] = { 0,0,0,0,0,0 };
		imp_->m_->getOutputPos(ee_position);
		ee_position[3] = std::fmod(ee_position[3], 2 * aris::PI);

		double pp[6]{ 0.0, 0.0, 0.0 }, vp[3]{ 0.0, 0.0, 0.0 }, ap[3]{ 0.0, 0.0, 0.0 };
		ee.getMpe(pp);
		aris::dynamic::s_as2ap(imp_->m_->partPool().back().vs(), imp_->m_->partPool().back().as(), pp, ap, vp);

		
		double max_pos_error = std::abs(data[6] - ee_position[0]);
		max_pos_error = std::max(max_pos_error, std::abs(data[7] - ee_position[1]));

		double max_vel_error = std::abs(data[9] - vp[0]);
		max_vel_error = std::max(max_vel_error, std::abs(data[10] - vp[1]));

		double max_acc_error = std::abs(data[12] - ap[0]);
		max_acc_error = std::max(max_acc_error, std::abs(data[13] - ap[1]));
		
		if (max_acc_error > 1e-5 * 1.0 ) {
			std::cout << "-------------Forward Kinematics Error----------- " << std::endl;
			std::cout << " max_pos_error: " << max_pos_error << " max_vel_error: " << max_vel_error << " max_acc_error: " << max_acc_error << std::endl;
			std::cout << " data : " << data[6] << " " << data[7] << " " << data[9] << " " << data[10] << " " << data[12] << " " << data[13] << " " << std::endl;
			std::cout << " Ford : " << ee_position[0] << " " << ee_position[1] << " " << vp[0] << " " << vp[1] << " " << ap[0] << " " << ap[1] << " " << std::endl;
		}

		//data[6] = ee_position[0];
		//data[7] = ee_position[1];
		//data[8] = ee_position[3];

		//data[9] = ee_velocity[0];
		//data[10] = ee_velocity[1];
		//data[11] = ee_velocity[5];
	}

	void Acrobot::setModel(Model* model) {
		imp_->m_.reset(model);
	}

	Acrobot::Acrobot() : imp_(new Imp(this)) {
		imp_->torque.resize(1);
		imp_->lastTorque.resize(1);
		imp_->stateVar.resize(15);
		imp_->desiredAcc.resize(3);
	}
	Acrobot::~Acrobot() = default;

}


