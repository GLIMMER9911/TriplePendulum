
#include "control.hpp"

#include "../utils/utils.hpp"
#include "../solver/math.hpp"
#include "../solver/qpsolver.hpp"

namespace triple {

	// calculate Controller implementation
	struct Controller::Imp {
		int count_ = 0;
		Controller* controller_;
		std::shared_ptr<aris::dynamic::Model> m_;
		double com_acc = 0;
		double cm_pos[3]{ 0.0, 0.0, 0.0 };
		double cm_vel[3]{ 0.0, 0.0, 0.0 };
		double cm_acc[3]{ 0.0, 0.0, 0.0 };
		double moment[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double lambda[5]{ 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::vector<double> stateVar;
		std::vector<double> desiredAcc;
		std::vector<double> calcdesiredAcc;
		std::vector<double> torque;
		Eigen::MatrixXd A;
		Eigen::MatrixXd B;
		Eigen::MatrixXd COMJacobian;
		Eigen::VectorXd COMCf;

		Eigen::MatrixXd A_7x2;
		Eigen::MatrixXd B_7x1;
		std::vector<double> desired7Acc;



		Imp(Controller* controller) : controller_(controller) {
			A = Eigen::MatrixXd::Zero(3, 2);
			B = Eigen::MatrixXd::Zero(3, 1);
			A_7x2 = Eigen::MatrixXd::Zero(7, 2);
			B_7x1 = Eigen::MatrixXd::Zero(7, 1);
			desired7Acc.resize(7);
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
	void Controller::setQPparameters(std::vector<double>& lambda_data) {
		for (int i = 0; i < lambda_data.size(); i++) {
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
	}

	void Controller::calculateCoMJacobian() {
		auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(imp_->m_->generalMotionPool().at(0));

		auto& forward_kinematic_solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&> (imp_->m_->solverPool().at(1));

		////////////////////////////////////////////////////////////////// COM Accelerate test ///////////////////////////////////////////////////////////////////////////
		// ��ʼ����x����ļ��ٶȣ�
		cptModelCm();
		double cx_acc = imp_->cm_acc[0];
		imp_->com_acc = imp_->cm_acc[0];

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
		double test0[3][6]{ {0, 0, 0, 0, 0, 0},
							{0, 0, 0, 0, 0, 0}, 
							{0, 0, 0, 0, 0, 0} };

		double test1[3][6]{ {0, 0, 0, 0, 0, 1},
							{0, 0, 0, 0, 0, 0},
							{0, 0, 0, 0, 0, 0} };

		double test2[3][6]{ {0, 0, 0, 0, 0, 0},
							{0, 0, 0, 0, 0, 1},
							{0, 0, 0, 0, 0, 0} };

		double test3[3][6]{ {0, 0, 0, 0, 0, 0},
							{0, 0, 0, 0, 0, 0},
							{0, 0, 0, 0, 0, 1} };

		double ee_Maa[6]{ 0, 0, 0, 0, 0, 0 };
		
		// set motion acceleration 
		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), test0[i]);
		}
		cptModelCm();
		ee.updA();
		ee.getMaa(ee_Maa);
		Cf << 0, ee_Maa[0], ee_Maa[1];
		CoMCf(0, 0) = imp_->cm_acc[0];


		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), test1[i]);
		}
		cptModelCm();
		ee.updA();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 0, 3, 1) << 1, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 0) = (imp_->cm_acc[0] - CoMCf[0]);

		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), test2[i]);
		}
		cptModelCm();
		ee.updA();
		ee.getMaa(ee_Maa);
		Jacobian.block(0, 1, 3, 1) << 0, (ee_Maa[0] - Cf(1, 0)), (ee_Maa[1] - Cf(2, 0));
		CoMJacobian(0, 1) = (imp_->cm_acc[0] - CoMCf[0]);
		//std::cout << std::endl;
		//std::cout << " Jacobian1: " << Jacobian << std::endl;


		for (int i = 0; i < imp_->m_->jointPool().size(); ++i) {
			imp_->m_->jointPool().at(i).makI()->fatherPart().setAs(*imp_->m_->jointPool().at(i).makJ(), test3[i]);
		}
		cptModelCm();
		ee.updA();
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
		vx_d = std::max(vx_d, -1.0);
		vx_d = std::min(vx_d, 1.0);
		double ax_d = kp_vcx * (vx_d - imp_->cm_vel[0]);
		ax_d = std::max(ax_d, -6.0);
		ax_d = std::min(ax_d, 6.0);
		imp_->desiredAcc[2] = ax_d + 0 * (imp_->COMJacobian(0, 1) * imp_->desiredAcc[0] );

		static std::ofstream outputFile("AngularMomentandCoMstates.txt");
		if (outputFile.is_open()) {
			outputFile << imp_->moment[5] << "\t " << imp_->cm_pos[0] << "\t " << imp_->cm_vel[0] << "\t " << imp_->com_acc << std::endl;
		}
		//outputFile.close();
		 
		
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
		double aj[3]{0.0};
		double as[6];
		imp_->m_->jointPool()[0].makI()->getAs(*imp_->m_->jointPool()[0].makJ(), as);
		aj[0] = as[5];

		{
			int i = 1;
			for (auto& m : imp_->m_->motionPool()) {
				m.updA();
				aj[i] = m.ma();
				i++;
			}
		}

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

		// calculate A_7x2 and B_7x1
		if (imp_->count_ < 5000) {
			if ((force[0] == 0) && (force[1] == 0)) {
				// get b
				imp_->B_7x1(0, 0) = ap[0];
				imp_->B_7x1(1, 0) = ap[1];
				imp_->B_7x1(2, 0) = imp_->cm_acc[0];
				imp_->B_7x1(3, 0) = imp_->cm_acc[1];
				imp_->B_7x1(4, 0) = aj[0];
				imp_->B_7x1(5, 0) = aj[1];
				imp_->B_7x1(6, 0) = aj[2];
			}
			else if ((force[0] == 1) && (force[1] == 0)) {
				imp_->A_7x2(0, 0) = ap[0] - imp_->B_7x1(0, 0);
				imp_->A_7x2(1, 0) = ap[1] - imp_->B_7x1(1, 0);
				imp_->A_7x2(2, 0) = imp_->cm_acc[0] - imp_->B_7x1(2, 0);
				imp_->A_7x2(3, 0) = imp_->cm_acc[1] - imp_->B_7x1(3, 0);
				imp_->A_7x2(4, 0) = aj[0] - imp_->B_7x1(4, 0);
				imp_->A_7x2(5, 0) = aj[1] - imp_->B_7x1(5, 0);
				imp_->A_7x2(6, 0) = aj[2] - imp_->B_7x1(6, 0);
			}
			else if ((force[0] == 0) && (force[1] == 1)) {
				imp_->A_7x2(0, 1) = ap[0] - imp_->B_7x1(0, 0);
				imp_->A_7x2(1, 1) = ap[1] - imp_->B_7x1(1, 0);
				imp_->A_7x2(2, 1) = imp_->cm_acc[0] - imp_->B_7x1(2, 0);
				imp_->A_7x2(3, 1) = imp_->cm_acc[1] - imp_->B_7x1(3, 0);
				imp_->A_7x2(4, 1) = aj[0] - imp_->B_7x1(4, 0);
				imp_->A_7x2(5, 1) = aj[1] - imp_->B_7x1(5, 0);
				imp_->A_7x2(6, 1) = aj[2] - imp_->B_7x1(6, 0);
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
				imp_->B_7x1(0, 0) = ap[0];
				imp_->B_7x1(1, 0) = ap[1];
				imp_->B_7x1(2, 0) = imp_->cm_acc[0];
				imp_->B_7x1(3, 0) = imp_->cm_acc[1];
				imp_->B_7x1(4, 0) = aj[0];
				imp_->B_7x1(5, 0) = aj[1];
				imp_->B_7x1(6, 0) = aj[2];
			}
			else if ((force[0] == 1) && (force[1] == 0)) {
				imp_->A_7x2(0, 0) = ap[0] - imp_->B_7x1(0, 0);
				imp_->A_7x2(1, 0) = ap[1] - imp_->B_7x1(1, 0);
				imp_->A_7x2(2, 0) = imp_->cm_acc[0] - imp_->B_7x1(2, 0);
				imp_->A_7x2(3, 0) = imp_->cm_acc[1] - imp_->B_7x1(3, 0);
				imp_->A_7x2(4, 0) = aj[0] - imp_->B_7x1(4, 0);
				imp_->A_7x2(5, 0) = aj[1] - imp_->B_7x1(5, 0);
				imp_->A_7x2(6, 0) = aj[2] - imp_->B_7x1(6, 0);

			}
			else if ((force[0] == 0) && (force[1] == 1)) {
				imp_->A_7x2(0, 1) = ap[0] - imp_->B_7x1(0, 0);
				imp_->A_7x2(1, 1) = ap[1] - imp_->B_7x1(1, 0);
				imp_->A_7x2(2, 1) = imp_->cm_acc[0] - imp_->B_7x1(2, 0);
				imp_->A_7x2(3, 1) = imp_->cm_acc[1] - imp_->B_7x1(3, 0);
				imp_->A_7x2(4, 1) = aj[0] - imp_->B_7x1(4, 0);
				imp_->A_7x2(5, 1) = aj[1] - imp_->B_7x1(5, 0);
				imp_->A_7x2(6, 1) = aj[2] - imp_->B_7x1(6, 0);

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

		Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(m, 1);
		Eigen::MatrixXd torq = Eigen::MatrixXd::Zero(n, 1);

		Eigen::MatrixXd allAcc = Eigen::MatrixXd::Zero(7, 1);


		torq(0, 0) = lastTorque[0];
		torq(1, 0) = lastTorque[1];
		acc = imp_->A * torq + imp_->B;
		allAcc = imp_->A_7x2 * torq + imp_->B_7x1;

		for (int i = 0; i < 7; ++i) {
			lastrealAcc[i] = allAcc(i, 0);
		}

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

		///////////////////////// QP ////////////////////////////

		///////////////////// Set NEXT desired Acc //////////////////////
		Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(ipt_size, 1);
		Eigen::MatrixXd error(3, 1);
		double comAcc_error = 0;
		double abs_comAcc = std::abs(imp_->desiredAcc[2]);

		// CoM
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
		const int n = opt_size;
		const int m = ipt_size;

		// P
		const int size_P = m + n;
		Eigen::MatrixXd P(size_P, size_P);
		P.setZero();
		P.diagonal() << imp_->lambda[0], imp_->lambda[1], imp_->lambda[2], imp_->lambda[3], imp_->lambda[4];

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
		l.block(m, 0, n, 1) << -15, -15;

		// u
		Eigen::MatrixXd u = Eigen::MatrixXd::Zero(n + m, 1);
		u.block(0, 0, m, 1) = (Acc - imp_->B + error);
		u.block(m, 0, n, 1) << 15, 15;

		// x 
		Eigen::MatrixXd x = Eigen::MatrixXd::Zero(n + m, 1);
		x = qpSolver(P, q, QP_A, l, u);

		Eigen::MatrixXd torque = x.block(0, 0, n, 1);

		//std::cout << "qp solver: " << x << std::endl;

		auto max_fce = 1000.0;
		imp_->torque[0] = std::min(torque(0, 0), max_fce);
		imp_->torque[0] = std::max(torque(0, 0), -max_fce);
		imp_->torque[1] = std::min(torque(1, 0), max_fce);
		imp_->torque[1] = std::max(torque(1, 0), -max_fce);

		Eigen::MatrixXd accAfter = imp_->A * torque + imp_->B;
		imp_->calcdesiredAcc[0] = accAfter(0, 0);
		imp_->calcdesiredAcc[1] = accAfter(1, 0);
		imp_->calcdesiredAcc[2] = accAfter(2, 0);



	}

	// 1. calculate G and g
	// (xdd-xddt_ref)^2 ||[a11 a12]*[torq1 torq2]^{T}+(b-xddt_ref)||2;
	//					= [torq1 torq2] * A * [torq1 torq2]^T + 2*(b-xddt_ref)[a11 a12]*[torq1 torq2]^T
	// (ydd-yddt_ref)^2, (cxdd-cxddt_ref)^2
	// 2. nG, nCE, nCI, aris_G, aris_g, aris_CE, aris_ce, aris_CI, aris_ci
	// 3. aris::dynamic::s_quadprog
	void Controller::arisCalcuTorque() {
		Eigen::Matrix2d square;
		square.setZero();
		for (int i = 0; i < 3; ++i) {
			square += imp_->A.block(i, 0, 1, 2).transpose() * imp_->lambda[i + 2] * imp_->A.block(i, 0, 1, 2);
		}

		//Eigen::Matrix2d square_A1 = imp_->A.block(0, 0, 1, 2).transpose() * imp_->lambda[2] * imp_->A.block(0, 0, 1, 2);
		//Eigen::Matrix2d square_A2 = imp_->A.block(1, 0, 1, 2).transpose() * imp_->lambda[3] * imp_->A.block(1, 0, 1, 2);
		//Eigen::Matrix2d square_A3 = imp_->A.block(2, 0, 1, 2).transpose() * imp_->lambda[4] * imp_->A.block(2, 0, 1, 2);

		Eigen::MatrixXd g1 = imp_->lambda[2] * (imp_->B(0, 0) - imp_->desiredAcc[0]) * imp_->A.block(0, 0, 1, 2);
		Eigen::MatrixXd g2 = imp_->lambda[3] * (imp_->B(1, 0) - imp_->desiredAcc[1]) * imp_->A.block(1, 0, 1, 2);
		Eigen::MatrixXd g3 = imp_->lambda[4] * (imp_->B(2, 0) - imp_->desiredAcc[2]) * imp_->A.block(2, 0, 1, 2);

		Eigen::Matrix2d G = Eigen::Matrix2d::Identity();
		G.diagonal() << imp_->lambda[0], imp_->lambda[1];
		G = G + square;

		Eigen::MatrixXd g = g1 + g2 + g3;

		const aris::Size nG = 2, nCE = 0, nCI = 4;

		double aris_G[nG * nG]{ G(0,0), G(0,1),
								G(1,0), G(1,1),
		};

		double aris_g[nG]{ g(0,0), g(0,1)};
		std::vector<double> aris_CE( nCE * nG);
		std::vector<double> aris_ce( nCE);

		double aris_CI[nCI * nG]{ 1, 0,
								  0, 1,
								  -1, 0,
								  0, -1,
		};

		double aris_ci[nCI]{ 100, 100, 100, 100 };

		std::vector<double> result(nG), mem(nG * nG * 2 + 8 * (nCE + nCI) + 3 * nG);
		auto r = aris::dynamic::s_quadprog(nG, nCE, nCI, aris_G, aris_g, aris_CE.data(), aris_ce.data(), aris_CI, aris_ci, result.data(), mem.data());
		


		//std::cout << "G :\n" << G << std::endl;
		//std::cout << "g: \n " << g << std::endl;

		//std::cout << "aris: " << result[0] << " " << result[1] << std::endl;
		//std::cout << "osqp: " << imp_->torque[0] << " " << imp_->torque[1] << std::endl;
	}

	void Controller::dspComputingInformation(int period) {

		if ((imp_->count_ - 1) % period == 0) {
			std::cout << "imp_->count_ " << imp_->count_ << " ---------------------------------------------------- " << std::endl;
			std::cout << " " << std::endl;
			std::cout << "imp_->A: \n" << imp_->A << std::endl;
			std::cout << "imp_->B : \n" << imp_->B << std::endl;

			std::cout << "Acc: " << imp_->desiredAcc[0] << " " << imp_->desiredAcc[1] << " " << imp_->desiredAcc[2] << std::endl;
			std::cout << "torque: " << imp_->torque[0] << "  " << imp_->torque[1] << " " << std::endl;
			std::cout << "CoM x and y com: " << imp_->cm_pos[0] << " " << imp_->cm_pos[1] << " " << std::endl;
			std::cout << "imp_->A_7x2 : \n" << imp_->A_7x2 << std::endl;
			std::cout << "imp_->B_7x1:  \n" << imp_->B_7x1 << std::endl;
		}
	}

	void Controller::cptdesired7Acc() {
		imp_->desired7Acc[0] = imp_->desiredAcc[0];
		imp_->desired7Acc[1] = imp_->desiredAcc[1];
		imp_->desired7Acc[2] = imp_->desiredAcc[2];

		// Map the Angle to [-pi. pi]
		double joint_pos[3]{0.0, 0.0, 0.0};
		double joint_vel[3]{0.0, 0.0, 0.0};
		for (int i = 0; i < 3; ++i) {
			joint_vel[i] = imp_->stateVar[3 + i];
			joint_pos[i] = std::fmod(imp_->stateVar[i], 2 * aris::PI);
			if (joint_pos[i] > aris::PI) {
				joint_pos[i] = joint_pos[i] - 2 * aris::PI;
			}
		}


		////  CoM acceleration in y direction
		static double cy_d = 0.42;
		double kp_cy = 10;
		double kp_vcy = 50;
		double vy_d = kp_cy * (cy_d - imp_->cm_pos[1]);
		vy_d = std::max(vy_d, -1.0);
		vy_d = std::min(vy_d, 1.0);
		double ay_d = kp_vcy * (vy_d - imp_->cm_vel[1]);
		ay_d = std::max(ay_d, -10.0);
		ay_d = std::min(ay_d, 10.0);
		imp_->desired7Acc[3] = ay_d ;

		//  Joint 1 acceleration, joint
		static double j1pos_d = 0;
		double kp_j1 = 10;
		double kd_j1 = 5;
		double j1vel_d = kp_j1 * (j1pos_d - joint_pos[0]);
		j1vel_d = std::max(j1vel_d, -10.0);
		j1vel_d = std::min(j1vel_d, 10.0);
		double j1acc_d = kd_j1 * (j1vel_d - joint_vel[0]);
		j1acc_d = std::max(j1acc_d, -1000.0);
		j1acc_d = std::min(j1acc_d, 1000.0);
		imp_->desired7Acc[4] = j1acc_d;

		static double j2pos_d = 0;
		double kp_j2 = 10;
		double kd_j2 = 5;
		double j2vel_d = kp_j2 * (j2pos_d - joint_pos[1]);
		j2vel_d = std::max(j2vel_d, -10.0);
		j2vel_d = std::min(j2vel_d, 10.0);
		double j2acc_d = kd_j2 * (j2vel_d - joint_vel[1]);
		j2acc_d = std::max(j2acc_d, -1000.0);
		j2acc_d = std::min(j2acc_d, 1000.0);
		imp_->desired7Acc[5] = j2acc_d;

		static double j3pos_d = 0;
		double kp_j3 = 10;
		double kd_j3 = 5;
		double j3vel_d = kp_j3 * (j3pos_d - joint_pos[2]);
		j3vel_d = std::max(j3vel_d, -10.0);
		j3vel_d = std::min(j3vel_d, 10.0);
		double j3acc_d = kd_j3 * (j3vel_d - joint_vel[2]);
		j3acc_d = std::max(j3acc_d, -1000.0);
		j3acc_d = std::min(j3acc_d, 1000.0);
		imp_->desired7Acc[6] = j3acc_d;

	}

	void Controller::osqpMatrix7x2test() {
		// P
		const int m = 7, n = 2;
		const int size_P = m + n;
		
		Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(m, 1);

		for (int i = 0; i < m; i++) {
			Acc(i, 0) = imp_->desired7Acc[i];
		}

		// torque1, torque2, ddx, ddy, ddcx, ddcy, j1a, j2a, ja3
		double lambda_9[9]{ 1e-6, 1e-6, 1e-3, 1e-3, 1000, 0, 0, 0, 0 };

		Eigen::MatrixXd P(size_P, size_P);
		P.setZero();
		P.diagonal() << lambda_9[0], lambda_9[1], lambda_9[2], lambda_9[3], lambda_9[4],
						lambda_9[5], lambda_9[6], lambda_9[7], lambda_9[8];

		// q
		Eigen::MatrixXd q(n + m, 1);
		q.setZero();

		// A 
		Eigen::MatrixXd Ad = imp_->A_7x2;
		Eigen::MatrixXd QP_A(m + n, m + n);
		QP_A.setZero();
		QP_A.block(0, 0, m, n) = Ad;
		QP_A.block(0, n, m, m) = (-1) * Eigen::MatrixXd(m, m).setIdentity();
		QP_A.block(m, 0, n, n) = Eigen::MatrixXd(n, n).setIdentity();

		// l
		Eigen::MatrixXd l = Eigen::MatrixXd::Zero(n + m, 1);
		l.block(0, 0, m, 1) = (Acc - imp_->B_7x1);
		l.block(m, 0, n, 1) << -100, -100;

		// u
		Eigen::MatrixXd u = Eigen::MatrixXd::Zero(n + m, 1);
		u.block(0, 0, m, 1) = (Acc - imp_->B_7x1);
		u.block(m, 0, n, 1) << 100, 100;

		// x 
		Eigen::MatrixXd x = Eigen::MatrixXd::Zero(n + m, 1);
		x = qpSolver(P, q, QP_A, l, u, n, m);

		Eigen::MatrixXd torque = x.block(0, 0, n, 1);

		//std::cout << "qp solver: " << x << std::endl;
		imp_->torque[0] = torque(0, 0);
		imp_->torque[1] = torque(1, 0);
	}

	void Controller::arisMatrix7x2test() {

		// Map the Angle to [-pi. pi]
		double joint_pos[3]{ 0.0, 0.0, 0.0 };
		double joint_vel[3]{ 0.0, 0.0, 0.0 };
		for (int i = 0; i < 3; ++i) {
			joint_vel[i] = imp_->stateVar[3 + i];
			joint_pos[i] = std::fmod(imp_->stateVar[i], 2 * aris::PI);
			if (joint_pos[i] > aris::PI) {
				joint_pos[i] = joint_pos[i] - 2 * aris::PI;
			}
		}

		// joint velcity and position constraint 
		// the actual position constraint [-2.0, 2.0] and the actual vel constraint is [-9, -9] 
		double max_joint_pos[2]{1.9, 1.9}; 
		double min_joint_pos[2]{-1.9, -1.9};
		double max_joint_vel[2]{10, 10};
		double min_joint_vel[2]{-10, 10};

		// calculate inequality
		double max_joint_acc[2]{0.0, 0.0};
		double min_joint_acc[2]{0.0, 0.0};
		max_joint_acc[0] = (max_joint_pos[0] - joint_pos[1]) * 1e6 - max_joint_vel[0] * 1e3;
		max_joint_acc[1] = (max_joint_pos[1] - joint_pos[2]) * 1e6 - max_joint_vel[1] * 1e3;

		min_joint_acc[0] = (min_joint_pos[0] - joint_pos[1]) * 1e6 - min_joint_vel[0] * 1e3;
		min_joint_acc[1] = (min_joint_pos[1] - joint_pos[2]) * 1e6 - min_joint_vel[1] * 1e3;

		double joint_CI[4] = { imp_->A_7x2(5, 0), imp_->A_7x2(5, 1),
								imp_->A_7x2(6, 0), imp_->A_7x2(6, 1), };
		double joint_ci[4] = { (max_joint_acc[0] - imp_->B_7x1(5,0)),
								(max_joint_acc[1] - imp_->B_7x1(6,0)),
								(-min_joint_acc[0] + imp_->B_7x1(5,0)),
								(-min_joint_acc[1] + imp_->B_7x1(6,0)) };

		//
		Eigen::Matrix2d square = Eigen::Matrix2d::Zero();
		Eigen::MatrixXd g(1, 2);
		g.setZero();

		// torque1, torque2, ddx, ddy, ddcx, ddcy, j1a, j2a, ja3
		double lambda_9[9]{ 1e-6, 1e-6, 1e-3, 1e-3, 1000, 0.0, 0.0, 0.0, 0.0 };

		for (int i = 0; i < 7; ++i) {
			square += imp_->A_7x2.block(i, 0, 1, 2).transpose() * lambda_9[i + 2] * imp_->A_7x2.block(i, 0, 1, 2);
		}

		for (int i = 0; i < 7; ++i) {
			g += lambda_9[i + 2] * (imp_->B_7x1(i, 0) - imp_->desired7Acc[i]) * imp_->A_7x2.block(i, 0, 1, 2);
		}

		Eigen::Matrix2d G = Eigen::Matrix2d::Identity();
		G.diagonal() << lambda_9[0], lambda_9[1];
		G = G + square;

		const aris::Size nG = 2, nCE = 0, nCI = 8;

		double aris_G[nG * nG]{ G(0,0), G(0,1),
								G(1,0), G(1,1),
		};

		double aris_g[nG]{ g(0, 0), g(0, 1) };
		std::vector<double> aris_CE(nCE * nG);
		std::vector<double> aris_ce(nCE);

		double aris_CI[nCI * nG]{ 1, 0,
								  0, 1,
								  -1, 0,
								  0, -1,
								  joint_CI[0], joint_CI[1],
								  joint_CI[2], joint_CI[3],
								  -joint_CI[0], -joint_CI[1],
								  -joint_CI[2], -joint_CI[3],
		};

		double aris_ci[nCI]{ 10, 10, 10, 10, joint_ci[0], joint_ci[1], joint_ci[2], joint_ci[3]};

		std::vector<double> result(nG), mem(nG * nG * 2 + 8 * (nCE + nCI) + 3 * nG);
		auto r = aris::dynamic::s_quadprog(nG, nCE, nCI, aris_G, aris_g, aris_CE.data(), aris_ce.data(), aris_CI, aris_ci, result.data(), mem.data());

		imp_->torque[0] = result[0];
		imp_->torque[1] = result[1];

 		//std::cout << "aris: " << result[0] << " " << result[1] << std::endl;
		//std::cout << "osqp: " << imp_->torque[0] << " " << imp_->torque[1] << std::endl;
	}

	// send torque
	auto Controller::sendTorque()->std::vector<double> {

		this->estimateState();
		this->cptModelCm();
		this->cptModelAngularMoment();

		this->calculateCoMJacobian();
		this->cptdesiredCoMAcc();
		this->cptdesired7Acc();

		this->calculateAandB();

		// Because the communication has ONE STEP ERROR, we need to check  whether the calculate Acc is as same as the real Acc. 
		this->checkrealAcc(3, 2);
		this->calculateTorque();

		this->arisCalcuTorque();

		//this->osqpMatrix7x2test();
		this->arisMatrix7x2test();

		this->dspComputingInformation(100);

		imp_->count_++;
		lastTorque = imp_->torque;

		return imp_->torque;
	}

	// send joint and end-effector accelerations 
	void Controller::sendDesiredAcc(std::vector<double>& desireddata) {

		std::vector<double> joint_acc;

		auto& force1 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(0));
		auto& force2 = dynamic_cast<aris::dynamic::SingleComponentForce&>(imp_->m_->forcePool().at(1));
		auto& forward_dynamic_solver = dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(imp_->m_->solverPool().at(3));

		force1.setFce(imp_->torque[0]);
		force2.setFce(imp_->torque[1]);
		forward_dynamic_solver.dynAccAndFce();

		for (auto& m : imp_->m_->motionPool()) m.updA();
		// 电机端, 每个关节的加速度；
		double as[6];
		imp_->m_->jointPool()[0].makI()->fatherPart().getAs(*imp_->m_->jointPool()[0].makJ(), as);
		joint_acc.push_back(as[5]);

		for (auto &m: imp_->m_->motionPool()) {
			joint_acc.push_back(m.ma());
		}
		joint_acc.push_back(imp_->calcdesiredAcc[0]);
		
		//std::cout << "joint acc " << std::endl;
		//for (auto a : joint_acc) {
		//	std::cout << a << " ";
		//}
		//std::cout << std::endl;

		desireddata = joint_acc;
	}

	// verify the model accelerate 
	// data : joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
	void Controller::verifyAccelerate(std::vector<double>& data) {

		double error = 0;
		if (imp_->count_ > 1) {
			for (int i = 0; i < lastrealAcc.size() - 1; ++i) {
				error = std::max(std::abs(data[12 + i] - lastrealAcc[i]), error);
			}

			if (error > 1e-1 * 1.0)
			{
				std::cout << "acc desired:";
				for (int i = 0; i < lastrealAcc.size(); i++) {
					std::cout << lastrealAcc[i] << " ";
				}
				std::cout << std::endl;

				std::cout << "data back: " << std::endl;
				std::cout << data[12] << " " << data[13] << " " << data[15] << " "
					<< data[16] << " " << data[17] << " " << std::endl;
				std::cout << "---------------------------- " << std::endl;
				throw std::runtime_error("error : acc data not correct");
			}
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
		imp_->calcdesiredAcc.resize(3);
	}

	//  Constructor Function
	Controller::Controller(Model* model, int outputSize, int intputSize) : imp_(new Imp(this)) {
		imp_->torque.resize(outputSize);
		lastTorque.resize(outputSize);
		imp_->m_.reset(model);		
		imp_->stateVar.resize(12);
		imp_->desiredAcc.resize(4);
		imp_->calcdesiredAcc.resize(3);

	}
	Controller::~Controller() = default;

}



