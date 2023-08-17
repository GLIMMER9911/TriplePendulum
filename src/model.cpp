///  建立三连杆模型,
#include "model.hpp"

//-------------------------------------------------------------------------------------------------------//
//三个杆件，最后一根连杆上有一个body
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
// TripleModel mainly has two function:
//		receive data from Simulink; x_t x_t(dot) y_t y_t(dot) 
//		calculate torque and send it to Simulink;
//-------------------------------------------------------------------------------------------------------//

const double PI = 3.141592653589793;

template <typename T>
double* Convert2Arr(const T* data, int rows, int cols);

Eigen::MatrixXd  computePeseudoInverse(const Eigen::MatrixXd& jacobian);

// Save data to .txt file
void saveDataToFile(std::vector<double>& state, std::vector<double>& data, std::vector<double>& acc_) {

	static int iteration = 1;

	std::ofstream outputFile("data.txt", std::ios::app);

	int state_size = state.size();
	int data_size = data.size();
	if (outputFile.is_open()) {
		outputFile << iteration << " state variable: ";
		for (int i = 0; i < state_size; i++) {
			outputFile << state.at(i) << "  ";
		}

		outputFile << " torque  ";
		//std::cout << " torque  ";
		for (int i = 0; i < data_size; i++) {
			outputFile << data.at(i) << "  ";
			//std::cout << data.at(i) << "  ";
		}

		outputFile << " Acc_: ";
		for (int i = 0; i < acc_.size(); i++) {
			outputFile << acc_.at(i) << "  ";
			//std::cout << data.at(i) << "  ";
		}

		outputFile << " " << std::endl;
		outputFile.close();
	}
	else {
		std::cout << " The file can not open! " << std::endl;
	}
	iteration += 1;
}

//  Create robot model
auto TripleModel::CreateModel() -> std::unique_ptr< Model > {

	double a = 0.4;
	double b = 0.4;
	double c = 0.4;

	// 定义关节的位置，以及轴线，有3个转动副，轴线都是Z轴
	const double joint1_position[3]{ 0 , 0 , 0 };
	const double joint1_axis[3]{ 0 , 0 , 1 };
	const double joint2_position[3]{ 0 , a , 0 };
	const double joint2_axis[3]{ 0 , 0 , 1 };
	const double joint3_position[3]{ 0 , a + b , 0 };
	const double joint3_axis[3]{ 0 , 0 , 1 };

	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	const double link1_pos_euler[6]{ 0, a / 2, 0, PI / 2, 0, 0 };
	const double link1_intertia_vector[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	const double link2_pos_euler[6]{ 0, a + b / 2, 0, PI / 2, 0, 0 };
	const double link2_intertia_vecter[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	const double link3_pos_euler[6]{ 0,  a + b + c / 2, 0, PI / 2, 0, 0 };
	const double link3_intertia_vecter[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	const double body_intertia_vecter[10]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	// 定义末端位置与321欧拉角
	const double body_position_and_euler321[6]{ 0 , a + b + c , 0 , PI / 2 , 0 , 0 };

	// 定义模型
	std::unique_ptr < Model> model(new Model);

	// 设置重力,重力在y轴
	const double gravity[6]{ 0.0, -9.81, 0.0, 0.0, 0.0, 0.0 };
	model->environment().setGravity(gravity);

	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto& link1 = model->addPartByPe(link1_pos_euler, "321", link1_intertia_vector);
	auto& link2 = model->addPartByPe(link2_pos_euler, "321", link2_intertia_vecter);
	auto& link3 = model->addPartByPe(link3_pos_euler, "321", link3_intertia_vecter);

	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto& joint1 = model->addRevoluteJoint(link1, model->ground(), joint1_position, joint1_axis);
	auto& joint2 = model->addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto& joint3 = model->addRevoluteJoint(link3, link2, joint3_position, joint3_axis);

	// 添加驱动 Joint1 为被动关节，不用加motion
	auto& motion2 = model->addMotion(joint2);
	auto& motion3 = model->addMotion(joint3);

	// 添加末端，第一个参数表明末端位于link4上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
	auto& end_effector = model->addGeneralMotionByPe(link3, model->ground(), body_position_and_euler321, "321");

	auto& force2 = model->forcePool().add< aris::dynamic::SingleComponentForce >("f2", motion2.makI(), motion2.makJ(), 5);
	auto& force3 = model->forcePool().add< aris::dynamic::SingleComponentForce >("f3", motion3.makI(), motion3.makJ(), 5);

	//-------------------------------------------- 添加求解器 --------------------------------------------//
	/// [Solver]
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto& inverse_kinematic_solver = model->solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic_solver = model->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic_solver = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic_solver = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

	model->init();
	std::cout << "Successful modeling ! \n" << std::endl;
	//std::cout << aris::core::toXmlString( *model ) << std::endl;
	//aris::core::toXmlFile(*model,"C:\\Users\\dong\\Documents\\Robot\\TriplePendulum2\\config\\model.xml ");

	return model;
}

// get state variable x_t, y_t, theta, vx, vy, w, ax, ay, bz, q1_dd;
void TripleModel::getstate_var(std::vector<double>& data) {
	state_var = data;
}

// send calculate torque
std::vector<double> TripleModel::send_torque() {
	this->calcu_torque();
	return torque;
}

// get state variable: q1, q2, q3, w1, w2, w3, ax, ay, aj1, aj2, aj3;
void TripleModel::calcu_torque() {

	// 
	double joint_pos[3][6] = { {0, 0, 0, 0, 0, state_var[0]},
							   {0, 0, 0, 0, 0, state_var[1]},
							   {0, 0, 0, 0, 0, state_var[2]},
	};
	double joint_velocity[3][6] = { {0, 0, 0, 0, 0, state_var[3]},
									{0, 0, 0, 0, 0, state_var[4]},
									{0, 0, 0, 0, 0, state_var[5]}

	};

	double ee_accelerate[6]{ 0,0,0,0,0,0 };
	double ee_acc_s[6]{ 0,0,0,0,0,0 };

	// ee_accelerate = { ax, ay, 0, 0, 0, bz };
	ee_accelerate[0] = state_var[6];
	ee_accelerate[1] = state_var[7];


	double force[2]{ 0.0, 0.0 };

	auto& ee = dynamic_cast<aris::dynamic::GeneralMotion&>(m_->generalMotionPool().at(0));

	auto& inverse_dynamic_solver = dynamic_cast<aris::dynamic::InverseDynamicSolver&>(m_->solverPool().at(2));
	auto& forward_kinematic_solver = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m_->solverPool().at(1));
	auto& forward_dynamic_solver = dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(m_->solverPool().at(3));

	auto& force1 = dynamic_cast<aris::dynamic::SingleComponentForce&>(m_->forcePool().at(0));
	auto& force2 = dynamic_cast<aris::dynamic::SingleComponentForce&>(m_->forcePool().at(1));


	// calculate the forward position
	for (int i = 0; i < 3; i++) {
		m_->jointPool().at(i).makI()->setPe(*m_->jointPool().at(i).makJ(), joint_pos[i], "123");
	}
	for (auto& m : m_->motionPool()) m.updP();
	ee.updP();


	// calculate the forward velocity
	for (int i = 0; i < m_->jointPool().size(); i++) {
		m_->jointPool().at(i).makI()->fatherPart().setVs(*m_->jointPool().at(i).makJ(), joint_velocity[i]);
	}
	for (auto& m : m_->motionPool()) m.updV();
	ee.updV();

	// 1.  [ddx, ddy]^T = A * torque + b
	// 2.  t = A^(-1) * [ddx, ddy]^T -  A^(-1)*b
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(5, 2);

	force[0] = 0.0;
	force[1] = 0.0;
	force1.setFce(force[0]);
	force2.setFce(force[1]);
	forward_dynamic_solver.dynAccAndFce();
	
	double pp[6]{ 0.0, 0.0, 0.0 }, vp[3]{ 0.0, 0.0, 0.0 }, ap[3]{ 0.0, 0.0, 0.0 };
	ee.getMpe(pp);
	aris::dynamic::s_as2ap(m_->partPool().back().vs(), m_->partPool().back().as(), pp, ap, vp);

	// 电机端
	double aj1, aj2, aj3;
	double as[6];
	for (auto& m : m_->motionPool()) m.updA();
	m_->jointPool()[0].makI()->getAs(*m_->jointPool()[0].makJ(), as);
	aj1 = as[5];
	aj2 = m_->motionPool()[0].ma();
	aj3 = m_->motionPool()[1].ma();
	
	// get b 
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(5, 1);
	B(0, 0) = ap[0];
	B(1, 0) = ap[1];
	B(2, 0) = aj1;
	B(3, 0) = aj2;
	B(4, 0) = aj3;

	// set force[2] = { 1, 0.0 }
	force[0] = 1.0;
	force[1] = 0.0;
	force1.setFce(force[0]);
	force2.setFce(force[1]);
	forward_dynamic_solver.dynAccAndFce();
	ee.getMpe(pp);
	aris::dynamic::s_as2ap(m_->partPool().back().vs(), m_->partPool().back().as(), pp, ap, vp);

	// 电机端
	for (auto& m : m_->motionPool()) m.updA();
	m_->jointPool()[0].makI()->getAs(*m_->jointPool()[0].makJ(), as);
	aj1 = as[5];
	aj2 = m_->motionPool()[0].ma();
	aj3 = m_->motionPool()[1].ma();

	A(0, 0) = ap[0] - B(0, 0);
	A(1, 0) = ap[1] - B(1, 0);
	A(2, 0) = aj1 - B(2, 0);
	A(3, 0) = aj2 - B(3, 0);
	A(4, 0) = aj3 - B(4, 0);

	//aris::dynamic::dsp(1, 3, ap);

	// set force[2] = { 0.0, 1.0 }
	force[0] = 0.0;
	force[1] = 1.0;
	force1.setFce(force[0]);
	force2.setFce(force[1]);
	forward_dynamic_solver.dynAccAndFce();
	ee.getMpe(pp);
	aris::dynamic::s_as2ap(m_->partPool().back().vs(), m_->partPool().back().as(), pp, ap, vp);

	// 电机端
	for (auto& m : m_->motionPool()) m.updA();
	m_->jointPool()[0].makI()->getAs(*m_->jointPool()[0].makJ(), as);
	aj1 = as[5];
	aj2 = m_->motionPool()[0].ma();
	aj3 = m_->motionPool()[1].ma();

	A(0, 1) = ap[0] - B(0, 0);
	A(1, 1) = ap[1] - B(1, 0);
	A(2, 1) = aj1 - B(2, 0);
	A(3, 1) = aj2 - B(3, 0);
	A(4, 1) = aj3 - B(4, 0);

	Eigen::MatrixXd torq = Eigen::MatrixXd::Zero(2, 1);
	Eigen::MatrixXd Acc_ = Eigen::MatrixXd::Zero(5, 1);

	///////////////////////// check acc ////////////////////
	torq(0, 0) = last_toq_[0];
	torq(1, 0) = last_toq_[1];
	Acc_ = A * torq + B;
	desired_acc_[0] = Acc_(0, 0);
	desired_acc_[1] = Acc_(1, 0);
	desired_acc_[2] = Acc_(2, 0);
	desired_acc_[3] = Acc_(3, 0);
	desired_acc_[4] = Acc_(4, 0);
	////////////////////////////////////////////////////////

	Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(5, 1);

	Acc(0, 0) = state_var[6];
	Acc(1, 0) = state_var[7];
	Acc(2, 0) = state_var[8];
	Acc(3, 0) = state_var[9];
	Acc(4, 0) = state_var[10];


	Eigen::MatrixXd Inv_A = computePeseudoInverse(A);

	double U[10]{};
	double Inv_[10]{};
	double tau[5]{};
	double tau2[5]{};
	aris::Size p[5];
	aris::Size rank;

	aris::dynamic::s_householder_utp(5, 2, A.data(), ColMajor(5), U, 2, tau, 1, p, rank, 1e-3);
	// 列主元 ColMajor(5)， U 是内存 》5*2 ；p是数组的序列，5  rank 秩  tau 1, 单个排列 
	// aris::dynamic::s_householder_utp(5, 2, A.data(), 2, U, 2, tau, 1, p, rank, 1e-3);
	aris::dynamic::s_householder_up2pinv(5, 2, rank, U, tau, p, Inv_, tau2, 1e-3);
	

	Eigen::MatrixXd rhs = Acc - B;
	// calculate torque
	torq = Inv_A * rhs;

	static int count_ = 0;
	if (++count_ % 50 == 0) {
		//std::cout << A << std::endl;
		//std::cout << Inv_A << std::endl;
		//std::cout << Inv_A*A << std::endl;
		//std::cout << rhs << std::endl << std::endl;
		//std::cout << A * torq << std::endl << std::endl;
		//std::cout << A * torq - rhs << std::endl << std::endl;
		std::cout << "count " << count_ << " ---------------------------------------------------- " << std::endl;

		std::cout << "A: " << std::endl;
		std::cout << A << std::endl;

		std::cout << "householder: " << std::endl;
		aris::dynamic::dsp(2, 5, Inv_);

		std::cout << "compute Peseudo Inverse: " << std::endl;
		std::cout << Inv_A << std::endl;

		std::cout << "acc before:" << Acc(0, 0) << " " << Acc(1, 0) << " " << Acc(2, 0) << " " <<
			Acc(3, 0) << " " << Acc(4, 0) << " " << std::endl;
		Eigen::MatrixXd Acc_after(2, 1);
		Acc_after = A * torq + B;

		std::cout << "acc after:  " << Acc_after(0, 0) << " " << Acc_after(1, 0) << " " <<
			Acc_after(2, 0) << " " << Acc_after(3, 0) << " " << Acc_after(4, 0) << std::endl;

		//Eigen::MatrixXd rhs_after(2, 1);
		//rhs_after = Acc - B;

		//std::cout << "rhs before:" << rhs_after(0, 0) << " " << rhs_after(1, 0) << " " << rhs_after(2, 0) << " " <<
		//	rhs_after(3, 0) << " " << rhs_after(4, 0) << " " << std::endl;
		//std::cout << "rhs norm:" << rhs_after.norm() << std::endl;

		//rhs_after = Acc_after - B;
		//std::cout << "rhs after:" << rhs_after(0, 0) << " " << rhs_after(1, 0) << " " << rhs_after(2, 0) << " " <<
		//	rhs_after(3, 0) << " " << rhs_after(4, 0) << " " << std::endl;
		//std::cout << "rhs norm:" << rhs_after.norm() << std::endl;
	}

	//if (std::abs(torque[0] - torq(0, 0)) > 50) {
	//	std::cout << " A " << std::endl;
	//	std::cout << A << std::endl;
	//	std::cout << std::endl;

	//	std::cout << " B " << std::endl;
	//	std::cout << B << std::endl;
	//	std::cout << std::endl;

	//	std::cout << "Inverse A " << std::endl;
	//	std::cout << Inv_A << std::endl;
	//	std::cout << std::endl;

	//	std::cout << "torque : " << std::endl;
	//	std::cout << torq(0, 0) << "  " << torq(1, 0) << " " << std::endl;



	//	torque[0] = torq(0, 0);
	//	torque[1] = torq(1, 0);
	//}
	//else {
	//	torque[0] = torq(0, 0);
	//	torque[1] = torq(1, 0);
	//	
	//}






	//torq(0, 0) = fce_;
	//torq(1, 0) = 0;
	//Acc_ = A * torq + B;

	auto max_fce = 100.0;

	torque[0] = std::min(torq(0, 0), max_fce);
	torque[0] = std::max(torque[0], -max_fce);
	torque[1] = std::min(torq(1, 0), max_fce);
	torque[1] = std::max(torque[1], -max_fce);

	//std::cout << torque[0] << "  " << torque[1] << " " << std::endl;

	std::vector<double> acc_(2);
	acc_[0] = Acc_(0, 0);
	acc_[1] = Acc_(1, 0);
	saveDataToFile(state_var, torque, acc_);

	last_toq_ = torque;
}

// calculate forward Kinematics 
std::vector<double> TripleModel::calcu_forwardKinematics(std::vector<double>& joint_angle) {
	std::vector<double> xytheta;
	double Joint_pos[3][6] = { {0, 0, 0, 0, 0, joint_angle[0]},
							   {0, 0, 0, 0, 0, joint_angle[1]},
							   {0, 0, 0, 0, 0, joint_angle[2]} };

	for (int i = 0; i < 3; i++) {
		m_->jointPool().at(i).makI()->setPe(*m_->jointPool().at(i).makJ(), Joint_pos[i], "123");
	}
	for (auto& m : m_->motionPool()) m.updP();

	m_->generalMotionPool()[0].updP();

	double ee_position[6] = { 0,0,0,0,0,0 };
	m_->getOutputPos(ee_position);
	ee_position[3] = std::fmod(ee_position[3], 2 * PI);
	//aris::dynamic::dsp(1,6,ee_position);
	xytheta.push_back(ee_position[0]);
	xytheta.push_back(ee_position[1]);
	xytheta.push_back(ee_position[3]);

	return xytheta;
}

TripleModel::TripleModel() {
	const int torque_size = 2;
	torque.resize(torque_size);
}
TripleModel::~TripleModel() = default;

// 将Jf(const T*) 和 M 转换为数组 
template <typename T>
double* Convert2Arr(const T* data, int rows, int cols) {

	int size = rows * cols;
	double* arr = new double[size];

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			arr[rows * j + i] = data[cols * i + j];
		}
	}
	return arr;
}

Eigen::MatrixXd computePeseudoInverse(const Eigen::MatrixXd& jacobian) {
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const double tolerance = 1e-7; // 容忍度，用于判断奇异值是否为零
	double toleranceSq = tolerance * tolerance;

	// 计算伪逆矩阵
	Eigen::MatrixXd singularValuesInv = svd.singularValues();
	for (int i = 0; i < singularValuesInv.size(); i++) {
		if (singularValuesInv(i) > toleranceSq)
			singularValuesInv(i) = 1.0 / singularValuesInv(i);
		else
			singularValuesInv(i) = 0.0;
	}
	return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}
