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
const double g = -9.81;

//  Create robot model
auto triple::TripleModel::createModel() -> std::unique_ptr<Model> {

	//double a = 0.4;
	//double b = 0.4;
	//double c = 0.4;

	//// 定义关节的位置，以及轴线，有3个转动副，轴线都是Z轴
	//const double joint1_position[3]{ 0 , 0 , 0 };
	//const double joint1_axis[3]{ 0 , 0 , 1 };
	//const double joint2_position[3]{ 0 , a , 0 };
	//const double joint2_axis[3]{ 0 , 0 , 1 };
	//const double joint3_position[3]{ 0 , a + b , 0 };
	//const double joint3_axis[3]{ 0 , 0 , 1 };

	//// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	//// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	//const double link1_pos_euler[6]{ 0, a / 2, 0, PI / 2, 0, 0 };
	//const double link1_intertia_vector[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	//const double link2_pos_euler[6]{ 0, a + b / 2, 0, PI / 2, 0, 0 };
	//const double link2_intertia_vecter[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	//const double link3_pos_euler[6]{ 0,  a + b + c / 2, 0, PI / 2, 0, 0 };
	//const double link3_intertia_vecter[10]{ 2 , 0 , 0 , 0 ,8.333333333338782e-04, 0.0271, 0.0271, 0, 0, 0 };
	//const double body_intertia_vecter[10]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	//// 定义末端位置与321欧拉角
	//const double body_position_and_euler321[6]{ 0 , a + b + c , 0 , PI / 2 , 0 , 0 };

	//double a = 0.325;
	//double b = 0.2;
	//double c = 0.2;

	//// 定义关节的位置，以及轴线，有3个转动副，轴线都是Z轴
	//const double joint1_position[3]{ 0 , 0 , 0 };
	//const double joint1_axis[3]{ 0 , 0 , 1 };
	//const double joint2_position[3]{ 0 , a , 0 };
	//const double joint2_axis[3]{ 0 , 0 , 1 };
	//const double joint3_position[3]{ 0 , a + b , 0 };
	//const double joint3_axis[3]{ 0 , 0 , 1 };

	//// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	//// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	//const double link1_pos_euler[6]{ 0, 245.436*1e-3, 0, PI / 2, 0, 0 };
	//const double link1_intertia_vector[10]{ 1.915 , 0 , 0 , 0 , 0, 0, 25720.752 * 1e-6, 0, 0, 0 };
	//const double link2_pos_euler[6]{ 0, a + 165.468*1e-3, 0, PI / 2, 0, 0 };
	//const double link2_intertia_vecter[10]{ 1.469 , 0 , 0 , 0 , 0, 0, 7667.511 * 1e-6, 0, 0, 0 };
	//const double link3_pos_euler[6]{ 0,  a + b + 163.706*1e-3, 0, PI / 2, 0, 0 };
	//const double link3_intertia_vecter[10]{ 2.285 , 0 , 0 , 0 , 0, 0, 8719.303 * 1e-6, 0, 0, 0 };
	//const double body_intertia_vecter[10]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };



	// Acrobot model, double pendulum
	double a = 0.325;
	double b = 0.2;
	double c = 0.2;

	// 定义关节的位置，以及轴线，有3个转动副，轴线都是Z轴
	const double joint1_position[3]{ 0 , 0 , 0 };
	const double joint1_axis[3]{ 0 , 0 , 1 };
	const double joint2_position[3]{ 0 , a , 0 };
	const double joint2_axis[3]{ 0 , 0 , 1 };
	const double joint3_position[3]{ 0 , a + b , 0 };
	const double joint3_axis[3]{ 0 , 0 , 1 };

	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	const double link1_pos_euler[6]{ 0, 245.436 * 1e-3, 0, PI / 2, 0, 0 };
	const double link1_intertia_vector[10]{ 1.915 , 0 , 0 , 0 , 0, 0, 25720.752 * 1e-6, 0, 0, 0 };
	const double link2_pos_euler[6]{ 0, a + 165.468 * 1e-3, 0, PI / 2, 0, 0 };
	const double link2_intertia_vecter[10]{ 1.469 , 0 , 0 , 0 , 0, 0, 7667.511 * 1e-6, 0, 0, 0 };
	const double link3_pos_euler[6]{ 0,  a + b + 127.337 * 1e-3, 0, PI / 2, 0, 0 };
	const double link3_intertia_vecter[10]{ 1.141 , 0 , 0 , 0 , 0, 0, 4949.014 * 1e-6, 0, 0, 0 }; // 去掉了负载
	//const double link3_intertia_vecter[10]{ 2.285 , 0 , 0 , 0 , 0, 0, 8719.303 * 1e-6, 0, 0, 0 }; // 加上负载
	const double body_intertia_vecter[10]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	// 定义末端位置与321欧拉角
	const double body_position_and_euler321[6]{ 0 , a + b + c , 0 , PI / 2 , 0 , 0 };

	// 定义模型
	std::unique_ptr<Model> model(new Model);

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
	//auto& motion1 = model->addMotion(joint1);
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
	std::cout << " Successful modeling ! " << std::endl;

	return model;

	//// test Gv and Gw
	//int nM = 0;
	//inverse_dynamic_solver.dynAccAndFce();
	//inverse_dynamic_solver.cptGeneralInverseDynamicMatrix();
	//nM = inverse_dynamic_solver.nM();

	//const double* M = inverse_dynamic_solver.M();
	//const double* h = inverse_dynamic_solver.h();

	//std::cout << "nM:" << nM << std::endl;
	//std::cout << std::endl;

	//std::cout << "M: " << std::endl;
	//aris::dynamic::dsp(nM, nM, M);

	//std::cout << "h: " << std::endl;
	//aris::dynamic::dsp(nM, 1, h);


	//this->m_ = std::move(model);

	//if (model) {
	//	std::cerr << " Model is not empty ! " << std::endl;
	//}


}

// calculate forward Kinematics 
// data : joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3 
void triple::TripleModel::calcuForwardKinematics(std::vector<double>& data) {

	double Joint_pos[3][6] = { {0, 0, 0, 0, 0, data[0]},
							   {0, 0, 0, 0, 0, data[1]},
							   {0, 0, 0, 0, 0, data[2]} };
	double joint_velocity[3][6] = { {0, 0, 0, 0, 0, data[3]},
							   {0, 0, 0, 0, 0, data[4]},
							   {0, 0, 0, 0, 0, data[5]} };

	for (int i = 0; i < m_->jointPool().size(); ++i) {
		m_->jointPool().at(i).makI()->setPe(*m_->jointPool().at(i).makJ(), Joint_pos[i], "123");
	}
	for (auto& m : m_->motionPool()) m.updP();
	m_->generalMotionPool()[0].updP();

	// calculate the forward velocity
	for (int i = 0; i < m_->jointPool().size(); ++i) {
		m_->jointPool().at(i).makI()->fatherPart().setVs(*m_->jointPool().at(i).makJ(), joint_velocity[i]);
	}
	for (auto& m : m_->motionPool()) m.updV();
	m_->generalMotionPool()[0].updV();

	double ee_position[6] = { 0,0,0,0,0,0 };
	m_->getOutputPos(ee_position);
	ee_position[3] = std::fmod(ee_position[3], 2 * PI);

	double ee_velocity[6] = { 0,0,0,0,0,0 };
	dynamic_cast<aris::dynamic::GeneralMotion&>(m_->generalMotionPool()[0]).getMva(ee_velocity);



	data[6] = ee_position[0];
	data[7] = ee_position[1];
	data[8] = ee_position[3];

	data[9] = ee_velocity[0];
	data[10] = ee_velocity[1];
	data[11] = ee_velocity[5];

}

//  Create Acrobot model
auto triple::TripleModel::createAcrobotModel() -> std::unique_ptr<Model> {

	// featherstone model parameters setting
	double l1 = 0.4;
	double m1 = 0.49;
	//double lc1 = 0.1714;
	double lc1 = 0.2;
	double I1 = 0.0036;

	double l2 = 0.6;
	double m2 = 0.11;
	//double lc2 = 0.4364;
	double lc2 = 0.3;
	double I2 = 0.0043;
	//double a = 0.325;
	//double b = 0.4;

	// 定义关节的位置，以及轴线，有3个转动副，轴线都是Z轴
	const double joint1_position[3]{ 0 , 0 , 0 };
	const double joint1_axis[3]{ 0 , 0 , 1 };
	const double joint2_position[3]{ 0 , l1 , 0 };
	const double joint2_axis[3]{ 0 , 0 , 1 };

	// set 
	const double link1_pos_euler[6]{ 0, lc1, 0, PI / 2, 0, 0 };
	const double link1_intertia_vector[10]{ m1 , 0 , 0 , 0 , 0, 0, I1, 0, 0, 0 };
	const double link2_pos_euler[6]{ 0, l1 + lc2, 0, PI / 2, 0, 0 };
	const double link2_intertia_vecter[10]{ m2 , 0 , 0 , 0 , 0, 0, I2, 0, 0, 0 };

	//// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	//// inertia_vector为惯量矩阵，其的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	//const double link1_pos_euler[6]{ 0, 245.436 * 1e-3, 0, PI / 2, 0, 0 };
	//const double link1_intertia_vector[10]{ 1.915 , 0 , 0 , 0 , 0, 0, 25720.752 * 1e-6, 0, 0, 0 };
	////
	//const double link2_pos_euler[6]{ 0, a + 236.232 * 1e-3, 0, PI / 2, 0, 0 };
	//const double link2_intertia_vecter[10]{ 1 , 0 , 0 , 0 , 0, 0, 29445.260 * 1e-6, 0, 0, 0 };
	////const double link2_intertia_vecter[10]{ 2.610 , 0 , 0 , 0 , 0, 0, 29445.260 * 1e-6, 0, 0, 0 };

	const double body_intertia_vecter[10]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	// 定义末端位置与321欧拉角
	const double body_position_and_euler321[6]{ 0 , l1 + l2 , 0 , PI / 2 , 0 , 0 };

	// 定义模型
	std::unique_ptr<Model> model(new Model);

	// 设置重力,重力在y轴
	const double gravity[6]{ 0.0, g, 0.0, 0.0, 0.0, 0.0 };
	model->environment().setGravity(gravity);

	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto& link1 = model->addPartByPe(link1_pos_euler, "321", link1_intertia_vector);
	auto& link2 = model->addPartByPe(link2_pos_euler, "321", link2_intertia_vecter);

	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto& joint1 = model->addRevoluteJoint(link1, model->ground(), joint1_position, joint1_axis);
	auto& joint2 = model->addRevoluteJoint(link2, link1, joint2_position, joint2_axis);

	// 添加驱动 Joint1 为被动关节，不用加motion
	auto& motion1 = model->addMotion(joint1);
	auto& motion2 = model->addMotion(joint2);

	// 添加末端，第一个参数表明末端位于link4上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
	auto& end_effector = model->addGeneralMotionByPe(link2, model->ground(), body_position_and_euler321, "321");

	auto& force2 = model->forcePool().add< aris::dynamic::SingleComponentForce >("f2", motion2.makI(), motion2.makJ(), 5);

	//-------------------------------------------- 添加求解器 --------------------------------------------//
	/// [Solver]
	// 添加两个求解器，并为求解器分配内存。注意，求解器一但分配内存后，请不要再添加或删除杆件、关节、驱动、末端等所有元素
	auto& inverse_kinematic_solver = model->solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic_solver = model->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic_solver = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic_solver = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

	model->init();
	std::cout << " Successful Acrobot modeling ! " << std::endl;

	return model;
}

triple::TripleModel::TripleModel() {

}
triple::TripleModel::~TripleModel() = default;

// ARIS_REGISTRATION{
//     aris::core::class_<triple::TripleModel>("TripleModel")
//         .inherit<aris::dynamic::Model>()
//         ;
// }




