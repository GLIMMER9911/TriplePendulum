#include "plan.hpp"


///--------------------------------- 运动学轨迹验证 ----------------------------------///
// 给两个电机一个 sin() 函数，对末端轨迹点进行验证
struct MotionTestParam {
	std::vector<double> joint_pos, joint_vel, joint_acc,
		joint_pos_begin, joint_pos_end;
	std::vector<aris::Size> total_count;
};
auto MotionTest:: prepareNrt() -> void
{
	double cef_ = doubleParam("myplan");
	for (auto& option : motorOptions()) option =
		aris::plan::Plan::NOT_CHECK_ENABLE |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

}
auto MotionTest:: executeRT()-> int
{
	const double PI = 3.141592653589793;
	double begin_angle[2]{0};

	// 设置时间，如果伺服周期为1000Hz，那么每个周期时间递加0.001s //
	model()->setTime(0.001 * count());
	if (count() == 1) {
		this->master()->logFileRawName("MotionTest");//建立记录数据的文件夹
	}

	// 设置电机位置 //
	double motion_pos[2]{ 0 };
	motion_pos[0] = 2 * cef_* std::sin(PI * 2 * count() / 1000);
	motion_pos[1] = 2 * cef_* std::sin(PI * 2 * count() / 1000);

	for (int i = 0; i < 2; i++) {
		dynamic_cast<aris::dynamic::Motion&>(model()->motionPool()[i]).setMp(motion_pos[i]);
	}
	// 求正解 //
	if (model()->solverPool().at(2).kinPos()) throw std::runtime_error("kinematic position failed");
	
	double end_pe321[6]{ 0 };
	auto& gm = model()->generalMotionPool().at(0);
	gm.getP(end_pe321);

	// 打印 //
	if (count()%10 == 0)
	{
		mout() << "motion2 :" << model()->motionPool()[0].mp() << "\t";
		mout() << "motion3 :" << model()->motionPool()[1].mp() << "\t";
		mout() << "End effector position : " << end_pe321[0] << " " << end_pe321[1] << " "<< end_pe321[2] << " " 
			<< end_pe321[3] << " " << end_pe321[4] << " " << end_pe321[5] << " " << "\t";
		mout() << std::endl;
	}
	// log //
	
	lout() << end_pe321[0] << " " << end_pe321[1] << " " << end_pe321[2] << " "
		<< end_pe321[3] << " " << end_pe321[4] << " " << end_pe321[5] << " " << std::endl;

	// 轨迹的长度和返回值有关，返回0时轨迹结束，这里仿真1000个周期 //
	return 1000 - count();
}
auto MotionTest::collectNrt() -> void {}
MotionTest::MotionTest(const std::string& name)
{
	aris::core::fromXmlString(command(),
		"<Command name=\"mtest\">"
		"	<GroupParam>"
		"		<Param name=\"myplan\" default=\"1\"/>"
		"	</GroupParam>"
		"</Command>");
}
MotionTest::~MotionTest() = default;  //析构函数

///--------------------------------- 正向动力学轨迹验证 ----------------------------------///
// 在初始位置给两个电机一个为零的力，验证末端轨迹
auto FDTest::prepareNrt() ->void
{
	for (auto& option : motorOptions()) {option =
			aris::plan::Plan::NOT_CHECK_ENABLE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
}
auto FDTest::executeRT() ->int {
	
	// 设置时间，如果伺服周期为1000Hz，那么每个周期时间递加0.001s //
	model()->setTime(0.001 * count());
	if (count() == 1) {
		double ee_begin_pos[6] = { 1.2, 0, 0, 0, 0, 0, };
		double ee_begin_vel[6] = { 0, 0, 0, 0, 0, 0, };

		model()->generalMotionPool()[0].setP(ee_begin_pos);
		model()->generalMotionPool()[0].setV(ee_begin_vel);

		model()->inverseKinematics();
		model()->inverseKinematicsVel();
		this->master()->logFileRawName("FDTest");//建立记录数据的文件夹
	}

	//输入电机的力
	std::vector<double> force_data = { 0.0, 0.0 };
	model()->setInputFce(force_data.data());
	// 获得末端的位置
	double end_pe321[6]{ 0 };
	auto& gm = model()->generalMotionPool().at(0);
	gm.getP(end_pe321);

	// 打印 //
	if (count() % 10 == 0)
	{
		mout() << "motion2 :" << model()->motionPool()[0].mp() << "\t";
		mout() << "motion3 :" << model()->motionPool()[1].mp() << "\t";
		mout() << "End effector position : " << end_pe321[0] << " " << end_pe321[1] << " " << end_pe321[2] << " "
			<< end_pe321[3] << " " << end_pe321[4] << " " << end_pe321[5] << " " << "\t";
		mout() << std::endl;
	}
	// log //

	lout() << end_pe321[0] << " " << end_pe321[1] << " " << end_pe321[2] << " "
		<< end_pe321[3] << " " << end_pe321[4] << " " << end_pe321[5] << " " << std::endl;

	return count() > 1000 ? 0 : 1;
}
auto FDTest::collectNrt() -> void {}
FDTest::FDTest(const std::string& name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"fdtest\">"
		"</Command>"
	);
}

struct ForceControlTest::Imp {};
auto ForceControlTest::prepareNrt() -> void {
	for (auto& option : motorOptions())
		option |= aris::plan::Plan::USE_TARGET_TOQ;
}
auto ForceControlTest::executeRT() -> int {
	std::vector<double> force_data = { 0.0, 0.0 };
	model()->setInputFce(force_data.data());
	return count() > 5000 ? 0 : 1;
}
auto ForceControlTest::collectNrt() -> void {}
ForceControlTest::ForceControlTest(const std::string& name) : imp_(new Imp) {
	aris::core::fromXmlString(command(),
		"<Command name=\"fctest\">"
		"</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(ForceControlTest);



ARIS_REGISTRATION{
	aris::core::class_<MotionTest>("MotionTest").inherit<aris::plan::Plan>();
	aris::core::class_<FDTest>("FDTest").inherit<aris::plan::Plan>();
	aris::core::class_<ForceControlTest>("ForceControlTest")
		.inherit<aris::plan::Plan>();
}

