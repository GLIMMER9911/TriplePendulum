#ifndef PLAN_H
#define PLAN_H

#include <aris.hpp>
#include <memory>

class MotionTest : public aris::core::CloneObject< MotionTest, aris::plan::Plan >
{
public:
	auto virtual prepareNrt() -> void;
	auto virtual executeRT() -> int;
	auto virtual collectNrt() -> void;

	virtual ~MotionTest();
	explicit MotionTest(const std::string& name = "motion_test");

private:
	double cef_;
};

class FDTest : public aris::core::CloneObject< FDTest, aris::plan::Plan>
{
public:
	auto virtual prepareNrt() ->void;
	auto virtual executeRT() -> int;
	auto virtual collectNrt() -> void;

	virtual ~FDTest() = default;
	explicit FDTest(const std::string &name = "fd_test");

private:
};

class ForceControlTest
	: public aris::core::CloneObject<ForceControlTest, aris::plan::Plan> {
public:
	auto virtual prepareNrt() -> void override;
	auto virtual collectNrt() -> void override;
	auto virtual executeRT() -> int override;
	virtual ~ForceControlTest() = default;
	explicit ForceControlTest(const std::string& name = "fc_test");
	ARIS_DECLARE_BIG_FOUR(ForceControlTest);

private:
	struct Imp;
	aris::core::ImpPtr<Imp> imp_;
};

#endif