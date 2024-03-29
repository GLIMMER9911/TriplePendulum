#ifndef PLAN_H
#define PLAN_H

#include <iostream>
#include <vector>
#include <aris.hpp>

namespace triple {

	class fixedPlan {
	public:
		void setStateVal(std::vector<double>& data) { this->input_data_ = data; };
		auto calcuStateVal()->std::vector<double>;
		auto getStateVal() -> std::vector<double>;
		fixedPlan(std::vector<double>& targetdata, std::vector<double>& weight);
		~fixedPlan();
	private:
		struct stateImp;
		std::vector<double> input_data_{};
		std::vector<stateImp>  stateVar_{};
	};



};

#endif // !PLAN_H
