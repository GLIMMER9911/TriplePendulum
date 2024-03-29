#ifndef PLAN_H
#define PLAN_H

#include <iostream>
#include <aris.hpp>
#include "export.h"

namespace triple {
	class BAL_CTRL_API fixedPlan {
	public:
		void setStateVal(std::vector<double>& data) { this->input_data_ = data; };
		auto calcuStateVal()->std::vector<double>;
		auto getStateVal() -> std::vector<double>;
		fixedPlan(std::vector<double> targetdata, std::vector<double> weight);
		~fixedPlan();
	private:
		// struct stateImp;
		// 计算末端加速度所需要的变量；
		struct stateImp {
			double targetPos;
			double trajectoryVel;
			double trajectoryAcc;
			double statePos;
			double stateVel;
			double desiredVel;
			double desiredAcc;
			double weightKp;
			double weightKv;
			double maxVel;
			double maxAcc;
			int count = 0;

			// initialize the struct parameters target, weightkp, weightKv and maxVel, maxAcc use default values
			stateImp(double target, double kp, double kv) : targetPos{ target }, weightKp{ kp }, weightKv{ kv } {
				this->maxVel = 10;
				this->maxAcc = 100;
				this->trajectoryVel = 0;
				this->trajectoryAcc = 0;

				if (target == 0) {
					this->targetPos = 0.000000001;
				}
			};
			
			// initialize the struct parameters target, weightkp, weightKv, maxVel, maxAcc
			stateImp(double target, double kp, double kv, double maxv, double maxa) :
				targetPos{ target }, weightKp{ kp }, weightKv{ kv }, maxVel{ maxv }, maxAcc{ maxa } {
				this->trajectoryVel = 0;
				this->trajectoryAcc = 0;
				if (target == 0) {
					this->targetPos = 0.000000001;
				}
			};

			void getStatedata(double pos, double vel) {
				this->statePos = pos;
				this->stateVel = vel;
			}

			void calcuDesiredVel() {
				if (trajectoryVel > 0) {
					desiredVel = weightKp * (targetPos - statePos) + 2.0 * trajectoryVel;
				}
				else {
					desiredVel = weightKp * (targetPos - statePos) + 2.0 * trajectoryVel;
				}
				desiredVel = std::max(desiredVel, -maxVel);
				this->desiredVel = std::min(desiredVel, maxVel);
			}

			void calcuDesiredAcc() {
				this->calcuDesiredVel();
				
				if (trajectoryAcc > 0) {
					desiredAcc = weightKv * (desiredVel - stateVel) + 2.0 * trajectoryAcc;
				}
				else {
					desiredAcc = weightKv * (desiredVel - stateVel);
				}

				desiredAcc = std::max(desiredAcc, -maxAcc);
				this->desiredAcc = std::min(desiredAcc, maxAcc);
			}

		};

		std::vector<double> input_data_{};
		std::vector<stateImp>  stateVar_{};
	};



};

#endif // !PLAN_H
