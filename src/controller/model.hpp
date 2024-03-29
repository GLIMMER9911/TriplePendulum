// model.h: 标准系统包含文件的包含文件

#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <fstream>
#include <aris.hpp>


namespace triple  {

	using namespace aris::dynamic;

	class TripleModel : public aris::dynamic::Model
	{
	public:
		auto TripleModel::createModel()->std::unique_ptr<Model>;
		auto TripleModel::createAcrobotModel() -> std::unique_ptr<Model>;
		void TripleModel::calcuForwardKinematics(std::vector<double>& data);
		auto TripleModel::getmodel()->std::shared_ptr<aris::dynamic::Model> { return this->m_; };
		auto TripleModel::getmodel()const -> const std::shared_ptr<aris::dynamic::Model> { return const_cast<TripleModel*>(this)->m_; };
		  
		TripleModel::TripleModel();

		TripleModel::~TripleModel();

	private:
		std::shared_ptr<aris::dynamic::Model> m_{nullptr};
	};
}





#endif // !MODEL_H