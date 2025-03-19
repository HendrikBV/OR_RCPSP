#include "algorithms.h"
#include <fstream>
#include <random>
#include <stdexcept>
#include <iostream>



namespace RCPSP
{
	Output& operator<<(Output& output, const std::string& msg)
	{
		if (output._on) {
			std::cout << msg;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, const char* msg)
	{
		if (output._on) {
			std::cout << msg;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, long unsigned int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, size_t value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, unsigned int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, int value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, int64_t value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, float value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}

	Output& operator<<(Output& output, double value)
	{
		if (output._on) {
			std::cout << value;
		}
		// otherwise no output
		return output;
	}


	///////////////////////////////////////////////////////////////////////////


	void Algorithm::read_data(const std::string& filename)
	{
		// DATA SHOULD INCLUDE DUMMY START AND END ACTIVITIES

		std::ifstream file;
		file.open(filename);
		if (!file.is_open())
		{
			throw std::invalid_argument("Couldn't open the file with name " + filename);
		}

		// 0. clear data
		_resource_availabilities.clear();
		_precedence_relations.clear();
		_activity_durations.clear();
		_activity_resource_requirements.clear();

		// 1. resources
		file >> _nb_resources;
		_resource_availabilities.reserve(_nb_resources);
		for (int k = 0; k < _nb_resources; ++k)
		{
			int val;
			file >> val;
			_resource_availabilities.push_back(val);
		}

		// 2. activities
		file >> _nb_activities;
		_activity_durations.reserve(_nb_activities);
		_activity_resource_requirements.reserve(_nb_activities);
		_precedence_relations.reserve(_nb_activities);

		// 2.a durations
		for (int i = 0; i < _nb_activities; ++i)
		{
			int val;
			file >> val;
			_activity_durations.push_back(val);
		}

		// 2.b resource requirements
		for (int i = 0; i < _nb_activities; ++i)
		{
			std::vector<int> vec;
			vec.reserve(_nb_resources);
			_activity_resource_requirements.push_back(vec);

			for (int k = 0; k < _nb_resources; ++k)
			{
				int val;
				file >> val;
				_activity_resource_requirements.back().push_back(val);
			}
		}

		// 2.c precedence relations
		for (int i = 0; i < _nb_activities; ++i)
		{
			std::vector<int> vec;
			vec.reserve(_nb_activities);
			_precedence_relations.push_back(vec);

			for (int j = 0; j < _nb_activities; ++j)
			{
				int val;
				file >> val;
				_precedence_relations.back().push_back(val);
			}
		}




		// Sanity check
		// A) dummy start and end
		if (_activity_durations.front() != 0 || _activity_durations.back() != 0)
		{
			throw std::logic_error("Duration dummy start or dummy end activity is not 0");
		}
		for (int k = 0; k < _nb_resources; ++k)
		{
			if (_activity_resource_requirements.front()[k] != 0 || _activity_resource_requirements.back()[k] != 0)
			{
				throw std::logic_error("Resource requirements for dummy start or dummy end activity are not 0");
			}
		}

		// B) check that resources do not exceed availability for individual resources
		for (int i = 0; i < _nb_activities; ++i)
		{
			for (int k = 0; k < _nb_resources; ++k)
			{
				int req = _activity_resource_requirements[i][k];
				int av = _resource_availabilities[k];
				if (req > av)
				{
					throw std::logic_error("Resource requirement for activity " + std::to_string(i + 1)
						+ " for resource type " + std::to_string(k + 1) + " exceeds availability");
				}
			}
		}
	}


	///////////////////////////////////////////////////////////////////////////


	std::unique_ptr<Algorithm> AlgorithmFactory::create(std::string& algorithm)
	{
		std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(),
			[](unsigned char c) { return std::tolower(c); });


		if (algorithm == "IP")
			return std::make_unique<SCIP1>();
		else if (algorithm == "DH")
			return std::make_unique<DH>();
		else
			throw std::invalid_argument("No algorithm " + algorithm + " exists");
	}
}