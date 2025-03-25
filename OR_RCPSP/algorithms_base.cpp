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

		// clear old data
		_resource_availabilities.clear();
		_activities.clear();


		std::ifstream file;
		file.open(filename);
		if (!file.is_open())
		{
			throw std::invalid_argument("Couldn't open the file with name " + filename);
		}

		int nb_activities = 0, nb_resources = 0;
		file >> nb_activities >> nb_resources;

		_resource_availabilities.reserve(nb_resources);
		for (int k = 0; k < nb_resources; ++k)
		{
			int val;
			file >> val;
			_resource_availabilities.push_back(val);
		}

		_activities.reserve(nb_activities);
		for (int i = 0; i < nb_activities; ++i)
		{
			_activities.push_back(Activity());
			_activities.back().id = i;

			int dur;
			file >> dur;
			_activities.back().duration = dur;

			_activities.back().resource_requirements.reserve(nb_resources);
			for (int k = 0; k < nb_resources; ++k)
			{
				int res;
				file >> res;
				_activities.back().resource_requirements.push_back(res);
			}

			int nbsucc;
			file >> nbsucc;
			_activities.back().successors.reserve(nbsucc);
			for (int s = 0; s < nbsucc; ++s)
			{
				int suc;
				file >> suc;
				--suc; // numbering starts at 1 instead of 0
				_activities.back().successors.push_back(suc);
			}
		}


		// calculate predecessors
		for (int i = 0; i < _activities.size(); ++i)
		{
			int pred = i;
			for (auto&& suc : _activities[i].successors)
			{
				_activities[suc].predecessors.push_back(pred);
			}
		}



		// Sanity check
		// A) dummy start and end
		if (_activities.front().duration != 0 || _activities.back().duration != 0)
		{
			throw std::logic_error("Duration dummy start or dummy end activity is not 0");
		}
		for (int k = 0; k < nb_resources; ++k)
		{
			if (_activities.front().resource_requirements[k] != 0 ||
				_activities.back().resource_requirements[k] != 0)
			{
				throw std::logic_error("Resource requirements for dummy start or dummy end activity are not 0");
			}
		}

		// B) check that resources do not exceed availability for individual resources
		for (int i = 0; i < nb_activities; ++i)
		{
			for (int k = 0; k < nb_resources; ++k)
			{
				int req = _activities[i].resource_requirements[k];
				int av = _resource_availabilities[k];
				if (req > av)
				{
					throw std::logic_error("Resource requirement for activity " + std::to_string(i + 1)
						+ " for resource type " + std::to_string(k + 1) + " exceeds availability");
				}
			}
		}
	}


	void Algorithm::check_solution()
	{
		bool ok = true;
		std::cout << "\n";

		// resource use
		for (int t = 0; t < _upper_bound; ++t)
		{
			for (int k = 0; k < _resource_availabilities.size(); ++k)
			{
				int resource_use = 0;
				for (int i = 0; i < _activities.size(); ++i)
				{
					if (_best_activity_finish_times[i] - _activities[i].duration >= t
						&& _best_activity_finish_times[i] < t)
					{
						resource_use += _activities[i].resource_requirements[k];
					}
				}

				if (resource_use > _resource_availabilities[k])
				{
					ok = false;
					std::cout << "\nResource use in period " << t << " exceeds resource availabilities";
				}
			}
		}

		// precedences
		for (int i = 0; i < _activities.size(); ++i)
		{
			for (auto&& suc : _activities[i].successors)
			{
				if (_best_activity_finish_times[suc] - _activities[suc].duration < _best_activity_finish_times[i])
				{
					ok = false;
					std::cout << "Activity " << i+1 << " finishes at time " << _best_activity_finish_times[i]
						<< " but its successor " << suc + 1 << " already starts at time "
							<< _best_activity_finish_times[suc] - _activities[suc].duration;
				}
			}
		}

		if (ok)
			std::cout << "\nCheck solution: OK";
	}


	///////////////////////////////////////////////////////////////////////////


	std::unique_ptr<Algorithm> AlgorithmFactory::create(std::string& algorithm)
	{
		std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(),
			[](unsigned char c) { return std::tolower(c); });


		if (algorithm == "dh")
			return std::make_unique<DH>();
		else if (algorithm == "ip")
			return std::make_unique<IP>();
		else
			throw std::invalid_argument("No algorithm " + algorithm + " exists");
	}
}