#include "algorithms.h"
#include <chrono>
#include <stdexcept>



namespace RCPSP
{
	void DH::run(bool verbose)
	{
		_output.set_on(true);
		_output << "\nStarting branch-and-bound procedure of Demeulemeester-Herroelen ...\n";
		_output.set_on(verbose);

		auto start_time = std::chrono::system_clock::now();

		

		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;

		_output.set_on(true);
		_output << "\n\nElapsed time (s): " << elapsed_time.count();
		/*_output << "\nNodes evaluated: " << _nb_nodes;
		_output << "\nNodes pruned: " << _nb_nodes_pruned;
		_output << "\nBest sequence: ";
		for (auto&& j : _best_sequence)
			_output << j + 1 << " ";
		_output << "\nTardiness: " << _best_value;*/
	}
}