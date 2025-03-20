#include "algorithms.h"
#include <chrono>
#include <stdexcept>




namespace RCPSP
{
	/**************************************************************************
	*                               Print
	**************************************************************************/

	void DH::print_activity_set(const std::vector<int>& set)
	{
		int size = set.size();
		if (size == 0)
		{
			_output << "{ }";
			return;
		}

		_output << "{";
		for (int64_t i = 0; i < size - 1; ++i)
			_output << set[i] + 1 << ",";
		_output << set[size - 1] + 1 << "}";
	}

	void DH::print_cutset(const Cutset& c)
	{
		_output << "\n\t" << c.node << "\t" << c.parent_node << "\t" << c.decision_point << "\t"
			<< c.latest_finish_time_active_activities << "\t";
		print_activity_set(c.unassigned_activities);
	}



	/**************************************************************************
	*                            Auxiliary functions
	**************************************************************************/

	void DH::reset_all_variables()
	{
		// variables
		_optimal_activity_finish_times.clear();
		_RCPL.clear();
		_partial_schedule.clear();
		_active_activities.clear();
		_activity_finish_times.clear();
		_eligible_activities.clear();
		_branching_tree.clear();
		_saved_cutsets.clear();
		_resources_to_release.clear();
		_set_DS.clear();
		_current_precedence_relations.clear();

		// statistics
		_nodes_evaluated = 0;
		_nodes_LB_dominated = 0;
		_nodes_cutset_dominated = 0;
		_nodes_leftshift_dominated = 0;
		_nb_times_theorem3_applied = 0;
		_nb_times_theorem4_applied = 0;
		_saved_branching_tree.clear();
	}

	int DH::calculate_EST(int activity)
	{
		if (activity == 0)
			return 0;	// dummy start activity

		int EST = 0;
		for (int pred = 0; pred < _nb_activities; ++pred)
		{
			if (_current_precedence_relations[pred][activity] == 1)
			{
				int val = _activity_durations[pred] + calculate_EST(pred);
				if (val > EST)
					EST = val;
			}
		}
		return EST;
	}

	int DH::calculate_LFT(int activity, int deadline)
	{
		if (activity == _nb_activities - 1)
			return deadline;	// dummy end activity

		int LFT = _LARGE_VALUE;
		for (int succ = 0; succ < _nb_activities; ++succ)
		{
			if (_current_precedence_relations[activity][succ] == 1)
			{
				int val = calculate_LFT(succ, deadline) - _activity_durations[succ];
				if (val < LFT)
					LFT = val;
			}
		}
		return LFT;
	}

	int DH::calculate_RCPL(int activity)
	{
		if (activity == _nb_activities - 1)
			return 0;	// dummy end activity

		int cp = 0;
		for (int i = 0; i < _nb_activities; ++i)
		{
			if (_precedence_relations[activity][i] == 1) // original precedence relations
			{
				int val = _activity_durations[activity] + calculate_RCPL(i);
				if (val > cp)
					cp = val;
			}
		}
		return cp;
	}

	bool DH::calculate_critical_path(int activity, std::vector<int>& path, int current_length, int& best_length)
	{
		bool improved = false;

		if (activity == _nb_activities - 1)
		{
			if (current_length > best_length)
			{
				best_length = current_length;
				path.clear();
				improved = true;
			}
			return improved;
		}

		for (int i = 0; i < _nb_activities; ++i)
		{
			if (_current_precedence_relations[activity][i] == 1)
			{
				if (calculate_critical_path(i, path, current_length + _activity_durations[i], best_length))
				{
					improved = true;
					path.push_back(i);
				}
			}
		}

		return improved;
	}

	void DH::compute_minimal_delaying_alternatives(int index, std::vector<int> current_delay_set)
	{
		// Check if enough resources released with current set
		bool ok = true;
		for (int k = 0; k < _nb_resources; ++k)
		{
			int released = 0;
			for (auto&& act : current_delay_set)
				released += _activity_resource_requirements[act][k];

			if (released < _resources_to_release[k])
			{
				ok = false;
				break;
			}
		}

		// Check if minimal by removing one activity
		for (auto&& i : current_delay_set)
		{
			if (!ok)
				break;

			bool enough_released = true;
			for (int k = 0; k < _nb_resources; ++k)
			{
				int released = 0;
				for (auto&& act : current_delay_set)
					if (act != i)
						released += _activity_resource_requirements[act][k];

				if (released < _resources_to_release[k])
				{
					enough_released = false;
					break;
				}
			}

			if (enough_released)
				ok = false;
		}

		// If yes, add node
		if (ok)
		{
			++_nodes_evaluated;

			_branching_tree.back().nodes.push_back(Delaying_Alternative());
			_branching_tree.back().nodes.back().number = _nodes_evaluated;
			_branching_tree.back().nodes.back().delayed_activities = current_delay_set;
			return;
		}

		// Add new activity and do new recursion
		for (int i = 0; i < _active_activities.size(); ++i)
		{
			if (current_delay_set.empty())
			{	// Always add at least one activity
				current_delay_set.push_back(_active_activities[i]);

				// New recursion
				compute_minimal_delaying_alternatives(i, current_delay_set);
				current_delay_set.pop_back();
			}
			else if (i > index)
			{	// Do not add activity that alone is sufficient
				int act = _active_activities[i];
				bool alone_sufficient = true;
				for (int k = 0; k < _nb_resources; ++k)
				{
					if (_activity_resource_requirements[act][k] < _resources_to_release[k])
					{
						alone_sufficient = false;
						break;
					}
				}

				if (!alone_sufficient)
				{
					current_delay_set.push_back(_active_activities[i]);

					// New recursion
					compute_minimal_delaying_alternatives(i, current_delay_set);
					current_delay_set.pop_back();
				}
			}
		}
	}

	void DH::calculate_LB_node(Delaying_Alternative& node)
	{
		// A. value previous node
		int LB_A = _current_LB;

		// B. critical-path LB
		int LB_B = 0;
		if (!node.delayed_activities.empty())
		{
			int EFT = _LARGE_VALUE;
			int act_eft = -1;
			for (auto&& act : _branching_tree.back().active_activities)
			{
				bool not_delayed = true;
				for (auto&& delact : node.delayed_activities)
				{
					if (act == delact)
					{
						not_delayed = false;
						break;
					}
				}

				if (not_delayed && _branching_tree.back().activity_finish_times[act] < EFT)
				{
					EFT = _branching_tree.back().activity_finish_times[act];
					act_eft = act;
				}
			}
			node.earliest_finishing_activity_not_delayed = act_eft;

			int max_RCPL = 0;
			int act_RCPL = -1;
			for (auto&& delact : node.delayed_activities) {
				if (_RCPL[delact] > max_RCPL)
				{
					max_RCPL = _RCPL[delact];
					act_RCPL = delact;
				}
			}

			LB_B = EFT + max_RCPL;
			_output << "\n\nThe critical path LB equals the finish time of the earliest finishing activity that is not delayed + the maximum remaining critical path length from the delayed activities";
			_output << "\nEarliest finishing activity not delayed is " << act_eft + 1 << " and finishes at time " << EFT;
			_output << "\nMaximum RCPL starts at activity " << act_RCPL + 1 << " and has length " << max_RCPL;
			_output << "\nLB = " << EFT << " + " << max_RCPL << " = " << LB_B;

		}

		// C. critical-sequence LB
		int LB_C = 0;

		if(_verbose) // only use this LB if output is on to show how it can be calculated; otherwise it slows down the algorithms considerably
		{
			_output << "\n\nThe critical sequence LB is calculated as follows.";

			// Temporarily include new precedence relations
			for (auto&& delact : node.delayed_activities)
			{
				_current_precedence_relations[node.earliest_finishing_activity_not_delayed][delact] = 1;
				_current_precedence_relations[delact][node.earliest_finishing_activity_not_delayed] = -1;
			}

			std::vector<int> critical_path;
			int CP_length = 0;
			calculate_critical_path(0, critical_path, 0, CP_length); // path will be in reverse due to 'push_back' sequence
			critical_path.push_back(0);
			_output << "\nThe critical path equals ";
			for (int l = critical_path.size() - 1; l >= 0; --l)
			{
				_output << critical_path[l] + 1;
				if (l > 0) _output << "-";
			}
			_output << " with length " << CP_length;
			LB_C = CP_length;

			std::vector<std::vector<int>> resource_usage_CP;
			{
				resource_usage_CP.reserve(CP_length);
				int current_time = 0;
				for (int i = critical_path.size() - 1; i >= 0; --i)
				{
					int act = critical_path[i];
					for (int t = current_time; t < current_time + _activity_durations[act]; ++t)
					{
						std::vector<int> vec;
						for (int k = 0; k < _nb_resources; ++k)
							vec.push_back(_activity_resource_requirements[act][k]);
						resource_usage_CP.push_back(vec);
					}
				}
			}

			std::vector<NC_activity> NC_activities;
			{
				// update PS'
				std::vector<int> PS_prime = _branching_tree.back().partial_schedule;
				for (auto&& actdel : node.delayed_activities)
				{
					for (int index = PS_prime.size() - 1; index >= 0; --index)
					{
						if (PS_prime[index] == actdel)
							PS_prime.erase(PS_prime.begin() + index);
					}
				}
				_output << "\nWe calculate PS' = "; print_activity_set(PS_prime);

				// Calculate set NC
				for (int i = 0; i < _nb_activities; ++i)
				{
					bool in_PS_or_CP = false;
					for (auto&& actps : PS_prime)
					{
						if (i == actps)
						{
							in_PS_or_CP = true;
							break;
						}
					}
					for (auto&& actcp : critical_path)
					{
						if (i == actcp)
						{
							in_PS_or_CP = true;
							break;
						}
					}
					if (!in_PS_or_CP)
					{
						NC_activities.push_back(NC_activity());
						NC_activities.back().act = i;
					}
				}
				_output << " and NC = {";
				for (int l = 0; l < NC_activities.size(); ++l)
				{
					_output << NC_activities[l].act + 1;
					if (l < NC_activities.size() - 1)
						_output << ",";
				}
				_output << "}";


				// Calculate EST and LFT for every activity in NC, then calculate e
				_output << "\nFor every activity in NC, we calculate EST, LFT, and e";
				for (auto&& actNC : NC_activities)
				{
					_output << "\nActivity " << actNC.act + 1;

					// Set d
					actNC.d = _activity_durations[actNC.act];

					// Calculate EST 
					actNC.EST = calculate_EST(actNC.act);
					_output << ", EST = " << actNC.EST;

					// Calculate LFT
					actNC.LFT = calculate_LFT(actNC.act, CP_length);
					_output << ", LFT = " << actNC.LFT;
					_output << ", d = " << actNC.d;

					// Calculate e
					actNC.e = 0;
					for (int t0 = actNC.EST; t0 <= actNC.LFT - actNC.d; ++t0)
					{
						int continous_periods = 0;
						for (int tcur = t0; tcur < t0 + actNC.d; ++tcur)
						{
							bool feasible = true;
							for (int k = 0; k < _nb_resources; ++k)
							{
								if (resource_usage_CP[tcur][k] + _activity_resource_requirements[actNC.act][k] > _resource_availabilities[k])
								{
									feasible = false;
									break;
								}
							}
							if (feasible)
								++continous_periods;
							else
								break;
						}
						if (continous_periods > actNC.e)
							actNC.e = continous_periods;
					}
					_output << ", e = " << actNC.e;

					// Update LB_C
					if (CP_length + actNC.d - actNC.e > LB_C)
						LB_C = CP_length + actNC.d - actNC.e;
				}
				_output << "\nLB = max{";
				for (int g = 0; g < NC_activities.size(); ++g)
				{
					auto& actNC = NC_activities[g];
					_output << CP_length << "+" << actNC.d << "-" << actNC.e;
					if (g < NC_activities.size() - 1)
						_output << ", ";
				}
				_output << "} = " << LB_C;
			}

			// Delete temporarily added precedence relations again
			for (auto&& delact : node.delayed_activities)
			{
				_current_precedence_relations[node.earliest_finishing_activity_not_delayed][delact] = 0;
				_current_precedence_relations[delact][node.earliest_finishing_activity_not_delayed] = 0;
			}
		}


		// Take maximum of the three values
		node.lower_bound = LB_A;
		if (LB_B > LB_A) node.lower_bound = LB_B;
		if (LB_C > LB_B) node.lower_bound = LB_C;
	}

	bool DH::check_cutset_dominated()
	{
		for (auto&& cs : _saved_cutsets)
		{
			if (cs.node != _current_cutset.node
				&& cs.node != _current_cutset.parent_node // different path in the tree!
				&& cs.unassigned_activities == _current_cutset.unassigned_activities
				&& cs.decision_point <= _current_cutset.decision_point 
				&& cs.latest_finish_time_active_activities <= std::max(_current_cutset.decision_point, _current_cutset.latest_finish_time_active_activities)) 
			{
				_output << "\n\nThe current cutset is dominated by a cutset saved earlier!";
				_output << "\nCutset saved earlier: ";
				print_cutset(cs);
				_output << "\nCurrent cutset: ";
				print_cutset(_current_cutset);

				++_nodes_cutset_dominated;

				for (auto&& sn : _saved_branching_tree)
				{
					if (sn.number == _current_delaying_alternative.number)
					{
						sn.cutset_dominated = true;
						break;
					}
				}

				return true;
			}
		}
		return false;
	}

	bool DH::check_leftshift_dominated()
	{
		// If not empty
		if (!_set_DS.empty())
		{
			_output << "\n\nThe set DS = ";
			print_activity_set(_set_DS);
			_output << " is not empty. We check the left-shift dominance rule. ";

			for (auto&& act : _active_activities)
			{
				// Only if the activity is started at this decision point
				if (_activity_finish_times[act] - _activity_durations[act] == _decision_point)
				{
					// calculate earliest start time
					int EST = 0;
					for (int pred = 0; pred < _nb_activities; ++pred)
					{
						if (_precedence_relations[pred][act] == 1) // ORIGINAL PRECEDENCES!
						{
							if (_activity_finish_times[pred] > EST)
								EST = _activity_finish_times[pred];
						}
					}

					// check if we can shift act one period left
					int time_period = _decision_point - 1;
					if (EST <= time_period)
					{
						bool feasible = true;
						for (int k = 0; k < _nb_resources; ++k)
						{
							int resources_used = 0;
							for (int i = 0; i < _nb_activities; ++i)
							{
								if (_activity_finish_times[i] > time_period
									&& _activity_finish_times[i] - _activity_durations[i] <= time_period)
									resources_used += _activity_resource_requirements[i][k];
							}
							resources_used += _activity_resource_requirements[act][k]; // activity that is left-shifted

							if (resources_used > _resource_availabilities[k])
							{
								feasible = false;
								break;
							}
						}

						// if we can left-shift, schedule is dominated
						if (feasible)
						{
							_output << "\nActivity " << act + 1 << " can be left-shifted, so the current schedule is dominated";
							++_nodes_leftshift_dominated;

							for (auto&& sn : _saved_branching_tree)
							{
								if (sn.number == _current_delaying_alternative.number)
								{
									sn.left_shift_dominated = true;
									break;
								}
							}

							return true;
						}
					}
				}
			}

			_output << "The schedule is not dominated.";
		}

		return false;
	}

	bool DH::transitive_precedence_between_activities_forward(int start, int target, int act_recursion)
	{
		bool exists = false;

		// end
		if (act_recursion == target)
			return true;	// target reached => path exists
		else if (act_recursion == 0 || act_recursion == _nb_activities - 1)
			return false;	// dummy start or end reached => no path exists

		// look forward
		for (int j = 0; j < _nb_activities; ++j)
		{
			if (_precedence_relations[act_recursion][j] == 1)	// based on original precedence relations
			{
				if (transitive_precedence_between_activities_forward(start, target, j))
					exists = true;
			}
		}

		return exists;
	}

	bool DH::transitive_precedence_between_activities_backward(int start, int target, int act_recursion)
	{
		bool exists = false;

		// end
		if (act_recursion == target)
			return true;	// target reached => path exists
		else if (act_recursion == 0 || act_recursion == _nb_activities - 1)
			return false;	// dummy start or end reached => no path exists

		// look backward
		for (int j = 0; j < _nb_activities; ++j)
		{
			if (_precedence_relations[j][act_recursion] == 1) 	// based on original precedence relations
			{
				if (transitive_precedence_between_activities_backward(start, target, j))
					exists = true;
			}
		}

		return exists;
	}





	/**************************************************************************
	*                        The 7 steps of the procedure
	**************************************************************************/

	void DH::initialization()
	{
		// memory
		_RCPL.reserve(_nb_activities);
		_partial_schedule.reserve(_nb_activities);
		_active_activities.reserve(_nb_activities);
		_eligible_activities.reserve(_nb_activities);
		_activity_finish_times.reserve(_nb_activities);
		_resources_to_release.reserve(_nb_resources);

		// precedences
		_current_precedence_relations = _precedence_relations;

		// UB, p, and m
		_upper_bound = _LARGE_VALUE;
		_level_tree = 0;
		_decision_point = 0;

		// activity finish times
		for (int i = 0; i < _nb_activities; ++i)
			_activity_finish_times.push_back(_LARGE_VALUE);
		_activity_finish_times[0] = 0; // dummy start

		// partial schedule
		_partial_schedule.push_back(0);

		// active activities
		_active_activities.push_back(0);

		// cutset activities
		_current_cutset.decision_point = 0;
		_current_cutset.latest_finish_time_active_activities = 0;
		_current_cutset.level_tree = 0;
		_current_cutset.node = 0;
		_current_cutset.parent_node = -1;
		for (int i = 1; i < _nb_activities; ++i)
			if (_current_precedence_relations[0][i] == 1)
				_current_cutset.unassigned_activities.push_back(i);

		// remaining critical path lengths and initial LB
		_output << "\nStep 1: Initialization\n\nWe first calculate the RCPL for every activity.\n(i,RCPL(i)) = {";
		for (int i = 0; i < _nb_activities; ++i)
			_RCPL.push_back(calculate_RCPL(i));
		_current_LB = _RCPL[0];
		for (int i = 0; i < _nb_activities; ++i) {
			_output << "(" << i + 1 << "," << _RCPL[i] << ")";
			if (i < _nb_activities - 1) _output << ",";
		} _output << "}\n\nWe also calculate the critical sequence bound for the root node:";
		if(_verbose) // only use when showing output to show how calculation is done
		{
			_branching_tree.push_back(Information_Level_Tree());
			_branching_tree.back().partial_schedule = _partial_schedule;
			_branching_tree.back().active_activities = _active_activities;
			_branching_tree.back().decision_point = 0;
			_branching_tree.back().activity_finish_times = _activity_finish_times;
			Delaying_Alternative da;
			calculate_LB_node(da);
			_current_LB = da.lower_bound;
		}

		// resources to release
		for (int k = 0; k < _nb_resources; ++k)
			_resources_to_release.push_back(0);

		// start node
		_saved_branching_tree.push_back(Node());
		_saved_branching_tree.back().level_tree = 0;
		_saved_branching_tree.back().number = 0;
		_saved_branching_tree.back().parent_node = -1;
		_saved_branching_tree.back().earliest_finishing_activity = -1;
		_saved_branching_tree.back().lower_bound = _current_LB;

		// Output screen
		_output << "\n\nWe can now initialize the algorithm: \nupper bound: T = " << _LARGE_VALUE 
			<< "\nlevel tree: p = 0 \ndecision point: m = 0 \nLB(0) = " << _current_LB 
			<< "\npartial schedule: PS = {1} \nactive activities: S = {1} \nThe activity finish times: ";
		for (int i = 0; i < _nb_activities; ++i) _output << "t(" << i + 1 << ") = " << _activity_finish_times[i] << "  ";
	}

	bool DH::incrementation()	// Returns true if we need to backtrack, false otherwise
	{
		while (true)
		{
			_output << "\n\n\n\n\nStep 2: Incrementation";

			// Compute next decision point
			_decision_point = _LARGE_VALUE;
			for (auto&& act : _active_activities)
			{
				if (_activity_finish_times[act] < _decision_point)
					_decision_point = _activity_finish_times[act];
			}
			_output << "\nThe next decision point is m = " << _decision_point;

			// Update active activities
			for (int i = _nb_activities - 1; i >= 0; --i)
			{
				if (_activity_finish_times[i] == _decision_point)
				{
					auto it = std::find(_active_activities.begin(), _active_activities.end(), i);
					if (it != _active_activities.end())
						_active_activities.erase(it);
				}
			}
			_output << "\nThe active activities are now: S = ";
			print_activity_set(_active_activities);

			// Check if schedule complete
			if (_activity_finish_times.back() < _LARGE_VALUE)
			{
				_output << "\nComplete schedule found! T = " << _activity_finish_times.back();

				for (auto&& sn : _saved_branching_tree)
				{
					if (sn.number == _current_delaying_alternative.number)
					{
						sn.schedule_found = true;
						sn.schedule_length = _activity_finish_times.back();
						break;
					}
				}

				if (_activity_finish_times.back() < _upper_bound)
				{
					_output << "\nUpper bound improved!";
					_optimal_activity_finish_times = _activity_finish_times;
					_upper_bound = _activity_finish_times.back();

					_output << "\nThe activity finish times: ";
					for (int i = 0; i < _nb_activities; ++i) _output << "t(" << i + 1 << ") = " << _activity_finish_times[i] << "  ";
				}

				return true; // backtrack
			}

			// Check if cutset dominated
			if (check_cutset_dominated())
				return true; // backtrack

			// Save new cutset
			if (std::find(_saved_cutsets.begin(), _saved_cutsets.end(), _current_cutset) == _saved_cutsets.end())
				_saved_cutsets.push_back(_current_cutset);
			_output << "\nThe current cutset is *not* dominated by a cutset saved earlier: save the cutset";
			_output << "\nSaved cutsets: \n\tnode\tparent\tm\tLFT S\tC";
			for (auto&& set : _saved_cutsets)
				print_cutset(set);

			// Determine eligible activities
			_eligible_activities.clear();
			for (int i = 0; i < _nb_activities; ++i)
			{
				bool eligible = true;
				for (int pred = 0; pred < _nb_activities; ++pred)
				{ // all predecessors finished
					if (_current_precedence_relations[pred][i] == 1
						&& _activity_finish_times[pred] > _decision_point)
					{
						eligible = false;
						break;
					}
				}
				for (auto&& actps : _partial_schedule)
				{ // not yet in PS
					if (i == actps)
					{
						eligible = false;
						break;
					}
				}
				if (eligible)
					_eligible_activities.push_back(i);
			}
			_output << "\nEligible activities: E = ";
			print_activity_set(_eligible_activities);

			// If no eligible activities, continue with this step; otherwise, exit
			if (_eligible_activities.size() > 0)
				return false; // no backtrack
		}
	}

	bool DH::separation()	// Returns true if theorems 3 or 4 applied, false otherwise
	{
		_output << "\n\n\n\n\nStep 3: Separation";

		// Only if S is empty
		if (!_active_activities.empty())
		{
			_output << "\nThere are still activities in progress. Theorems 3 and 4 do not apply.";
			return false;
		}

		// For every eligible activity, check with how many unscheduled activities (not necessarily eligible) it can be scheduled
		for (auto&& i : _eligible_activities)
		{
			std::list<int> other_act;
			for (int j = 0; j < _nb_activities; ++j)
			{
				if (j != i)
				{
					bool unassigned = true;
					for (auto&& ps : _partial_schedule)
					{
						if (j == ps)
						{
							unassigned = false;
							break;
						}
					}

					bool can_be_parallel = false;
					if (!transitive_precedence_between_activities_forward(i, j, i)
						&& !transitive_precedence_between_activities_backward(i, j, i))
						can_be_parallel = true;

					if (unassigned && can_be_parallel)
					{
						bool feasible = true;
						for (int k = 0; k < _nb_resources; ++k)
						{
							if (_activity_resource_requirements[i][k] + _activity_resource_requirements[j][k] > _resource_availabilities[k])
							{
								feasible = false;
								break;
							}
						}
						if (feasible)
							other_act.push_back(j);
					}
				}
			}

			// Theorem 3
			if (other_act.size() == 0)
			{
				// Schedule the activity
				_partial_schedule.push_back(i);
				_active_activities.push_back(i);
				_activity_finish_times[i] = _decision_point + _activity_durations[i];

				// Add precedence relation to delayed activities
				for (auto&& j : _eligible_activities)
				{
					if (j != i)
					{
						_current_precedence_relations[i][j] = 1;
					}
				}

				_output << "\nNo other unscheduled activity can be scheduled together with activity " << i + 1;
				_output << "\nTheorem 3 applies: schedule activity " << i + 1;
				_output << "\nPS = "; print_activity_set(_partial_schedule);
				_output << "\nS = "; print_activity_set(_active_activities);
				_output << "\n"; for (int c = 0; c < _nb_activities; ++c) _output << "t(" << c + 1 << ") = " << _activity_finish_times[c] << "  ";

				// Update cutset 
				{
					_current_cutset.decision_point = _decision_point;
					_current_cutset.level_tree = _level_tree;
					_current_cutset.node = _current_delaying_alternative.number;
					_current_cutset.latest_finish_time_active_activities = 0;
					for (auto&& act : _active_activities)
						if (_activity_finish_times[act] > _current_cutset.latest_finish_time_active_activities)
							_current_cutset.latest_finish_time_active_activities = _activity_finish_times[act];
					_current_cutset.unassigned_activities.clear();
					for (int c = 0; c < _nb_activities; ++c)
					{
						if (std::find(_partial_schedule.begin(), _partial_schedule.end(), c) != _partial_schedule.end())
							continue;	// not itself in PS

						bool all_pred_in_PS = true;
						for (int pred = 0; pred < _nb_activities; ++pred)
						{
							if (_current_precedence_relations[pred][c] == 1)
							{
								if (std::find(_partial_schedule.begin(), _partial_schedule.end(), pred) == _partial_schedule.end())
								{
									all_pred_in_PS = false;
									break;
								}
							}
						}

						if (all_pred_in_PS)
							_current_cutset.unassigned_activities.push_back(c);
					}
					_output << "\nUpdate cutset: C = "; print_activity_set(_current_cutset.unassigned_activities);
				}

				++_nb_times_theorem3_applied;
				return true;
			}

			// Theorem 4
			else if (other_act.size() == 1)
			{
				int act = other_act.front();
				bool eligible = true;

				// Check if this other activity is eligible!
				for (int pred = 0; pred < _nb_activities; ++pred)
				{
					if (_current_precedence_relations[pred][act] == 1)
					{
						if (_activity_finish_times[pred] > _decision_point)
						{
							eligible = false;
							break;
						}
					}
				}

				if (eligible &&
					_activity_durations[act] <= _activity_durations[i])
				{
					// Schedule both activities
					_partial_schedule.push_back(i);
					_active_activities.push_back(i);
					_activity_finish_times[i] = _decision_point + _activity_durations[i];
					_partial_schedule.push_back(act);
					_active_activities.push_back(act);
					_activity_finish_times[act] = _decision_point + _activity_durations[act];

					_output << "\nActivity " << i + 1 << " can only be scheduled together with unassigned activity " << act + 1 << " which has a shorter duration";
					_output << "\nTheorem 4 applies: schedule activities " << i + 1 << " and " << act + 1;
					_output << "\nPS = "; print_activity_set(_partial_schedule);
					_output << "\nS = "; print_activity_set(_active_activities);
					_output << "\n"; for (int c = 0; c < _nb_activities; ++c) _output << "t(" << c + 1 << ") = " << _activity_finish_times[c] << "  ";

					// Remove shortest activity so that incrementation step goes to other activity finish time immediately
					auto itt = std::find(_active_activities.begin(), _active_activities.end(), act);
					_active_activities.erase(itt);

					// Add precedence relation to delayed activities (from longest activity i)
					for (auto&& j : _eligible_activities)
					{
						if (j != i && j != act)
						{
							_current_precedence_relations[i][j] = 1;
						}
					}

					// Update cutset 
					{
						_current_cutset.decision_point = _decision_point;
						_current_cutset.level_tree = _level_tree;
						_current_cutset.node = _current_delaying_alternative.number;
						_current_cutset.latest_finish_time_active_activities = 0;
						for (auto&& act : _active_activities)
							if (_activity_finish_times[act] > _current_cutset.latest_finish_time_active_activities)
								_current_cutset.latest_finish_time_active_activities = _activity_finish_times[act];
						_current_cutset.unassigned_activities.clear();
						for (int c = 0; c < _nb_activities; ++c)
						{
							if (std::find(_partial_schedule.begin(), _partial_schedule.end(), c) != _partial_schedule.end())
								continue;	// not itself in PS

							bool all_pred_in_PS = true;
							for (int pred = 0; pred < _nb_activities; ++pred)
							{
								if (_current_precedence_relations[pred][c] == 1)
								{
									if (std::find(_partial_schedule.begin(), _partial_schedule.end(), pred) == _partial_schedule.end())
									{
										all_pred_in_PS = false;
										break;
									}
								}
							}

							if (all_pred_in_PS)
								_current_cutset.unassigned_activities.push_back(c);
						}
						_output << "\nUpdate cutset: "; print_cutset(_current_cutset);
					}

					++_nb_times_theorem4_applied;
					return true;
				}
			}
		}


		_output << "\nAll eligible activities can be scheduled with at least two other unassigned activities. Theorems 3 and 4 are not applicable.";
		return false;
	}

	bool DH::scheduling()	// Returns true if resource conflict, false otherwise
	{
		_output << "\n\n\n\n\nStep 4: Scheduling";

		// Temporarily schedule eligible activities
		for (auto&& act : _eligible_activities)
		{
			_partial_schedule.push_back(act);
			_active_activities.push_back(act);
			_activity_finish_times[act] = _decision_point + _activity_durations[act];
		}
		_output << "\nTemporarily schedule eligible activities";
		_output << "\nPS = "; print_activity_set(_partial_schedule);
		_output << "\nS = "; print_activity_set(_active_activities);
		_output << "\n"; for (int i = 0; i < _nb_activities; ++i) _output << "t(" << i + 1 << ") = " << _activity_finish_times[i] << "  ";

		// Update cutset 
		_current_cutset.decision_point = _decision_point;
		_current_cutset.level_tree = _level_tree;
		_current_cutset.node = _current_delaying_alternative.number;
		_current_cutset.latest_finish_time_active_activities = 0;
		for (auto&& act : _active_activities)
			if (_activity_finish_times[act] > _current_cutset.latest_finish_time_active_activities)
				_current_cutset.latest_finish_time_active_activities = _activity_finish_times[act];
		_current_cutset.unassigned_activities.clear();
		for (int i = 0; i < _nb_activities; ++i)
		{
			if (std::find(_partial_schedule.begin(), _partial_schedule.end(), i) != _partial_schedule.end())
				continue;	// not itself in PS

			bool all_pred_in_PS = true;
			for (int pred = 0; pred < _nb_activities; ++pred)
			{
				if (_current_precedence_relations[pred][i] == 1)
				{
					if (std::find(_partial_schedule.begin(), _partial_schedule.end(), pred) == _partial_schedule.end())
					{
						all_pred_in_PS = false;
						break;
					}
				}
			}

			if (all_pred_in_PS)
				_current_cutset.unassigned_activities.push_back(i);
		}
		_output << "\nUpdate cutset: C = "; print_activity_set(_current_cutset.unassigned_activities);

		// Check for resource conflict
		bool resource_conflict = false;
		for (int k = 0; k < _nb_resources; ++k)
		{
			int used = 0;
			for (auto&& act : _active_activities)
				used += _activity_resource_requirements[act][k];

			_resources_to_release[k] = 0; // reset
			if (used - _resource_availabilities[k] > 0)
			{
				_resources_to_release[k] = used - _resource_availabilities[k];
				resource_conflict = true;
			}
		}

		return resource_conflict;
	}

	bool DH::resolve_resource_conflict()		// Returns true if we need to backtrack, false otherwise
	{
		_output << "\n\n\n\n\nStep 5: Resource conflict";

		// Update level of search tree
		++_level_tree;
		_output << "\nThere is a resource conflict! We branch one level further: p = " << _level_tree;

		// Save information of the current level of the tree
		_current_cutset.parent_node = _current_delaying_alternative.number; // first save current node, then go to next node
		_branching_tree.push_back(Information_Level_Tree());
		_branching_tree.back().decision_point = _decision_point;
		_branching_tree.back().active_activities = _active_activities;
		_branching_tree.back().partial_schedule = _partial_schedule;
		_branching_tree.back().activity_finish_times = _activity_finish_times;
		_branching_tree.back().current_cutset = _current_cutset;


		// Now compute the delaying alternatives (== nodes)
		{
			std::vector<int> vec;
			compute_minimal_delaying_alternatives(0, vec);
		}
		_output << "\n\nDelaying alternatives D = {";
		for (int i = 0; i < _branching_tree.back().nodes.size(); ++i)
		{
			print_activity_set(_branching_tree.back().nodes[i].delayed_activities);
			if (i < _branching_tree.back().nodes.size() - 1)
				_output << ",";
		}
		_output << "}";

		// For every delaying alternative, calculate LB
		for (auto&& node : _branching_tree.back().nodes)
		{
			_output << "\n\n\nWe calculate the lower bound for delaying alternative ";
			print_activity_set(node.delayed_activities);
			calculate_LB_node(node);
		}

		// Store every node
		for (auto&& node : _branching_tree.back().nodes)
		{
			_saved_branching_tree.push_back(Node());
			_saved_branching_tree.back().level_tree = _level_tree;
			_saved_branching_tree.back().parent_node = _current_delaying_alternative.number;
			_saved_branching_tree.back().number = node.number;
			_saved_branching_tree.back().earliest_finishing_activity = node.earliest_finishing_activity_not_delayed;
			_saved_branching_tree.back().delay_set = node.delayed_activities;
			_saved_branching_tree.back().lower_bound = node.lower_bound;
		}

		// Find next node
		_output << "\n\nWe find the next node with best LB at this level of the tree";
		int best_index = -1;
		int best_LB = _LARGE_VALUE;
		for (int i = 0; i < _branching_tree.back().nodes.size(); ++i)
		{
			auto& node = _branching_tree.back().nodes[i];
			if (node.lower_bound >= _upper_bound)
			{
				for (auto&& sn : _saved_branching_tree)
				{
					if (sn.number == node.number)
					{
						sn.lower_bound_dominated = true;
						break;
					}
				}
			}

			else if (node.lower_bound < best_LB)
			{
				best_index = i;
				best_LB = node.lower_bound;
			}
		}

		// Check if node found
		if (best_index >= 0)
		{
			_current_delaying_alternative = _branching_tree.back().nodes[best_index];
			_current_LB = _current_delaying_alternative.lower_bound;
			_branching_tree.back().nodes[best_index].examined = true;
			return false; // no backtrack needed
		}


		_output << "\nNo eligible node exists: backtrack";
		return true; // backtrack
	}

	bool DH::delay()		// Returns true if we need to backtrack, false otherwise
	{
		_output << "\n\n\n\n\nStep 6: Delay";

		// Determine the set DS of activities that were started earlier than m and are now delayed
		_set_DS.clear();
		for (auto&& act : _partial_schedule)
			if (_activity_finish_times[act] - _activity_durations[act] < _decision_point
				&& std::find(_current_delaying_alternative.delayed_activities.begin(), _current_delaying_alternative.delayed_activities.end(), act) != _current_delaying_alternative.delayed_activities.end())
				_set_DS.push_back(act);

		// Update precedence constraints
		_output << "\nAdd new precedence constraints: ";
		for (auto&& delact : _current_delaying_alternative.delayed_activities)
		{
			_output << "(" << _current_delaying_alternative.earliest_finishing_activity_not_delayed + 1 << "," << delact + 1 << ")  ";
			_current_precedence_relations[_current_delaying_alternative.earliest_finishing_activity_not_delayed][delact] = 1;
			_current_precedence_relations[delact][_current_delaying_alternative.earliest_finishing_activity_not_delayed] = -1;
		}

		// Update info for the delayed activities
		_output << "\nDelay the activities in the delaying alternative";
		for (auto&& delact : _current_delaying_alternative.delayed_activities)
		{
			auto it = std::find(_active_activities.begin(), _active_activities.end(), delact);
			if (it != _active_activities.end())
				_active_activities.erase(it);

			auto it2 = std::find(_partial_schedule.begin(), _partial_schedule.end(), delact);
			if (it2 != _partial_schedule.end())
				_partial_schedule.erase(it2);

			_activity_finish_times[delact] = _LARGE_VALUE;
		}
		_output << "\nPS = "; print_activity_set(_partial_schedule);
		_output << "\nS = "; print_activity_set(_active_activities);
		_output << "\n"; for (int i = 0; i < _nb_activities; ++i) _output << "t(" << i + 1 << ") = " << _activity_finish_times[i] << "  ";

		// Update cutset 
		_current_cutset.decision_point = _decision_point;
		_current_cutset.level_tree = _level_tree;
		_current_cutset.node = _current_delaying_alternative.number;
		_current_cutset.latest_finish_time_active_activities = 0;
		for (auto&& act : _active_activities)
			if (_activity_finish_times[act] > _current_cutset.latest_finish_time_active_activities)
				_current_cutset.latest_finish_time_active_activities = _activity_finish_times[act];
		_current_cutset.unassigned_activities.clear();
		for (int i = 0; i < _nb_activities; ++i)
		{
			if (std::find(_partial_schedule.begin(), _partial_schedule.end(), i) != _partial_schedule.end())
				continue;	// not itself in PS

			bool all_pred_in_PS = true;
			for (int pred = 0; pred < _nb_activities; ++pred)
			{
				if (_current_precedence_relations[pred][i] == 1)
				{
					if (std::find(_partial_schedule.begin(), _partial_schedule.end(), pred) == _partial_schedule.end())
					{
						all_pred_in_PS = false;
						break;
					}
				}
			}

			if (all_pred_in_PS)
				_current_cutset.unassigned_activities.push_back(i);
		}
		_output << "\nUpdate cutset: C = "; print_activity_set(_current_cutset.unassigned_activities);

		// Determine the set DS and apply left-shift dominance rule
		if (check_leftshift_dominated())
			return true; // backtrack

		return false; // do not backtrack
	}

	bool DH::backtrack()		// Returns true if all nodes checked, false otherwise
	{
		// Continue until new node found for exploring, or algorithm is finished
		while (true)
		{
			_output << "\n\n\n\n\nStep 7: Backtrack";

			// Delete newly added precedence relation
			_output << "\nRemove last added precedence relations: ";

			for (int n = 0; n < _branching_tree.back().nodes.size(); ++n)
			{
				auto& node = _branching_tree.back().nodes[n];
				if (node.examined)
				{
					for (auto&& delact : node.delayed_activities)
					{
						_output << "(" << node.earliest_finishing_activity_not_delayed + 1 << "," << delact + 1 << ")  ";
						_current_precedence_relations[node.earliest_finishing_activity_not_delayed][delact] = 0;
						_current_precedence_relations[delact][node.earliest_finishing_activity_not_delayed] = 0;
					}

					// Delete node
					_branching_tree.back().nodes.erase(_branching_tree.back().nodes.begin() + n);
				}

			}

			// Find next node on this level of the tree
			_output << "\nFind next best node on this level of the tree";
			int best_LB = _LARGE_VALUE;
			int index_node = -1;
			for (int i = 0; i < _branching_tree.back().nodes.size(); ++i)
			{
				if (_branching_tree.back().nodes[i].lower_bound >= _upper_bound)
				{
					++_nodes_LB_dominated;

					for (auto&& sn : _saved_branching_tree)
					{
						if (sn.number == _branching_tree.back().nodes[i].number)
						{
							sn.lower_bound_dominated = true;
							break;
						}
					}

				}
				else if (_branching_tree.back().nodes[i].lower_bound < best_LB)
				{
					index_node = i;
					best_LB = _branching_tree.back().nodes[i].lower_bound;
				}
			}

			// If no node found, go level higher
			if (index_node < 0)
			{
				--_level_tree;

				// Delete last level of the tree
				if (!_branching_tree.empty())
				{
					_branching_tree.pop_back();
				}

				_output << "\nNo eligible node: backtrack to level p = " << _level_tree;
			}
			// If node found, restore info
			else
			{
				_branching_tree.back().nodes[index_node].examined = true;
				_current_delaying_alternative = _branching_tree.back().nodes[index_node];
				_activity_finish_times = _branching_tree.back().activity_finish_times;
				_partial_schedule = _branching_tree.back().partial_schedule;
				_active_activities = _branching_tree.back().active_activities;
				_current_cutset = _branching_tree.back().current_cutset;
				_decision_point = _branching_tree.back().decision_point;
				_current_LB = _current_delaying_alternative.lower_bound;

				_output << "\nNode selected: restore the information of that level of the tree (PS, S, t(i), m, C)";
				_output << "\np = " << _level_tree;
				_output << "\nm = " << _decision_point;
				_output << "\nPS = "; print_activity_set(_partial_schedule);
				_output << "\nS = "; print_activity_set(_active_activities);

				return false;
			}

			// If level 0 reached, stop
			if (_level_tree <= 0)
			{
				_output << "\n\nLevel 0 of the tree reached: STOP";
				return true;
			}
		}
	}





	/**************************************************************************
	*                          The branching procedure
	**************************************************************************/

	void DH::print_branching_tree()
	{
		_output << "\n\n\n\n\nBranch-and-bound tree";

		int examined = 0;
		int level = -1;
		while (examined < _saved_branching_tree.size())
		{
			++level;
			_output << "\n\n\nLevel tree: " << level;

			for (auto&& node : _saved_branching_tree)
			{
				if (node.level_tree == level)
				{
					++examined;

					_output << "\n\nNode " << node.number;
					_output << "\nParent node: " << node.parent_node;
					_output << "\nAdded precedence relations: ";
					_output << node.earliest_finishing_activity + 1 << " <. ";
					for (int i = 0; i < node.delay_set.size(); ++i)
					{
						_output << node.delay_set[i] + 1;
						if (i < node.delay_set.size() - 1)
							_output << ",";
					}
					_output << "\nLB = " << node.lower_bound;
					if (node.schedule_found)
						_output << "\nSchedule found with length " << node.schedule_length;
					else if (node.lower_bound_dominated)
						_output << "\nNode LB dominated";
					else if (node.cutset_dominated)
						_output << "\nNode cutset dominated";
					else if (node.left_shift_dominated)
						_output << "\nNode left-shift dominated";
					else
						_output << "\nNode continued";
				}
			}
		}
	}

	void DH::run(bool verbose)
	{
		_verbose = verbose;
		_output.set_on(true);
		_output << "\nStarting branch-and-bound procedure of Demeulemeester-Herroelen ...\n";
		_output.set_on(verbose);

		auto start_time = std::chrono::system_clock::now();


		// Reset all values
		reset_all_variables();


		// Initialize algorithm
		initialization();

		// test
		_current_LB = 67;

		bool need_backtrack = false;
		while (true)
		{
			while (!need_backtrack)
			{
				if (incrementation())
					break;		// backtrack

				if (separation())
					continue;	// go to 'incrementation'

				if (scheduling())	// if resource conflict
				{
					if (resolve_resource_conflict())
						break;	// backtrack

					if (delay())
						break;	// backtrack
				}
			}

			if (backtrack())
				break;	// optimal solution found

			need_backtrack = false;
			if (delay())
				need_backtrack = true;
		}



		// Print statistics
		_output.set_on(true);
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;
		_output << "\n\n\nOptimal solution found with makespan " << _upper_bound;
		_output << "\nActivity finish times: "; for (int i = 0; i < _nb_activities; ++i) _output << "t(" << i + 1 << ") = " << _optimal_activity_finish_times[i] << "  ";
		_output << "\n\nElapsed time (s): " << elapsed_time.count();
		_output << "\nNodes evaluated: " << _nodes_evaluated;
		_output << "\nNodes LB dominated: " << _nodes_LB_dominated;
		_output << "\nNodes cutset dominated: " << _nodes_cutset_dominated;
		_output << "\nNodes left-shift dominated: " << _nodes_leftshift_dominated;
		_output << "\nTheorem 3 applied: " << _nb_times_theorem3_applied;
		_output << "\nTheorem 4 applied: " << _nb_times_theorem4_applied;

		if(verbose)
			print_branching_tree();
	}
}