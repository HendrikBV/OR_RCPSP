#include "algorithms.h"
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <list>



namespace RCPSP
{
	/**************************************************************************
	*                            Print functions
	**************************************************************************/

	void DH::print_node(const Node& node)
	{
		_output << "\nLevel tree: " << node.level_tree
			<< ", LB = " << node.lower_bound
			<< ", finish times: ";
		for (int i = 0; i < _activities.size(); ++i)
			_output << "f(" << i + 1 << ") = " << node.act_finish_time[i] << "   ";
	}


	void DH::print_cutset(const Cutset& cutset)
	{
		_output << "\n" << cutset.node << "\t" << cutset.parent_node << "\t" << cutset.decision_point << "\tS={ ";
		for (int i = 0; i < _activities.size(); ++i)
			if (cutset.act_active[i])
				_output << i + 1 << " ";
		_output << "}, C={ ";
		for (int i = 0; i < _activities.size(); ++i)
			if (cutset.act_unassigned[i])
				_output << i + 1 << " ";
		_output << "}, ";
		for (int i = 0; i < _activities.size(); ++i)
			_output << "f(" << i + 1 << ") = " << cutset.act_finish_time[i] << "   ";
	}





	/**************************************************************************
	*                            Auxiliary functions
	**************************************************************************/

	void DH::clear_all()
	{
		// variables
		_remaining_nodes.clear();
		_saved_cutsets.clear();
		_RCPL.clear();
		_upper_bound = std::numeric_limits<int>::max();
		_best_activity_finish_times.clear();

		// statistics
		_nodes_evaluated = 0;
		_nodes_LB_dominated = 0;
		_nodes_cutset_dominated = 0;
		_nodes_leftshift_dominated = 0;
		_nb_times_theorem3_applied = 0;
		_nb_times_theorem4_applied = 0;
	}


	int DH::calculate_RCPL(int activity)
	{
		if (activity == _activities.size() - 1)
			return 0;	// dummy end activity

		int cp = 0;
		auto& act = _activities[activity];

		for (auto&& suc : act.successors)
		{
			int val = act.duration + calculate_RCPL(suc);
			if (val > cp)
				cp = val;
		}
		return cp;
	}


	bool DH::transitive_precedence_between_activities_forward(int start, int target, int act_recursion)
	{
		bool exists = false;

		// end
		if (act_recursion == target)
			return true;	// target reached => path exists
		else if (act_recursion == 0 || act_recursion == _activities.size() - 1)
			return false;	// dummy start or end reached => no path exists

		// look forward
		for (auto&& suc : _activities[act_recursion].successors)
		{
			if (transitive_precedence_between_activities_forward(start, target, suc))
				exists = true;
		}

		return exists;
	}


	bool DH::transitive_precedence_between_activities_backward(int start, int target, int act_recursion)
	{
		bool exists = false;

		// end
		if (act_recursion == target)
			return true;	// target reached => path exists
		else if (act_recursion == 0 || act_recursion == _activities.size() - 1)
			return false;	// dummy start or end reached => no path exists

		// look backward
		for (auto&& pred : _activities[act_recursion].predecessors)
		{
			if (transitive_precedence_between_activities_backward(start, target, pred))
				exists = true;
		}

		return exists;
	}


	std::vector<std::vector<int>> DH::find_min_delaying_sets(const Node& node, const std::vector<int>& res_to_release)
	{
		std::vector<std::vector<int>> result;

		// activities for which to calculate delaying sets
		std::vector<int> acts;
		for (int i = 0; i < _activities.size(); ++i) {
			if (node.act_active[i]) {
				acts.push_back(i);
			}
		}


		// Iterate over all possible subsets using bitmasks
		for (int mask = 0; mask < (1 << acts.size()); mask++)
		{
			std::vector<int> currentset;  // Current subset (indices)
			std::vector<int> currentsum;  // Sum of weights in current subset
			for (int k = 0; k < _resource_availabilities.size(); ++k)
				currentsum.push_back(0);

			// Build the subset and compute its sum
			for (int i = 0; i < acts.size(); i++)
			{
				if (mask & (1 << i))
				{  // If bit i is set, include item i
					currentset.push_back(acts[i]);
					for (int k = 0; k < _resource_availabilities.size(); ++k)
						currentsum[k] += _activities[acts[i]].resource_requirements[k];
				}
			}

			// Check if sum is at least W
			bool sufficient_resources = true;
			for (int k = 0; k < _resource_availabilities.size(); ++k) {
				if (currentsum[k] < res_to_release[k]) {
					sufficient_resources = false;
					break;
				}
			}

			if (sufficient_resources)
			{
				bool minimal = true;

				// Check minimality: removing any item should make sum < W
				for (int idx : currentset) // idx is element from set acts 
				{
					bool enough_after_del_act = true;
					for (int k = 0; k < _resource_availabilities.size(); ++k)
					{
						if (currentsum[k] - _activities[idx].resource_requirements[k] < res_to_release[k])
						{
							enough_after_del_act = false;
						}
					}
					if (enough_after_del_act)
					{
						minimal = false;
						break;
					}
				}

				// If subset is minimal, output its indices
				if (minimal)
				{
					result.push_back(currentset);
					/*for (int i = 0; i < currentset.size(); i++)
					{
						result.push_back(currentset);
					}*/
				}
			}
		}


		return result;
	}





	/**************************************************************************
	*                            Main algorithm
	**************************************************************************/

	void DH::procedure()
	{
		// INITIALIZE	 
		// Compute remaining critical path length for each activity
		{
			_output << "\n\nCompute RCPL for every activity\n";
			_RCPL.reserve(_activities.size());
			for (int i = 0; i < _activities.size(); ++i)
			{
				int RCPL = calculate_RCPL(i);
				_RCPL.push_back(RCPL);

				_output << "RCPL[" << i + 1 << "] = " << RCPL << "\t";
			}
		}

		// Create root node
		_remaining_nodes.push_back(Node());
		{
			_output << "\n\nCreate root node";
			++_nodes_evaluated;

			Node& root_node = _remaining_nodes.back();
			root_node.id = _nodes_evaluated;
			root_node.level_tree = 0;
			root_node.lower_bound = _RCPL[0];

			// set initial values of all activities
			for (int i = 0; i < _activities.size(); ++i)
			{
				root_node.act_active.push_back(false);
				root_node.act_finish_time.push_back(std::numeric_limits<int>::max());
				root_node.act_in_PS.push_back(false);
				root_node.act_eligible.push_back(false);
			}

			// schedule dummy start activity
			root_node.act_active[0] = true;
			root_node.act_finish_time[0] = 0;
			root_node.act_in_PS[0] = true;
			root_node.decision_point = 0;

			// update cutset
			root_node.cutset.decision_point = root_node.decision_point;
			root_node.cutset.node = root_node.id;
			root_node.cutset.parent_node = -1;
			root_node.cutset.act_active = root_node.act_active;
			root_node.cutset.act_finish_time = root_node.act_finish_time;
			for (int i = 0; i < _activities.size(); ++i) {
				root_node.cutset.act_unassigned.push_back(false);
			}
			for (auto&& s : _activities[0].successors)
				root_node.cutset.act_unassigned[s] = true; // successors of dummy start			
		}
		if (_verbose) print_node(_remaining_nodes.back());



		// BRANCHING
		// Branching until no nodes left
		while (true)
		{
			// remove all nodes with LB >= UB
			for (int i = _remaining_nodes.size() - 1; i >= 0; --i)
			{
				if (_remaining_nodes[i].lower_bound >= _upper_bound)
				{
					_output << "\n\nNext node on this level is LB dominated:";
					if (_verbose) print_node(_remaining_nodes[i]);


					_remaining_nodes.erase(_remaining_nodes.begin() + i);
					++_nodes_LB_dominated;
				}
			}

			if (_remaining_nodes.empty())
			{
				_output << "\n\nNo nodes left: STOP";
				break; // done
			}


			// find node on current level of tree with best bound
			Node current_node;
			{
				// find deepest level
				int level_tree = -1;
				for (auto&& node : _remaining_nodes) {
					if (node.level_tree > level_tree) {
						level_tree = node.level_tree;
					}
				}

				// find best LB on this level
				int index_node = -1;
				int best_LB = std::numeric_limits<int>::max();
				for (auto i = 0; i < _remaining_nodes.size(); ++i) {
					if (_remaining_nodes[i].level_tree == level_tree && _remaining_nodes[i].lower_bound < best_LB) {
						best_LB = _remaining_nodes[i].lower_bound;
						index_node = i;
					}
				}

				// node should have been found
				if (index_node < 0) throw std::logic_error("No node was found, but there are still remaining nodes");
				current_node = _remaining_nodes[index_node];
				_remaining_nodes.erase(_remaining_nodes.begin() + index_node);
			}
			_output << "\n\nContinuing with best node on current level of tree";
			if (_verbose) print_node(current_node);


			// keep scheduling until resource conflict
			while (true)
			{
				// determine decision point
				current_node.decision_point = std::numeric_limits<int>::max();
				for (int i = 0; i < _activities.size(); ++i)
				{
					if (current_node.act_active[i] && current_node.act_finish_time[i] < current_node.decision_point)
					{
						current_node.decision_point = current_node.act_finish_time[i];
					}
				}
				_output << "\nGo to decision point: " << current_node.decision_point;

				// set activities that have been completed to inactive
				for (int i = 0; i < _activities.size(); ++i)
				{
					if (current_node.act_active[i] && current_node.act_finish_time[i] <= current_node.decision_point)
					{
						current_node.act_active[i] = false;
					}
				}

				// check if dummy finish activity has been scheduled
				// if so, complete schedule has been found: backtrack
				{
					int dummy_finish_index = _activities.size() - 1;
					if (current_node.act_in_PS[dummy_finish_index] && !current_node.act_active[dummy_finish_index])
					{
						_output << "\n\nDummy finish activity scheduled. Complete schedule found.";

						// check if better solution
						if (current_node.act_finish_time[dummy_finish_index] < _upper_bound)
						{
							_upper_bound = current_node.act_finish_time[dummy_finish_index];
							_best_activity_finish_times = current_node.act_finish_time;

							_output << "\nNew best solution found! T = " << _upper_bound;
						}

						break; // backtrack
					}
				}



				// check cutset dominated
				bool cutset_dominated = false;
				for (auto&& sc : _saved_cutsets)
				{
					if (sc.node != current_node.cutset.node
						&& sc.node != current_node.cutset.parent_node // different path in the tree!
						&& sc.act_unassigned == current_node.cutset.act_unassigned
						&& sc.decision_point <= current_node.cutset.decision_point)
					{
						bool condition = true;
						for (int i = 0; i < _activities.size(); ++i)
						{
							if (sc.act_active[i])
							{
								if (sc.act_finish_time[i] > std::max(current_node.cutset.act_finish_time[i], current_node.cutset.decision_point))
								{
									condition = false;
									break;
								}
							}
						}

						if (condition)
						{
							_output << "\n\nThe current cutset is dominated by a cutset saved earlier!";
							_output << "\nCutset saved earlier: ";
							if (_verbose) print_cutset(sc);
							_output << "\nCurrent cutset: ";
							if (_verbose) print_cutset(current_node.cutset);

							++_nodes_cutset_dominated;
							cutset_dominated = true;
							break;
						}
					}
				}
				if (cutset_dominated)
				{
					break; // backtrack
				}

				// if not dominated, save cutset
				_saved_cutsets.push_back(current_node.cutset);
				_output << "\nCutset not dominated. Save the cutset.";
				if (_verbose) {
					for (auto&& sc : _saved_cutsets) {
						//print_cutset(sc);
					}
				}


				// find eligible activities
				_output << "\nFind eligible activities: ";
				bool eligible_act_exist = false;
				for (int i = 0; i < _activities.size(); ++i)
				{
					bool eligible = false;
					if (!current_node.act_in_PS[i])
					{
						eligible = true;
						for (auto&& pred : _activities[i].predecessors)
						{
							if (!current_node.act_in_PS[pred] || current_node.act_active[pred])
							{
								eligible = false;
								break;
							}
						}

						if (eligible)
						{
							eligible_act_exist = true;
							_output << i + 1 << " ";
						}
					}

					current_node.act_eligible[i] = eligible;
				}

				if (eligible_act_exist)
				{
					// check if still activities in progress
					// if not, apply theorems 3 and 4
					bool activities_in_progress = false;
					for (int i = 0; i < _activities.size(); ++i) {
						if (current_node.act_active[i]) {
							activities_in_progress = true;
							break;
						}
					}

					bool activity_scheduled_theorems34 = false;
					if (!activities_in_progress)
					{
						_output << "\n\nThere are no activities in progress: check if theorems 3 and 4 apply";

						// For every eligible activity, check with how many unscheduled activities (not necessarily eligible) it can be scheduled
						for (int i = 0; i < _activities.size(); ++i)
						{
							if (current_node.act_eligible[i])
							{
								std::list<int> other_act;
								for (int j = 0; j < _activities.size(); ++j)
								{
									if (j != i && !current_node.act_in_PS[j]) // unassigned, so not yet in PS
									{
										if (!transitive_precedence_between_activities_forward(i, j, i)
											&& !transitive_precedence_between_activities_backward(i, j, i)) // can they be scheduled in parallel?
										{
											bool feasible = true;
											for (int k = 0; k < _resource_availabilities.size(); ++k)
											{
												if (_activities[i].resource_requirements[k] + _activities[j].resource_requirements[k]
											> _resource_availabilities[k])
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
									current_node.act_active[i] = true;
									current_node.act_finish_time[i] = current_node.decision_point + _activities[i].duration;
									current_node.act_in_PS[i] = true;
									current_node.act_eligible[i] = false;

									_output << "\nNo other unscheduled activity can be scheduled together with activity " << i + 1;
									_output << "\nTheorem 3 applies: schedule activity " << i + 1;

									// update cutset
									{
										// update cutset
										current_node.cutset.decision_point = current_node.decision_point;
										current_node.cutset.act_active = current_node.act_active;
										current_node.cutset.act_finish_time = current_node.act_finish_time;

										for (int ii = 0; ii < _activities.size(); ++ii) {
											current_node.cutset.act_unassigned[ii] = false; // first reset
											bool all_pred_in_PS = true;
											for (auto&& pred : _activities[ii].predecessors) {
												if (!current_node.act_in_PS[pred]) {
													all_pred_in_PS = false;
													break;
												}
											}
											if (all_pred_in_PS && !current_node.act_in_PS[ii])
												current_node.cutset.act_unassigned[ii] = true;
										}

										_output << "\nUpdate the cutset";
										if (_verbose) print_cutset(current_node.cutset);
									}


									++_nb_times_theorem3_applied;
									activity_scheduled_theorems34 = true;
									break; // exit for loop over activities
								}

								// Theorem 4
								else if (other_act.size() == 1)
								{
									int act = other_act.front();
									bool eligible = true;

									// Check if this other activity is eligible and has a shorter duration
									if (current_node.act_eligible[act] && _activities[act].duration <= _activities[i].duration)
									{
										// Schedule both activities
										current_node.act_active[i] = true;
										current_node.act_finish_time[i] = current_node.decision_point + _activities[i].duration;
										current_node.act_in_PS[i] = true;
										current_node.act_eligible[i] = false; // 
										current_node.act_active[act] = true;
										current_node.act_finish_time[act] = current_node.decision_point + _activities[act].duration;
										current_node.act_in_PS[act] = true;
										current_node.act_eligible[act] = false; // ?

										_output << "\nActivity " << i + 1 << " can only be scheduled together with unassigned activity " << act + 1 << " which has a shorter duration";
										_output << "\nTheorem 4 applies: schedule activities " << i + 1 << " and " << act + 1;

										// Remove shortest activity so that incrementation step goes to other activity finish time immediately
										current_node.act_active[act] = false;

										// update cutset
										{
											// update cutset
											current_node.cutset.decision_point = current_node.decision_point;
											current_node.cutset.act_active = current_node.act_active;
											current_node.cutset.act_finish_time = current_node.act_finish_time;

											for (int ii = 0; ii < _activities.size(); ++ii) {
												current_node.cutset.act_unassigned[ii] = false; // first reset
												bool all_pred_in_PS = true;
												for (auto&& pred : _activities[ii].predecessors) {
													if (!current_node.act_in_PS[pred]) {
														all_pred_in_PS = false;
														break;
													}
												}
												if (all_pred_in_PS && !current_node.act_in_PS[ii])
													current_node.cutset.act_unassigned[ii] = true;
											}

											_output << "\nUpdate the cutset";
											if (_verbose) print_cutset(current_node.cutset);
										}

										++_nb_times_theorem4_applied;
										activity_scheduled_theorems34 = true;
										break; // exit for loop over activities
									}
								} // theorem 4 
							}
						}

					} // applying theorems 3 and 4

					if (!activity_scheduled_theorems34)
					{
						_output << "\n\nAll eligible activities can be scheduled with at least two other unassigned activities."
							"Theorems 3 and 4 are not applicable.\nTemporarily schedule eligible activities";

						// temporarily schedule activities
						for (int i = 0; i < _activities.size(); ++i)
						{
							if (current_node.act_eligible[i])
							{
								current_node.act_active[i] = true;
								current_node.act_finish_time[i] = current_node.decision_point + _activities[i].duration;
								current_node.act_in_PS[i] = true;
								current_node.act_eligible[i] = false; // ?
							}
						}

						// update cutset
						{
							// update cutset
							current_node.cutset.decision_point = current_node.decision_point;
							current_node.cutset.act_active = current_node.act_active;
							current_node.cutset.act_finish_time = current_node.act_finish_time;

							for (int i = 0; i < _activities.size(); ++i) {
								current_node.cutset.act_unassigned[i] = false; // first reset
								bool all_pred_in_PS = true;
								for (auto&& pred : _activities[i].predecessors) {
									if (!current_node.act_in_PS[pred]) {
										all_pred_in_PS = false;
										break;
									}
								}
								if (all_pred_in_PS && !current_node.act_in_PS[i])
									current_node.cutset.act_unassigned[i] = true;
							}
						}
						_output << "\nUpdate the cutset";
						if (_verbose) print_cutset(current_node.cutset);

						// check resource conflict
						std::vector<int> resources_to_release;
						bool resource_conflict = false;
						for (int k = 0; k < _resource_availabilities.size(); ++k)
						{
							int total_use = 0;
							for (int i = 0; i < _activities.size(); ++i)
							{
								if (current_node.act_active[i])
									total_use += _activities[i].resource_requirements[k];
							}

							resources_to_release.push_back(0);
							if (total_use > _resource_availabilities[k])
							{
								resource_conflict = true;
								resources_to_release[k] = total_use - _resource_availabilities[k];
							}
						}

						if (resource_conflict)
						{
							// resolve resource conflict
							std::vector<std::vector<int>> minimal_delaying_sets = find_min_delaying_sets(current_node, resources_to_release);

							if (_verbose)
							{
								_output << "\n\n\nThere is a resource conflict. The minimal delaying sets are {";
								for (auto&& subset : minimal_delaying_sets) {
									_output << " { ";
									for (auto&& elem : subset) {
										_output << elem + 1 << " ";
									}
									_output << "} ";
								}
								_output << "}";
							}

							// for every set, create new node and calculate lower bound
							for (auto&& subset : minimal_delaying_sets)
							{
								if (_verbose)
								{
									_output << "\n\nChecking delaying alternative: ";
									_output << " { ";
									for (auto&& elem : subset) {
										_output << elem + 1 << " ";
									}
									_output << "} ";
								}

								++_nodes_evaluated;

								// copy current node
								_remaining_nodes.push_back(Node(current_node));
								_remaining_nodes.back().id = _nodes_evaluated;
								_remaining_nodes.back().level_tree++; // increase level of tree

								// delay the delayed activity
								for (auto&& actdel : subset)
								{
									_remaining_nodes.back().act_active[actdel] = false;
									_remaining_nodes.back().act_finish_time[actdel] = std::numeric_limits<int>::max();
									_remaining_nodes.back().act_in_PS[actdel] = false;
								}

								// check left-shift dominance
								bool left_shift_dominated = false;
								{
									// Determine the set DS of activities that were started earlier than m and are now delayed
									std::list<int> set_DS;
									for (int ii = 0; ii < _activities.size(); ++ii) {
										if (current_node.act_finish_time[ii] - _activities[ii].duration < current_node.decision_point
											&& std::find(subset.begin(), subset.end(), ii) != subset.end()) {
											set_DS.push_back(ii);
										}
									}

									// If set DS is not empty
									if (!set_DS.empty())
									{
										_output << "\nThe set DS is not empty. We check the left-shift dominance rule.";

										// Check an activity that is started now (not delayed) and could be started earlier
										// without violating the original precedence constraints and resource constraints
										for (int ii = 0; ii < _activities.size(); ++ii)
										{
											if (_remaining_nodes.back().act_active[ii] &&
												_remaining_nodes.back().act_finish_time[ii] - _activities[ii].duration == _remaining_nodes.back().decision_point)
											{
												// calculate earliest start time
												int EST = 0;
												for (auto&& pred : _activities[ii].predecessors) // original precedences
												{
													if (_remaining_nodes.back().act_finish_time[pred] > EST)
														EST = _remaining_nodes.back().act_finish_time[pred];
												}

												// check if we can shift act one period left
												int time_period = _remaining_nodes.back().decision_point - 1;
												if (EST <= time_period)
												{
													bool feasible = true;
													for (int k = 0; k < _resource_availabilities.size(); ++k)
													{
														int resources_used = 0;
														for (int iii = 0; iii < _activities.size(); ++iii)
														{
															if (_remaining_nodes.back().act_finish_time[iii] > time_period
																&& _remaining_nodes.back().act_finish_time[iii] - _activities[iii].duration <= time_period)
																resources_used += _activities[iii].resource_requirements[k];
														}
														resources_used += _activities[ii].resource_requirements[k];

														if (resources_used > _resource_availabilities[k])
														{
															feasible = false;
															break;
														}
													}

													// if we can left-shift, schedule is dominated
													if (feasible)
													{
														_output << "\nActivity " << ii + 1 << " can be left-shifted, so the current schedule is dominated";
														++_nodes_leftshift_dominated;
														left_shift_dominated = true;
														break; // for loop of activities
													}
												}
											}
										}
									}
								} // left-shift dominance 

								if (left_shift_dominated) // delete node
								{
									_remaining_nodes.pop_back();
								}
								else // add node to list of remaining nodes
								{
									_output << "\nThe left-shift dominance rule does not apply.";

									// find earliest finishing activity that is not delayed
									int eft = std::numeric_limits<int>::max();
									int efand = 0; // earliest finishing activity not delayed
									for (int i = 0; i < _activities.size(); ++i)
									{
										if (_remaining_nodes.back().act_active[i] && _remaining_nodes.back().act_finish_time[i] < eft)
										{
											eft = _remaining_nodes.back().act_finish_time[i];
											efand = i;
										}
									}

									// additional precedence relations
									// do we use this?????????
									for (auto&& actdel : subset)
									{
										_remaining_nodes.back().additional_precedences.push_back(std::pair<int, int>());
										_remaining_nodes.back().additional_precedences.back().first = efand;
										_remaining_nodes.back().additional_precedences.back().second = actdel;
									}

									// calculate lower bound (only critical path lower bound is used)
									for (auto&& actdel : subset)
									{
										int LB_RCPL = eft + _RCPL[actdel];
										if (LB_RCPL > _remaining_nodes.back().lower_bound)
											_remaining_nodes.back().lower_bound = LB_RCPL;
									}
									_output << "\nThe new lower bound is LB = " << _remaining_nodes.back().lower_bound;

									// update cutset
									{
										_remaining_nodes.back().cutset.node = _remaining_nodes.back().id;
										_remaining_nodes.back().cutset.parent_node = current_node.id;

										for (auto&& actdel : subset) {
											_remaining_nodes.back().cutset.act_unassigned[actdel] = true;
											for (auto&& suc : _activities[actdel].successors) {
												_remaining_nodes.back().cutset.act_unassigned[suc] = false;
											}
										}

										_remaining_nodes.back().cutset.act_active = current_node.act_active;
										_remaining_nodes.back().cutset.act_finish_time = current_node.act_finish_time;
									}
								}
							}

							break; // stop scheduling and branch into new node

						} // resource conflict
					} // activity scheduled theorems 3 & 4
				} // eligible act exists
			} // scheduling within same node
		} // branching
	}


	void DH::run(bool verbose)
	{
		_verbose = verbose;
		_output.set_on(true);
		_output << "\nStarting branch-and-bound procedure of Demeulemeester and Herroelen ...\n";
		_output.set_on(verbose);

		auto start_time = std::chrono::system_clock::now();

		// Reset all values
		clear_all();

		// Main procedure
		procedure();

		// Print statistics
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;
		_output.set_on(true);
		_output << "\n\n\nOptimal solution found with makespan " << _upper_bound;
		_output << "\nActivity finish times: "; for (int i = 0; i < _activities.size(); ++i) _output << "t(" << i + 1 << ") = " << _best_activity_finish_times[i] << "  ";
		_output << "\n\nElapsed time (s): " << elapsed_time.count();
		_output << "\nNodes evaluated: " << _nodes_evaluated;
		_output << "\nNodes LB dominated: " << _nodes_LB_dominated;
		_output << "\nNodes cutset dominated: " << _nodes_cutset_dominated;
		_output << "\nNodes left-shift dominated: " << _nodes_leftshift_dominated;
		_output << "\nTheorem 3 applied: " << _nb_times_theorem3_applied;
		_output << "\nTheorem 4 applied: " << _nb_times_theorem4_applied;
	}
}