#pragma once
#ifndef ALGORITHMS_RCPSP_H
#define ALGORITHMS_RCPSP_H


#include <exception>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include "ortools/linear_solver/linear_solver.h"



namespace RCPSP // resource-constrained project scheduling problem
{
	class Output
	{
		bool _on = true;

	public:
		void set_on(bool on) { _on = on; }
		friend Output& operator<<(Output& output, const std::string& msg);
		friend Output& operator<<(Output& output, const char* msg);
		friend Output& operator<<(Output& output, long unsigned int value);
		friend Output& operator<<(Output& output, size_t value);
		friend Output& operator<<(Output& output, unsigned int value);
		friend Output& operator<<(Output& output, int value);
		friend Output& operator<<(Output& output, int64_t value);
		friend Output& operator<<(Output& output, float value);
		friend Output& operator<<(Output& output, double value);
	};

	///////////////////////////////////////////////////////////////////////////

	// Base class
	class Algorithm
	{
	protected:
		Output _output;

		int _nb_resources;
		int _nb_activities;

		std::vector<int> _resource_availabilities;
		std::vector<std::vector<int>> _precedence_relations; // If i before j then [i][j] == 1 and [j][i] == -1    
		std::vector<int> _activity_durations;
		std::vector<std::vector<int>> _activity_resource_requirements;

		int precedence(int act_i, int act_j) { return _precedence_relations[act_i][act_j]; }
		int resource_requirement(int activity, int resource) { return _activity_resource_requirements[activity][resource]; }

	public:
		virtual ~Algorithm() {}

		void read_data(const std::string& filename);

		virtual void run(bool verbose) = 0;
	};

	///////////////////////////////////////////////////////////////////////////

	// with variables x[j][t] == 1 if job j starts at time t, 0 otherwise
	class SCIP : public Algorithm
	{
		std::unique_ptr<operations_research::MPSolver> _solver; // OR Tools solver

		/*!
		 *	@brief Which solver is used?
		 *
		 *	solver_id is case insensitive, and the following names are supported:
		 *  - SCIP_MIXED_INTEGER_PROGRAMMING or SCIP
		 *  - CBC_MIXED_INTEGER_PROGRAMMING or CBC
		 *  - CPLEX_MIXED_INTEGER_PROGRAMMING or CPLEX or CPLEX_MIP			(license needed)
		 *  - GUROBI_MIXED_INTEGER_PROGRAMMING or GUROBI or GUROBI_MIP	    (license needed)
		 *  - XPRESS_MIXED_INTEGER_PROGRAMMING or XPRESS or XPRESS_MIP		(license needed)
		 */
		std::string _solver_type = "SCIP";

		void build_problem();
		void solve_problem();

		bool _output_screen = false;
		double _max_computation_time = 600; // seconds

	public:
		void run(bool verbose) override;
		void set_max_time(double time) { _max_computation_time = time; }
	};

	///////////////////////////////////////////////////////////////////////////

	// procedure of Demeulemeester-Herroelen (1992)
	class DH : public Algorithm
	{
		struct Cutset
		{
			int decision_point;
			int level_tree;
			int node;
			int parent_node;
			int latest_finish_time_active_activities;
			std::vector<int> unassigned_activities;

			bool operator==(const Cutset& c)
			{
				if (decision_point == c.decision_point
					&& level_tree == c.level_tree
					&& node == c.node
					&& latest_finish_time_active_activities == c.latest_finish_time_active_activities
					&& unassigned_activities == c.unassigned_activities)
					return true;
				return false;
			}
		};

		struct NC_activity
		{
			int act;
			int EST;
			int LFT;
			int d;
			int e;
		};

		struct Delaying_Alternative
		{
			int number;
			int earliest_finishing_activity_not_delayed;
			std::vector<int> delayed_activities;
			int lower_bound;

			bool examined = false;

			bool operator==(const Delaying_Alternative& n)
			{
				if (number == n.number)
					return true;
				return false;
			}
		};

		struct Information_Level_Tree
		{
			int decision_point;
			int level_tree;
			std::vector<int> partial_schedule;			// at the time of the resource conflict (before delaying)!
			std::vector<int> active_activities;			// at the time of the resource conflict (before delaying)!
			std::vector<int> activity_finish_times;		// at the time of the resource conflict (before delaying)!
			Cutset current_cutset;						// at the time of the resource conflict (before delaying)!
			std::vector<Delaying_Alternative> nodes;	// delaying alternatives at this level of the branch-and-bound tree
		};

		struct Node
		{
			int number;
			int level_tree;
			int parent_node;

			int lower_bound;
			int earliest_finishing_activity;
			std::vector<int> delay_set;

			bool schedule_found = false;
			int schedule_length = -1;
			bool lower_bound_dominated = false;
			bool left_shift_dominated = false;
			bool cutset_dominated = false;
		};


		void print_activity_set(const std::vector<int>& set);
		void print_cutset(const Cutset& c);

		void reset_all_variables();
		int calculate_EST(int activity);
		int calculate_LFT(int activity, int deadline);
		int calculate_RCPL(int activity);
		bool calculate_critical_path(int activity, std::vector<int>& path, int current_length, int& best_length);
		void compute_minimal_delaying_alternatives(int index, std::vector<int> current_delay_set);
		void calculate_LB_node(Delaying_Alternative& node);
		bool check_cutset_dominated();
		bool check_leftshift_dominated();
		bool transitive_precedence_between_activities_forward(int start, int target, int act_recursion);
		bool transitive_precedence_between_activities_backward(int start, int target, int act_recursion);
		
		void initialization();
		bool incrementation();
		bool seperation();
		bool scheduling();
		bool resolve_resource_conflict();
		bool delay();
		bool backtrack();

		void print_branching_tree();

		const int _LARGE_VALUE = std::numeric_limits<int>::max();

		size_t _upper_bound;
		std::vector<int> _optimal_activity_finish_times;
		std::vector<int> _RCPL;
		int _level_tree;
		int _decision_point;
		int _current_LB;

		std::vector<int> _partial_schedule;
		std::vector<int> _active_activities;
		std::vector<int> _activity_finish_times;
		std::vector<int> _eligible_activities;

		std::vector<Information_Level_Tree> _branching_tree;
		Delaying_Alternative _current_delaying_alternative;

		std::vector<Cutset> _saved_cutsets;
		Cutset _current_cutset;

		std::vector<int> _resources_to_release;
		std::vector<int> _set_DS;
		std::vector<std::vector<int>> _current_precedence_relations;

		size_t _nodes_evaluated = 0;
		size_t _nodes_LB_dominated = 0;
		size_t _nodes_cutset_dominated = 0;
		size_t _nodes_leftshift_dominated = 0;
		size_t _nb_times_theorem3_applied = 0;
		size_t _nb_times_theorem4_applied = 0;
		std::vector<Node> _saved_branching_tree;


	public:
		void run(bool verbose) override;
	};

	///////////////////////////////////////////////////////////////////////////

	class AlgorithmFactory
	{
	public:
		static std::unique_ptr<Algorithm> create(std::string& algorithm);
	};

	
}

#endif // ALGORITHMS_RCPSP_H