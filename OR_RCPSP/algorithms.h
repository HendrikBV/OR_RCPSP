#pragma once
#ifndef ALGORITHMS_RCPSP_H
#define ALGORITHMS_RCPSP_H


#include <exception>
#include <vector>
#include <string>
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
		// output
		Output _output;
		bool _verbose;

		// project data
		struct Activity
		{
			int id;
			int duration;
			std::vector<int> resource_requirements;
			std::vector<int> successors;
			std::vector<int> predecessors;
		};
		std::vector<Activity> _activities;
		std::vector<int> _resource_availabilities;


		// optimal solution
		int _upper_bound = std::numeric_limits<int>::max();
		std::vector<int> _best_activity_finish_times;



	public:
		virtual ~Algorithm() {}

		void read_data(const std::string& filename);
		void check_solution();
		virtual void run(bool verbose) = 0;
	};

	///////////////////////////////////////////////////////////////////////////

	// with variables x[j][t] == 1 if job j starts at time t, 0 otherwise
	class IP : public Algorithm
	{
		std::unique_ptr<operations_research::MPSolver> _solver; // OR Tools solver
		std::string _solver_type = "SCIP";

		void build_problem();
		void solve_problem();

		bool _output_screen = false;
		double _max_computation_time = 1800; // seconds

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
			int node;
			int parent_node;
			int decision_point;
			std::vector<bool> act_active;
			std::vector<int> act_finish_time;
			std::vector<bool> act_unassigned;
		};

		struct Node
		{
			int id = -1;
			int level_tree = -1;
			int lower_bound = 0;
			int decision_point = 0;

			std::vector<bool> act_in_PS; // [i] == true if activity i is in partial schedule
			std::vector<bool> act_active; // [i] == true if activity i is active
			std::vector<int> act_finish_time; // [i] == finish time of activity i
			std::vector<bool> act_eligible;	// [i] == true if activity i is eligible
			//std::vector<std::pair<int, int>> additional_precedences; // (i,j) if i before j

			Cutset cutset;
		};

		std::vector<Cutset> _saved_cutsets;
		std::vector<Node> _remaining_nodes;
		std::vector<int> _RCPL;

		void print_node(const Node& node);
		void print_cutset(const Cutset& cutset);

		int calculate_RCPL(int activity);
		bool transitive_precedence_between_activities_forward(int start, int target, int act_recursion);
		bool transitive_precedence_between_activities_backward(int start, int target, int act_recursion);
		std::vector<std::vector<int>> find_min_delaying_sets(const Node& node, const std::vector<int>& res_to_release);

		void clear_all();
		void procedure();

		size_t _nodes_evaluated = 0;
		size_t _nodes_LB_dominated = 0;
		size_t _nodes_cutset_dominated = 0;
		size_t _nodes_leftshift_dominated = 0;
		size_t _nb_times_theorem3_applied = 0;
		size_t _nb_times_theorem4_applied = 0;


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
