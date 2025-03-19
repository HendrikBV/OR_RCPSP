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
		std::vector<std::vector<int>> _precedence_relations;
		std::vector<int> _activity_durations;
		std::vector<std::vector<int>> _activity_resource_requirements;


	public:
		virtual ~Algorithm() {}

		void read_data(const std::string& filename);

		virtual void run(bool verbose) = 0;
	};

	///////////////////////////////////////////////////////////////////////////

	// with variables x[j][t] == 1 if job j starts at time t, 0 otherwise
	class SCIP1 : public Algorithm
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

	// with flow variables
	class SCIP2 : public Algorithm
	{
		std::unique_ptr<operations_research::MPSolver> _solver; // OR Tools solver
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