#include "algorithms.h"
#include <stdexcept>
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <queue>


namespace RCPSP
{
	void SCIP::build_problem()
	{
		// create the solver (_solver_type == SCIP or CPLEX or ...)
		_solver.reset(operations_research::MPSolver::CreateSolver(_solver_type));


		// add variables
		size_t max_periods = 0;
		for (auto&& d : _activity_durations)
			max_periods += d;
		const double infinity = _solver->infinity();


		// variables x_jt
		for (auto j = 0; j < _nb_activities; ++j)
		{
			for (auto t = 0; t < max_periods; ++t)
			{
				std::string varname = "x_" + std::to_string(j + 1) + "_" + std::to_string(t + 1);
				operations_research::MPVariable* var = _solver->MakeBoolVar(varname);
			}
		}

		// variable Z
		{
			std::string varname = "Z";
			operations_research::MPVariable* var = _solver->MakeNumVar(0.0, infinity, varname);
		}


		// set objective function
		operations_research::MPObjective* objective = _solver->MutableObjective();
		objective->SetMinimization();
		{ // Z variable
			operations_research::MPVariable* var = _solver->variable(_nb_activities*max_periods);
			objective->SetCoefficient(var, 1);
		}



		// add constraints
		int nb_constraints = -1;

		// 1: every activity starts exactly once
		for (auto j = 0; j < _nb_activities; ++j)
		{
			++nb_constraints;

			std::string conname = "c1_" + std::to_string(j + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(1.0, 1.0, conname);

			// x_jt
			for (auto t = 0; t < max_periods; ++t)
			{
				size_t index_var = j * max_periods + t;
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}
		}

		// 2: precedence relations
		for (auto i = 0; i < _nb_activities; ++i)
		{
			for (auto j = 0; j < _nb_activities; ++j)
			{
				if (precedence(i, j) == 1) // j after i
				{
					++nb_constraints;

					std::string conname = "c2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
					operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(_activity_durations[i], infinity, conname);

					// x_jt
					for (auto t = 0; t < max_periods; ++t)
					{
						size_t index_var = j * max_periods + t;
						operations_research::MPVariable* var = _solver->variable(index_var);
						constraint->SetCoefficient(var, t);
					}

					// x_it
					for (auto t = 0; t < max_periods; ++t)
					{
						size_t index_var = i * max_periods + t;
						operations_research::MPVariable* var = _solver->variable(index_var);
						constraint->SetCoefficient(var, -t);
					}
				}
			}
		}

		// 3: resource constraints
		for (auto k = 0; k < _nb_resources; ++k)
		{
			for (auto t = 0; t < max_periods; ++t)
			{
				++nb_constraints;

				std::string conname = "c3_" + std::to_string(k + 1) + "_" + std::to_string(t + 1);
				operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(-infinity, _resource_availabilities[k], conname);

				// x_jtau
				for (auto j = 0; j < _nb_activities; ++j)
				{
					for (auto tau = std::max(t - _activity_durations[j] + 1, 0); tau <= t; ++tau)
					{
						size_t index_var = j * max_periods + tau;
						operations_research::MPVariable* var = _solver->variable(index_var);
						constraint->SetCoefficient(var, resource_requirement(j,k));
					}
				}
			}
		}

		// 4: makespan constraints
		for (auto j = 0; j < _nb_activities; ++j)
		{
			++nb_constraints;

			std::string conname = "c4_" + std::to_string(j + 1);
			operations_research::MPConstraint* constraint = _solver->MakeRowConstraint(_activity_durations[j], infinity, conname);

			// x_jt
			for (auto t = 0; t < max_periods; ++t)
			{
				size_t index_var = j * max_periods + t;
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, -t);
			}

			// Z
			{
				size_t index_var = _nb_activities * max_periods;
				operations_research::MPVariable* var = _solver->variable(index_var);
				constraint->SetCoefficient(var, 1);
			}
		}

		


		// write to file
		//_solver->Write("SCIP1.lp"); // NOT YET SUPPORTED

		{
			std::ofstream mfile("ORTools_IP.lp");
			mfile << "Obj\t";
			for (auto& var : _solver->variables())
			{
				if (_solver->Objective().GetCoefficient(var) > 0.00001 || _solver->Objective().GetCoefficient(var) < -0.00001)
					mfile << _solver->Objective().GetCoefficient(var) << " " << var->name() << " + ";
			}

			for (auto& con : _solver->constraints())
			{
				mfile << "\n\n" << con->name() << "\t" << con->lb() << " <=  ";
				for (auto& var : _solver->variables())
				{
					if (con->GetCoefficient(var) > 0.00001 || con->GetCoefficient(var) < -0.00001)
						mfile << con->GetCoefficient(var) << " " << var->name() << " + ";
				}
				mfile << "  <= " << con->ub();
			}
		}
	}

	void SCIP::solve_problem()
	{
		std::cout << "\nUsing an IP model with x_jk = 1 if activity j starts at time t, 0 otherwise"
			<< "\nUsing ORTools with SCIP to solve the model ...\n\n";

		// Output to screen
		if (_output_screen)
			_solver->EnableOutput();
		else
			_solver->SuppressOutput();

		// Set time limit (milliseconds) 
		int64_t time_limit = static_cast<int64_t>(_max_computation_time * 1000);
		_solver->set_time_limit(time_limit);

		// Solve the problem
		auto start_time = std::chrono::system_clock::now();
		const operations_research::MPSolver::ResultStatus result_status = _solver->Solve();
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time_IP = std::chrono::system_clock::now() - start_time;

		std::cout << "\nResult solve = " << result_status;

		// If optimal or feasible (e.g. time limit reached)
		if (result_status == operations_research::MPSolver::OPTIMAL || result_status == operations_research::MPSolver::FEASIBLE)
		{
			double objval = _solver->Objective().Value();

			std::cout << "\nElapsed time (s): " << elapsed_time_IP.count();

			std::cout << "\nMinimum project length = " << objval;

			size_t max_periods = 0;
			for (auto&& d : _activity_durations)
				max_periods += d;

			std::cout << "\nActivity finish times:";
			for (auto j = 0; j < _nb_activities; ++j)
			{
				for (auto t = 0; t < max_periods; ++t)
				{
					operations_research::MPVariable* var = _solver->variable(j*max_periods + t);
					double solvalue = var->solution_value();


					if (solvalue > 0.99)
						std::cout << "  f(" << j + 1 << ") = " << t + _activity_durations[j];
				}
			}
		}
	}

	void SCIP::run(bool verbose)
	{
		_output_screen = verbose;

		build_problem();
		solve_problem();
	}

	///////////////////////////////////////////////////////////////////////////
}