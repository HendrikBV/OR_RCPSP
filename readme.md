Algorithms for the Resource-Constrained Project Scheduling Problem
==================================================================
This software implements algorithms for the resource-constrained project scheduling problem. A set of activities need to be scheduled without violating precedence or resource constraints such that the project makespan is minimized.

Example on how to run the software:
`OR_RCPSP --data="example.txt" --algorithm="DH" --verbose`

Parameters:
* `--algorithm`  The choice of algorithm. Possibilities:
  + "IP1": an integer programming model solved with SCIP (with variables x[j][t] = 1 if activity j start at time t)
  + "IP2": an alternative integer programming model solved with SCIP (flow-based)
  + "DH": the branch-and-bound procedure of Demeulemeester-Herroelen (1992)
* `--data`       Name of the file containing the problem data
* `--verbose`        Explain the various steps of the algorithm
* `--help`         Help on how to use the application