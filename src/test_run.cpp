#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "cbs_improve.h"
#include "fast_feasibility.h"
#include "feasibility_solver.h"
#include "greedy_improve.h"
#include "greedy_opt_with_random.h"
#include "its_all_a_simulation.h"
#include "physics_solver.h"

using namespace std;
enum {
	GREEDY_RANDOM,
	PHYSICS,
	FEASIBILITY,
	FAST_FEAS,
	SIMULATION,
	RR,
	SLOWDOWN,
	CBS_IMPROVE
};

#define ALG FAST_FEAS

int main(int argc, char* argv[]) {
	int seconds = 1;
	if (argc > 1) {
		seconds = stoi(string(argv[1]));
	}

	string filename;
	while (cin >> filename) {
		filename = remove_ext(filename);
		instance ins;
		ins.read(filename);
		cerr << "READ INPUT n=" << ins.n << endl;
		if (ALG == GREEDY_RANDOM) {
			if (greedy::try_a_bunch(ins, seconds)) { // optimize!
				cout << "BAD GREEDY RANDOM SUCCEEDED (how?)" << endl;
				ins.write();
			} else {
				cout << "BAD GREEDY RANDOM FAILED (obviously)" << endl;
				ins.debug_write("debug.out");
			}
		} else if (ALG == PHYSICS) {
			bool succ = physics::run_physics(ins, seconds);
			if (succ) {
				cout << "PHYSICS TRIUMPHS" << endl;
				ins.write();
				ins.debug_write("debug.out");
			} else {
				cout << "PHYSICS FAILS :(" << endl;
				ins.debug_write("debug.out");
			}
		} else if (ALG == FEASIBILITY) {
			cout << "running feasibility!" << endl;
			feasibility::run_feasibility(ins, seconds);
			ins.debug_write("debug.out");
			ins.write();
		} else if (ALG == FAST_FEAS) {
			cout << "running FAST feasibility!" << endl;
			fast_feasibility::run_fast_feasibility(ins);
			ins.debug_write("debug.out");
			ins.write();
		} else if (ALG == SIMULATION) {
			cout << "IT'S ALL A SIMULATION" << endl;
			bool succ = simulation::run_sim(ins, seconds);
			if (succ) {
				cout << "ESCAPED THE SIMULATION!" << endl;
				ins.write();
			} else
				cout << "forever lost" << endl;
			ins.debug_write("debug.out");
		} else if (ALG == SLOWDOWN) {
			int x = 5;
			cerr << "slowing down by " << x << endl;
			slowdown(ins, x);
		} else if (ALG == CBS_IMPROVE) {
			ins.read_out(true);
			if (cbs_improve::run(ins, seconds)) {
				cout << "CBS Improver b nice" << endl;
				ins.write();
			} else
				cout << "cbs improver sad" << endl;
			ins.debug_write("debug.out");
		}

		cout << "About to run round robin!" << endl;

		if (greedy_improver::longest_improve(ins, seconds, false)) {
			cout << "GREEDY IMPROVER RUN SUCCESSFUL" << endl;
			ins.write();
			ins.debug_write(ins.name + ".test");
		}
	}
}
