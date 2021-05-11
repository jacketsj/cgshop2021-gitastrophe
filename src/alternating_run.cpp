#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "greedy_improve.h"
#include "two_opt_solver.h"

using namespace std;

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
	int seconds = 1;
	bool makespan = false;
	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			if (string(argv[i]) == string("-m"))
				makespan = true;
			else
				seconds = stoi(string(argv[i]));
		}
	}

	if (!makespan)
		cout << "Running with distance output, use -m to get makespan output"
				 << endl;

	string filename;
	while (cin >> filename) {
		filename = remove_ext(filename);
		instance ins;
		ins.read(filename);
		ins.read_out(makespan);
		cerr << "READ INPUT n=" << ins.n << endl;
		cout << "ALTERNATING GREEDY and TWO-OPT" << endl;
		int start_time = clock();
		while ((clock() - start_time) < seconds * CLOCKS_PER_SEC) {
			if (two_opt_solver::run(ins, 5)) {
				cout << "done 1 second of 2-opt greedy" << endl;
				ins.write();
			}
			if (!((clock() - start_time) < seconds * CLOCKS_PER_SEC))
				break;
			if (greedy_improver::round_robin_improve(ins, 5, true)) {
				cout << "done 1 second of greedy" << endl;
				ins.write();
			}
		}
		cout << "ALTERNATING RUN SUCCESSFUL" << endl;
		ins.debug_write("debug.out");
	}
}
