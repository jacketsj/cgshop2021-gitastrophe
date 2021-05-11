#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "two_perm_solver.h"

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
		cout << "Running with distance output, use -m to get makespan output" << endl;

	string filename;
	while (cin >> filename) {
		filename = remove_ext(filename);
		instance ins;
		ins.read(filename);
		ins.read_out(makespan);
		cerr << "READ INPUT n=" << ins.n << endl;
		if (two_perm_solver::run(ins, seconds)) {
			cout << "GREEDY IMPROVER RUN SUCCESSFUL" << endl;
			ins.write();
		}
		ins.debug_write("debug.out");
	}
}
