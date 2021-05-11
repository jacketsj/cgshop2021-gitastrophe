#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "cbs_improve.h"

using namespace std;

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
	int seconds = 5;
	bool makespan = false;
	bool custom = false;
	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			if (string(argv[i]) == string("-m"))
				makespan = true;
			else if (string(argv[i]) == string("-c"))
				custom = true;
			else
				seconds = stoi(string(argv[i]));
		}
	}

	if (!makespan && !custom)
		cout << "Running with distance output, use -m to get makespan output, or "
						"-c to use custom output"
				 << endl;

	string filename;
	while (cin >> filename) {
		instance ins;
		if (custom)
			ins.read_custom(filename);
		else {
			ins.read(filename);
			ins.read_out(makespan);
		}
		if (cbs_improve::run(ins, seconds)) {
			cout << "CBS IMPROVER RUN SUCCESSFUL" << endl;
			ins.write();
		}
		ins.debug_write("debug.out");
	}
}
