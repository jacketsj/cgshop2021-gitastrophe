#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "cbs.h"

using namespace std;

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
		if (cbs::search(ins, seconds)) {
			cout << "SUCCESS" << endl;
			ins.write();
			// ins.debug_write("../output/debug.out");
		} else {
			cout << "FAILURE" << endl;
		}
		// optimizer opt(ins);
		// auto output = opt.a_star(); // optimize!
		// output.write(filename);
	}
}
