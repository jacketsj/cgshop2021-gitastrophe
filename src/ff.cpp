#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "fast_feasibility.h"

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
		cerr << "READ INPUT n=" << ins.n << endl;
		cout << "running FAST feasibility on " << ins.name << "!" << endl;
		fast_feasibility::run_fast_feasibility(ins);
		string redoloc = OUTPATH + string("redo4/") + ins.name + OUTEXT;
		ins.debug_write(redoloc);
		ins.write();
	}
}
