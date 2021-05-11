#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "fast_direct_feasibility.h"

using namespace std;

int main(int argc, char* argv[]) {
	string filename;
	while (cin >> filename) {
		filename = remove_ext(filename);
		instance ins;
		ins.read(filename);
		cerr << "READ INPUT n=" << ins.n << endl;
		cout << "running FAST DIRECT feasibility on " << ins.name << "!" << endl;
		fast_direct_feasibility::run_fast_direct_feasibility(ins);
		string redoloc = OUTPATH + string("redo4/") + ins.name + OUTEXT;
		ins.improve_only_debug_write(redoloc, true);
		//ins.debug_write("debug.out");
		ins.write();
	}
}
