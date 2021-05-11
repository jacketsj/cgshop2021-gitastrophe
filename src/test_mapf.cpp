#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"

using namespace std;

int main(int argc, char* argv[]) {
	string filename;
	while (cin >> filename) {
		filename = remove_ext(filename);
		instance ins;
		ins.read(filename);
		cerr << "READ INPUT n=" << ins.n << " m=" << ins.m << endl;
    ins.write_mapf(filename);
	}
}
