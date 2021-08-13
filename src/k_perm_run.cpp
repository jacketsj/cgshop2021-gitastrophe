#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "k_perm_solver.h"

using namespace std;

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
  int seconds = 1;
  int k = 3;
  int R = 8;
  bool makespan = true;
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      if (string(argv[i]) == string("-k")) {
        k = atoi(argv[i+1]);
        i++;
      } else if (string(argv[i]) == string("-d")) {
        makespan = false;
      } else if (string(argv[i]) == string("-R")) {
        R = atoi(argv[i+1]);
        i++;
      } else {
        seconds = stoi(string(argv[i]));
      }
    }
  }

  string filename;
  while (cin >> filename) {
    filename = remove_ext(filename);
    instance ins;
    ins.read(filename);
    ins.read_out(makespan);
    cerr << "READ INPUT n=" << ins.n << endl;
    if (makespan) {
      if (k_perm_solver::run(ins, seconds, true, k, R)) {
        cout << "MAKESPAN IMPROVER RUN SUCCESSFUL" << endl;
        ins.write();
      }
    } else {
      if (k_perm_solver::run(ins, seconds, false, k, R)) {
        cout << "DISTANCE IMPROVER RUN SUCCESSFUL" << endl;
        ins.write();
      }
    }
    ins.debug_write("debug.out");
  }
}
