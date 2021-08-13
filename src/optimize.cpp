#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "k_perm_solver.h"

//using namespace std;

std::string help_text = R"(
Run optimizer on input_file for the given number of seconds (60 if not specified).

Options:
    -h:     print help text
    -m:     makespan version of optimization. Default is distance optimization
    -k val: value of k. Default is k=7 for distance and k=3 for makespan.
    -R val: value of radius R. Default is R=20
)";

void print_usage(char* prog_name) {
  std::cout << "usage: " << prog_name << " input_file [seconds] [-m] [-k val] [-R radius] " << std::endl;
  std::cout << help_text <<std::endl;
}

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
  int seconds = 60;
  int k = -1;
  int R = 20;
  bool makespan = false;
  std::string filename;
  if (argc <= 2) {
    print_usage(argv[0]);
    exit(0);
  }
  if (argc > 2) {
    filename = argv[1];
    for (int i = 2; i < argc; ++i) {
      if (std::string(argv[i]) == std::string("-k")) {
        k = atoi(argv[i+1]);
        i++;
      } else if (std::string(argv[i]) == std::string("-m")) {
        makespan = true;
      } else if (std::string(argv[i]) == std::string("-R")) {
        R = atoi(argv[i+1]);
        i++;
      } else if (std::string(argv[i]) == std::string("-h") || std::string(argv[i]) == std::string("--help")) {
        print_usage(argv[0]);
        exit(0);
      } else {
        seconds = stoi(std::string(argv[i]));
      }
    }
  }
  if (k == -1) {
    if (makespan) {
      k = 3;
    }
    else {
      k = 7;
    }
  }

  filename = remove_ext(filename);
  instance ins;
  ins.read(filename);
  ins.read_out(makespan);
  if (makespan) {
    if (k_perm_solver::run(ins, seconds, true, k, R)) {
      std::cerr << "Makespan improver run successful on " << filename << std::endl;
      ins.write();
    }
  } else {
    if (k_perm_solver::run(ins, seconds, false, k, R)) {
      std::cerr << "Distance improver run successful on " << filename << std::endl;
      ins.write();
    }
  }
  //ins.debug_write("debug.out");
}
