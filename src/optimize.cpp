#include <cassert>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

#include "Instance.h"
#include "k_perm_solver.h"
#include "fast_direct_feasibility.h"

using json = nlohmann::json;

std::string help_text = R"(
Run optimizer on input_file for the given number of seconds (60 if not specified).
Output saved in output/distance or output/makespan depending on what type of optimization.

Options:
    -h:     print help text
    -m:     makespan version of optimization. Default is distance optimization
    -o:     output in json format specified by CG:SHOP (if not set, no JSON will be saved)
    -k val: value of k. Default is k=7 for distance and k=3 for makespan.
    -R val: value of radius R. Default is R=20
)";

void print_usage(char* prog_name) {
  std::cout << "usage: " << prog_name << " input_file [seconds] [-m] [-o output] [-k val] [-R radius] " << std::endl;
  std::cout << help_text <<std::endl;
}

instance convert(const std::string& filepath) {
	ifstream ifs(filepath);
	json raw_j = json::parse(ifs);
	instance ret;
	ret.clear();
	ret.name = raw_j["name"];
	for (const auto& p : raw_j["starts"])
		ret.start.emplace_back(p[0], p[1]);
	for (const auto& p : raw_j["targets"])
		ret.target.emplace_back(p[0], p[1]);
	for (const auto& p : raw_j["obstacles"])
		ret.obstacle.emplace_back(p[0], p[1]);

	assert(ret.start.size() == ret.target.size());
	ret.n = ret.start.size();
	ret.m = ret.obstacle.size();

	return ret;
}

void convert_back(const std::string& filename, bool makespan, const std::string& output_filename) {
	ifstream ifs(out_file_full(filename, makespan));
	json raw_j;

	std::string name;
	ifs >> name;
	raw_j["instance"] = name;

	int n = 0;
	ifs >> n;

	int time;
	ifs >> time;
	raw_j["steps"] = json::array();
	// raw_j["steps"].emplace_back(time, json::object());
	for (int i = 0; i < time; ++i) {
		raw_j["steps"].push_back(json::object());
		for (int j = 0; j < n; ++j) {
			int move;
			ifs >> move;
			if (move > 0)
				raw_j["steps"][i][std::to_string(j)] = dirnames[move];
		}
	}

	ofstream ofs(output_filename);
  std::cerr << "Outputting JSON to: " << output_filename << std::endl;
	ofs << raw_j.dump();
}

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
  int seconds = 60;
  int k = -1;
  int R = 20;
  bool makespan = false;
  bool outjson = false;
  std::string json_outfile;
  std::string raw_filename;
  if (argc < 2) {
    print_usage(argv[0]);
    exit(0);
  }
  else {
    raw_filename = argv[1];
    for (int i = 2; i < argc; ++i) {
      if (std::string(argv[i]) == std::string("-k")) {
        k = atoi(argv[i+1]);
        i++;
      } else if (std::string(argv[i]) == std::string("-m")) {
        makespan = true;
      } else if (std::string(argv[i]) == std::string("-o")) {
        outjson = true;
        json_outfile = argv[i+1];
        i++;
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

  std::string filename = remove_ext(raw_filename);
  instance ins;
  if (get_ext(raw_filename) == "json") {
    std::cerr << "json extention detected." << std::endl;
    if (ins.check_exists(filename)) {
      std::cerr << ".in file found, skipping reading from json file read" << std::endl;
    }
    else {
      std::cerr << ".in file not found, writing input from json to /input folder" << std::endl;
      ins = convert(raw_filename);
      ins.write_input(ins.name);
    }
  }
  ins.read(filename);

  if (ins.check_out_exists(filename, makespan)) {
    std::cerr << "WARNING: out file exists, will not update out file if there is no improvement." << std::endl;
    ins.read_out(makespan);
  }
  else {
    std::cerr << "Running matching initializer (fff) since no feasible out file exists..." << std::endl;
		fast_direct_feasibility::run_fast_direct_feasibility(ins);
    ins.write();
  }

  std::cerr << "Beginning "<< (makespan? "makespan": "distance") 
    << " optimization for " << seconds << " seconds, hang tight." << endl;
  if (makespan) {
    if (k_perm_solver::run(ins, seconds, true, k, R)) {
      std::cerr << "Makespan improver run completed on " << filename << std::endl;
      ins.write();
    }
  } else {
    if (k_perm_solver::run(ins, seconds, false, k, R)) {
      std::cerr << "Distance improver run completed on " << filename << std::endl;
      ins.write();
    }
  } 

  // write saved input to json
  if (outjson) {
    convert_back(filename, makespan, json_outfile);
  }

  // use this to forcibly write output
  //ins.debug_write("debug.out");
}
