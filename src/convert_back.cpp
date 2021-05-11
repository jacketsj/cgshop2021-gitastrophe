#include <cassert>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "Instance.h"

using std::cout;
using std::string;
using json = nlohmann::json;

const string RAW_JSON_OUTPUT_FOLDER = "../json_out/";

void convert_back(const string& filename, bool makespan) {
	ifstream ifs(out_file_full(filename, makespan));
	json raw_j;

	string name;
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
				raw_j["steps"][i][to_string(j)] = dirnames[move];
		}
	}

	string ms_or_dist = makespan ? "makespan" : "distance";
	string output_name =
			RAW_JSON_OUTPUT_FOLDER + name + "-" + ms_or_dist + ".json";
	ofstream ofs(output_name);
	cout << "outputting to: " << output_name << endl;
	ofs << raw_j.dump();
}

int main(int argc, char* argv[]) {
	std::string filename;
	while (std::cin >> filename) {
		if (filename != "." && filename != "..") {
			filename = remove_ext(filename);
			convert_back(filename, false);
			convert_back(filename, true);
		}
	}
}
