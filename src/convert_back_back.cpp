// this file is basically for generating test data for the visualizer

#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

using std::cout;
using json = nlohmann::json;
using std::ifstream;
using std::map;
using std::string;
using std::to_string;

const string RAW_JSON_OUTPUT_FOLDER = "manual/";

void convert_back_back(const string& filename) {
	ifstream ifs(RAW_JSON_OUTPUT_FOLDER + filename);
	json raw_j = json::parse(ifs);
	int n = 10; // hardcoded since i'm only using this for one file
	map<char, int> dir_rev = {{'N', 1}, {'E', 2}, {'S', 3}, {'W', 4}};
	cout << 10 << '\n' << raw_j["steps"].size() << '\n';
	for (auto& s : raw_j["steps"]) {
		for (int i = 0; i < n; ++i) {
			if (s.contains(to_string(i))) {
				cout << dir_rev[s[to_string(i)].dump()[1]];
			} else {
				cout << 0;
			}
			cout << ' ';
		}
		cout << '\n';
	}
}

int main(int argc, char* argv[]) {
	std::string filename;
	while (std::cin >> filename) {
		convert_back_back(filename);
	}
}
