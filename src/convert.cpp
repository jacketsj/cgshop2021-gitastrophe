#include <cassert>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "Instance.h"

using std::cout;
using std::string;
using json = nlohmann::json;

const string RAW_JSON_INPUT_FOLDER = "../json_in/";

instance convert(const string& filename) {
	ifstream ifs(RAW_JSON_INPUT_FOLDER + filename + ".json");
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

int main(int argc, char* argv[]) {
	std::string filename;
	while (std::cin >> filename) {
		auto instance = convert(remove_ext(filename));
		instance.write_input(instance.name);
	}
}
