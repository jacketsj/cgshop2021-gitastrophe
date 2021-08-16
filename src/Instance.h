#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <filesystem>

using namespace std;

const string OUTPATH = "../output/";
const string MSPATH = "makespan/";
const string DISTPATH = "distance/";
const string INPATH = "../input/";
const string EXT = ".in";
const string OUTEXT = ".out";

string get_ext(string s) {
	size_t last_slash = s.find_last_of("/");
	if (last_slash != string::npos)
		s = s.substr(last_slash + 1);

	size_t last_dot = s.find_last_of("."); // find last extention
	if (last_dot == string::npos)
		return s;
	return s.substr(last_dot+1, s.size());
}

string remove_ext(string s) {
	size_t last_slash = s.find_last_of("/");
	if (last_slash != string::npos)
		s = s.substr(last_slash + 1);

	size_t last_dot = s.find_first_of("."); // strip all extentions (this is important)
	if (last_dot == string::npos)
		return s;
	return s.substr(0, last_dot);
}

string out_directory(bool makespan) {
  return OUTPATH + (makespan ? MSPATH : DISTPATH);
}

string out_file_full(string name, bool makespan) {
	return out_directory(makespan) + name + OUTEXT;
}

const int INF = 0x3f3f3f3f;

struct pt {
	pt() : x(0), y(0) {}
	pt(int x, int y) : x(x), y(y) {}
	int x, y;
	pt operator+(const pt& oth) const { return pt(x + oth.x, y + oth.y); }
	pt operator-(const pt& oth) const { return pt(x - oth.x, y - oth.y); }
	bool operator==(const pt& oth) const {
		return tie(x, y) == tie(oth.x, oth.y);
	}
	bool operator!=(const pt& oth) const {
		return tie(x, y) != tie(oth.x, oth.y);
	}
	bool operator<(const pt& oth) const { return tie(x, y) < tie(oth.x, oth.y); }
};
int abs(const pt& p) { return abs(p.x) + abs(p.y); }
int dist(const pt& a, const pt& b) { return abs(a - b); }
long long dot(const pt& a, const pt& b) { return a.x * b.x + a.y * b.y; }
double norm(const pt& a) { return sqrt(dot(a, a)); }

ostream& operator<<(ostream& stream, const pt& p) {
	stream << "(" << p.x << ", " << p.y << ")";
	return stream;
}

// Move encoding
// 0: Stand still
// 1: North
// 2: East
// 3: South
// 4: West
const int dx[] = {0, 0, 1, 0, -1, 1, -1, 1, -1};
const int dy[] = {0, 1, 0, -1, 0, 1, -1, -1, 1};
const pt dxy[] = {pt(0, 0), pt(0, 1),		pt(1, 0),	pt(0, -1), pt(-1, 0),
									pt(1, 1), pt(-1, -1), pt(1, -1), pt(-1, 1)};
const vector<string> dirnames = {"", "N", "E", "S", "W"};
const int opposite_dir[] = {0, 3, 4, 1, 2};

namespace std {
template <> struct hash<pt> {
	size_t operator()(const pt& p) const { return (p.x << 9) + p.y; }
};

template <> struct hash<pair<pt, int>> {
	size_t operator()(const pair<pt, int>& p) const {
		return hash<pt>()(p.first) ^ (hash<int>()(p.second) << 1);
	}
};
} // namespace std

struct instance;
bool verify(instance& ins);
bool improvement(instance& ins, bool makespan);
bool improvement_custom(instance& ins, string full_filename, bool makespan);
struct instance {
	string name;
	int n, m; // n is the number of robots, m obstacles
	int time;
	vector<pt> start;
	vector<pt> target;
	vector<pt> obstacle;
	vector<vector<int>> moves; // list of simultaneous moves (not sparse)

	// big score is bad
	// TODO we don't actually use this
	long double score;
	bool operator<(const instance& other) const { return score > other.score; }

	bool done(int i) { return abs(start[i] - target[i]) == 0; }
	bool done() {
		for (int i = 0; i < n; i++)
			if (!done(i))
				return false;
		return true;
	}

	void clear() {
		name = "";
		n = m = 0;
		time = 0;
		score = 0;
		start.clear();
		target.clear();
		obstacle.clear();
		moves.clear();
	}

	instance sub_instance(const vector<size_t>& sub_robots) {
		instance ret;
		ret.clear();
		ret.m = m;
		ret.obstacle = obstacle;
		ret.time = time;
		ret.n = sub_robots.size();
		for (int i = 0; i < ret.n; ++i) {
			ret.start.push_back(start[sub_robots[i]]);
			ret.target.push_back(target[sub_robots[i]]);
		}
		ret.moves.resize(ret.time);
		for (int t = 0; t < ret.time; ++t)
			for (size_t i = 0; i < sub_robots.size(); ++i)
				ret.moves[t].push_back(moves[t][sub_robots[i]]);
		return ret;
	}

	void merge_sub_instance(const instance& sub,
													const vector<size_t>& sub_robots) {
		time = max(time, sub.time);
		while (moves.size() < size_t(time))
			moves.emplace_back(n);
		for (int t = 0; t < sub.time; ++t)
			for (size_t i = 0; i < sub_robots.size(); ++i)
				moves[t][sub_robots[i]] = sub.moves[t][i];
		for (int t = sub.time; t < time; ++t)
			for (size_t i = 0; i < sub_robots.size(); ++i)
				moves[t][sub_robots[i]] = 0;
	}

  bool check_exists(string filename) {
		filename = remove_ext(filename);
		string filepath = INPATH + filename + EXT;
    return filesystem::exists(filepath);
  }

  bool check_out_exists(string filename, bool makespan) {
    if (!filesystem::exists(out_directory(makespan))) {
      filesystem::create_directory(out_directory(makespan));
      return false;
    }
    return filesystem::exists(out_file_full(filename, makespan));
  }

	void read(string filename) { // clear instance and read new instance
		clear();
		filename = remove_ext(filename);
		name = filename;
		string filepath = INPATH + filename + EXT;
		// cerr << "READING FILE " << filepath << endl;
		ifstream in(filepath);
		in >> n >> m;
		for (int i = 0; i < n; i++) {
			int x, y;
			in >> x >> y;
			start.emplace_back(x, y);
		}
		for (int i = 0; i < n; i++) {
			int x, y;
			in >> x >> y;
			target.emplace_back(x, y);
		}
		for (int i = 0; i < m; i++) {
			int x, y;
			in >> x >> y;
			obstacle.emplace_back(x, y);
		}
	}

	void read_custom(string fullpath) {
		ifstream in(fullpath);

		string in_name;
		in >> in_name;
		read(in_name);

		int idump;
		in >> idump;
		in >> time;

		moves = vector<vector<int>>(time, vector<int>(n));
		for (auto& vi : moves)
			for (auto& i : vi)
				in >> i;
	}

	string out_file(bool makespan) { return out_file_full(name, makespan); }

	void read_out(bool makespan) { // read moves and time
    if (!filesystem::exists(out_file(makespan))) {
      cerr << "ERROR: Did not find .out file" <<endl;
    }
		ifstream in(out_file(makespan));

		// dump extra info, and get time
		string dump;
		in >> dump;
		int idump;
		in >> idump;
		in >> time;

		moves = vector<vector<int>>(time, vector<int>(n));
		for (auto& vi : moves)
			for (auto& i : vi)
				in >> i;
	}

	void write_input(string filename) {
		// Re-outputs the input string
		// Only really used for converting json
		ofstream out(INPATH + filename + EXT);
		// cout << "writing to " << INPATH << filename << EXT << endl;
		out << n << ' ' << m << '\n';
		for (const auto& p : start)
			out << p.x << ' ' << p.y << '\n';
		for (const auto& p : target)
			out << p.x << ' ' << p.y << '\n';
		for (const auto& p : obstacle)
			out << p.x << ' ' << p.y << '\n';
	}

	void write_mapf(string filename) {
		const string MAPF_OUTPATH = "../mapf_input/";
		const string MAPF_MAP_EXT = ".map";
		const string MAPF_AGENT_EXT = ".agent";
		const int GAP = 2;
		string map_filepath = MAPF_OUTPATH + filename + MAPF_MAP_EXT;
		string agents_filepath = MAPF_OUTPATH + filename + MAPF_AGENT_EXT;
		ofstream mapout(map_filepath);
		ofstream agentout(agents_filepath);
		cerr << "writing to " << map_filepath << " and " << agents_filepath << endl;
		// compute map for mapf
		pt mx(-1, -1);
		string blank(120, '.');
		vector<string> g(120, blank);
		for (auto& p : obstacle) {
			g[p.x + GAP][p.y + GAP] = '@';
			mx.x = max(mx.x, p.x);
			mx.y = max(mx.y, p.y);
		}
		for (auto& p : start) {
			mx.x = max(mx.x, p.x);
			mx.y = max(mx.y, p.y);
		}
		for (auto& p : target) {
			mx.x = max(mx.x, p.x);
			mx.y = max(mx.y, p.y);
		}
		mx.x += 2 * GAP;
		mx.y += 2 * GAP;
		mapout << mx.x << ',' << mx.y << '\n';
		while ((int)g.size() > mx.y) {
			g.pop_back();
		}
		for (auto& s : g) {
			s.resize(mx.x);
			mapout << s << '\n';
		}
		agentout << n << '\n';
		for (int i = 0; i < n; i++) {
			agentout << start[i].x + GAP << ',' << start[i].y + GAP << ',';
			agentout << target[i].x + GAP << ',' << target[i].y + GAP << '\n';
		}
	}

	void complete_write(bool makespan) {
		string out_f = out_file(makespan);
		// cout << "writing to " << out_f << endl;
		ofstream out(out_f);
		out << name << '\n'; // also include the instance name for convenience
		out << n << '\n';		 // also include the value of n for convenience
		out << time << '\n';
		for (const auto& v : moves) {
			for (int i = 0; i < n; i++) {
				out << v[i] << " ";
			}
			out << '\n';
		}
	}

	void write() {
		if (!verify(*this)) {
			cerr << "Verification failed! Not writing file." << endl;
			return;
		}
		vector<bool> vals = {false, true};
		for (bool makespan : vals)
			if (improvement(*this, makespan)) {
        cerr << "Improvement found for " << name << " on " << (makespan? "makespan": "distance") << endl;
				complete_write(makespan);
			}
	}

	void debug_write(string full_filename) {
		ofstream out(full_filename);
		out << name << '\n';
		out << n << '\n';
		out << time << '\n';
		for (const auto& v : moves) {
			for (int i = 0; i < n; i++) {
				out << v[i] << " ";
			}
			out << '\n';
		}
	}

	void improve_only_debug_write(string full_filename, bool makespan) {
		if (!verify(*this)) {
			cerr << "Verification failed! Not writing file." << endl;
			return;
		}
		if (improvement_custom(*this, full_filename, makespan)) {
			cerr << "It's a (debug) improvement (" << full_filename << ")!" << endl;
			debug_write(full_filename);
		}
	}
};

void slowdown(instance& ins, int slow) {
	vector<vector<int>> m2;
	vector<int> zeroes(ins.n);
	for (auto v : ins.moves) {
		m2.push_back(v);
		for (int i = 0; i < slow; i++)
			m2.push_back(zeroes);
	}
	ins.time = m2.size();
	ins.moves = m2;
}

// the reverse problem (and solution)
void reverse_instance(instance& ins) {
	ins.start.swap(ins.target);
	reverse(ins.moves.begin(), ins.moves.end());
	for (auto& v : ins.moves) {
		for (int& d : v) {
			d = opposite_dir[d];
		}
	}
}

vector<vector<int>> combine_movesets(const instance& ins1,
																		 const instance& ins2) {
	assert(ins1.n == ins2.n);
	assert(ins1.m == ins2.m);
	assert(ins1.target == ins2.start);

	vector<vector<int>> moves = ins1.moves;
	for (auto& vi : ins2.moves)
		moves.push_back(vi);
	return moves;
}

bool verify(instance& ins) {
	if (ins.time != int(ins.moves.size()))
		return false;

	vector<pt> locations = ins.start;
	set<pt> obstacle_s;
	for (pt& p : ins.obstacle) {
		obstacle_s.insert(p);
	}
	int t = 0;
	for (auto& step : ins.moves) {
		map<pt, int> dir_from_loc;
		for (int i = 0; i < ins.n; ++i) {
			dir_from_loc[locations[i]] = step[i];
		}
		for (int i = 0; i < ins.n; ++i) {
			pt p = locations[i] + dxy[step[i]];
			// determine if move is a collision
			if (obstacle_s.count(p) > 0 ||
					(dir_from_loc.count(p) > 0 && dir_from_loc[p] != step[i])) {
				cout << "failed on timestep t=" << t << endl;
				return false;
			}
			// move if it's not (doesn't affect later loop checks)
			locations[i] = p;
		}
		++t;
	}
	return locations == ins.target;
}

// lower score is better
int score(instance& ins, bool makespan) {
	if (!verify(ins))
		return INF;

	assert(ins.time == int(ins.moves.size()));
	if (makespan)
		return ins.time;

	int distance = 0;
	for (auto& step : ins.moves) {
		for (int i = 0; i < ins.n; ++i) {
			if (step[i] != 0)
				++distance;
		}
	}
	return distance;
}

double average_makespan(instance& ins) {
	vector<int> counter(ins.n);
	double ms = 0;
	for (auto& step : ins.moves) {
		for (int i = 0; i < ins.n; i++) {
			if (step[i]) {
				ms += 1 + counter[i];
				counter[i] = 0;
			} else {
				counter[i]++;
			}
		}
	}
	return ms / ins.n;
}
double squared_makespan_sum(instance& ins) {
	double ms = 0;
	for (int i = 0; i < ins.n; i++) {
		double cur = 0;
		int counter = 0;
		for (size_t s = 0; s < ins.moves.size(); ++s) {
			if (ins.moves[s][i]) {
				cur += 1 + counter;
				counter = 0;
			} else {
				counter++;
			}
		}
		ms += cur * cur;
	}
	return ms;
}

bool improvement(instance& ins, bool makespan) {
	instance dupe = ins; // copy most info from ins
	dupe.read_out(makespan);
	int old_score = score(dupe, makespan);
	int new_score = score(ins, makespan);

	if (old_score == new_score) {
		// Break ties by average makespan
		if (squared_makespan_sum(ins) < squared_makespan_sum(dupe)) {
			return true;
		}
	}
	return new_score < old_score;
}

bool improvement_custom(instance& ins, string full_filename, bool makespan) {
	instance dupe; // copy most info from ins
	dupe.read_custom(full_filename);
	if (ins.n != dupe.n)
		return false;

	int old_score = score(dupe, makespan);
	int new_score = score(ins, makespan);

	if (old_score == new_score) {
		// Break ties by average makespan
		if (squared_makespan_sum(ins) < squared_makespan_sum(dupe)) {
			return true;
		}
	}
	return new_score < old_score;
}
