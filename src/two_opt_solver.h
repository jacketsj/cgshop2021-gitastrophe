#pragma once

#include <bits/stdc++.h>

#include "Instance.h"

using namespace std;

namespace two_opt_solver {
struct node {
	pt r0, r1; // robot (-1 means obstacle, 0 means empty, i means i-1th robot
	int t;		 // time of the graph node
	node(pt r0, pt r1, int time = 0) : r0(r0), r1(r1), t(time) {}
	const bool operator<(const node& o) const {
		if (t != o.t)
			return t > o.t;
		if (r0 != o.r0)
			return r0 < o.r0;
		return r1 < o.r1;
	}
};

struct two_opt_solver {
	using state = pair<pt, int>;

	instance& ins;
	vector<vector<int>> moves; // first index is robot (transpose of move matrix)
	unordered_map<state, int> blocked_f;
	unordered_map<state, int> blocked_b;
	set<pt> blocked_s;
	bool spam_stdio;

	two_opt_solver(instance& _ins) : ins(_ins), spam_stdio(true) {
		if (!verify(ins)) {
			cerr << "Requires valid initial sequence" << endl;
			assert(false);
		}

		moves.resize(ins.n);
		// convert dense moves into per-robot moves
		for (int t = 0; t < ins.time; t++) {
			for (int j = 0; j < ins.n; j++) {
				moves[j].push_back(ins.moves[t][j]);
			}
		}

		// for each robot, add moves as obstacles
		for (int i = 0; i < ins.n; i++) {
			add_path(i);
		}

		// block the obstacles
		for (pt p : ins.obstacle) {
			blocked_s.insert(p);
		}
	}

	two_opt_solver(instance& _ins, bool _spam_stdio)
			: ins(_ins), spam_stdio(_spam_stdio) {
		if (!verify(ins)) {
			cerr << "Requires valid initial sequence" << endl;
			assert(false);
		}

		moves.resize(ins.n);
		// convert dense moves into per-robot moves
		for (int t = 0; t < ins.time; t++) {
			for (int j = 0; j < ins.n; j++) {
				moves[j].push_back(ins.moves[t][j]);
			}
		}

		// for each robot, add moves as obstacles
		for (int i = 0; i < ins.n; i++) {
			add_path(i);
		}

		// block the obstacles
		for (pt p : ins.obstacle) {
			blocked_s.insert(p);
		}
	}

	void add_path(int r) {
		pt p = ins.start[r];
		int L = max((int)moves[r].size(), ins.time);
		for (int t = 0; t <= L; t++) {
			if (t < (int)moves[r].size()) {
				blocked_f[state(p, t)] = moves[r][t];
				p = p + dxy[moves[r][t]];
				blocked_b[state(p, t)] = moves[r][t];
			} else {
				blocked_f[state(p, t)] = 0;
			}
		}
		assert(p == ins.target[r]);
	}

	// removes the path for a robot
	void remove_path(int r) {
		pt p = ins.start[r];
		int L = max((int)moves[r].size(), ins.time);
		for (int t = 0; t <= L; t++) {
			if (t < (int)moves[r].size()) {
				blocked_f.erase(state(p, t));
				p = p + dxy[moves[r][t]];
				blocked_b.erase(state(p, t));
			} else {
				blocked_f.erase(state(p, t));
			}
		}
		assert(p == ins.target[r]);
	}

	bool allowed(node s, int d0, int d1) {
		node ns(s.r0 + dxy[d0], s.r1 + dxy[d1], s.t + 1);
		// Obstacles
		if (blocked_s.count(ns.r0) || blocked_s.count(ns.r1))
			return false;

		// Some robot is already there
		if (blocked_f.count(state(ns.r0, ns.t)) ||
				blocked_f.count(state(ns.r1, ns.t)))
			return false;

		// Some robot is there right now and not moving away in the same dir
		if (blocked_f.count(state(ns.r0, s.t)) &&
				blocked_f[state(ns.r0, s.t)] != d0)
			return false;

		if (blocked_f.count(state(ns.r1, s.t)) &&
				blocked_f[state(ns.r1, s.t)] != d1)
			return false;

		// Some robot is moving into our spot and we're not moving away
		if (blocked_b.count(state(s.r0, s.t)) && blocked_b[state(s.r0, s.t)] != d0)
			return false;

		if (blocked_b.count(state(s.r1, s.t)) && blocked_b[state(s.r1, s.t)] != d1)
			return false;

		// The two robots interfere with each other
		if (abs(s.r0 - s.r1) == 1 && abs(ns.r0 - ns.r1) <= 1 && d0 != d1)
			return false;

		// The two robots collide
		if (ns.r0 == ns.r1)
			return false;

		return true;
	}

	// Solves the optimization problem for two robots
	void solve_two(int r0, int r1, double time_left) {
		pt t0 = ins.target[r0], t1 = ins.target[r1];

		// remove the two robots' paths from moveset
		remove_path(r0);
		remove_path(r1);

		map<node, int> dist;
		map<node, pair<int, int>> par;

		auto comp = [&](const node& u, const node& v) {
			int s0 = dist[u] + abs(u.r0 - t0) + abs(u.r1 - t1);
			int s1 = dist[v] + abs(v.r0 - t0) + abs(v.r1 - t1);
			if (s0 != s1)
				return s0 < s1;
			return u < v;
		};

		node s(ins.start[r0], ins.start[r1], 0);
		set<node, decltype(comp)> q(comp);
		dist[s] = 0;
		q.insert(s);

		// Max time for search is remaining time
		auto start_time = clock();
		bool found = false;
		while (clock() - start_time < CLOCKS_PER_SEC * time_left) {
			s = *q.begin();
			q.erase(q.begin());
			if (s.r0 == t0 && s.r1 == t1 && s.t >= ins.time) {
				if (spam_stdio) {
					cerr << "Path found for " << ins.start[r0] << " " << ins.start[r1]
							 << "!" << endl;
					cerr << "Nodes expanded: " << dist.size() << endl;
				}
				found = true;
				break;
			}
			if (s.t >= ins.time)
				continue;

			for (int d0 = 0; d0 <= 4; d0++) {
				for (int d1 = 0; d1 <= 4; d1++) {
					if (allowed(s, d0, d1)) {
						node ns(s.r0 + dxy[d0], s.r1 + dxy[d1], s.t + 1);
						int ndist = dist[s] + (bool(d0) + bool(d1));
						if (!dist.count(ns) || ndist < dist[ns]) {
							q.erase(ns);
							dist[ns] = ndist;
							par[ns] = make_pair(d0, d1);
							q.insert(ns);
						}
					}
				}
			}
		}
		if (!found) {
			add_path(r0);
			add_path(r1);
			return;
		}

		moves[r0].clear();
		moves[r1].clear();
		while (s.t != 0) {
			auto move = par[s];
			moves[r0].push_back(move.first);
			moves[r1].push_back(move.second);
			s.r0 = s.r0 - dxy[move.first];
			s.r1 = s.r1 - dxy[move.second];
			s.t--;
		}
		reverse(moves[r0].begin(), moves[r0].end());
		reverse(moves[r1].begin(), moves[r1].end());
		// re-add robot paths
		add_path(r0);
		add_path(r1);
	}

	bool optimize(int seconds) {
		if (spam_stdio)
			cerr << "Given a compute budget of " << seconds << " seconds." << endl;
		auto ti = clock();

		// int sum;
		// map<int, pair<int, int>> dis;
		// auto resample = [&](){
		// 	dis.clear();
		// 	sum = 0;
		// 	for (int i = 0; i < ins.n; i++) {
		// 		for (int j = 0; j < ins.n; j++) {
		// 			int count = 0;
		// 			pt p = ins.start[i], q = ins.start[j];
		// 			for (int k = 0; k < (int) min(moves[i].size(), moves[j].size());
		// k++) { 				p = p + dxy[moves[i][k]]; 				q = q +
		// dxy[moves[j][k]]; 				if (abs(p-q) == 1
		// && moves[i][k] + moves[j][k] != 0) { 					count++;
		// 				}
		// 			}
		// 			sum += count;
		// 			dis[sum] = make_pair(i, j);
		// 		}
		// 	}
		// };
		// resample();
		// set<pair<int, int>> done;
		vector<pair<int, int>> pairs;
		for (int i = 0; i < ins.n; i++) {
			for (int j = i+1; j < ins.n; j++) {
				pairs.push_back({i, j});
			}
		}
		srand(time(0));
		random_shuffle(pairs.begin(), pairs.end());

		int r = 0;
		srand(time(NULL));
		while (r < (int) pairs.size() && clock() - ti < seconds * CLOCKS_PER_SEC) {
			int a = pairs[r].first, b = pairs[r].second;
			r++;

			// reoptimize wrt to two robots
			double curr_time = (clock() - ti) / double(CLOCKS_PER_SEC);
			solve_two(a, b, seconds - curr_time);
		}

		// Clean-up moves
		if (spam_stdio)
			cerr << "Old makespan: " << ins.time << endl;
		ins.time = 0;
		for (int i = 0; i < ins.n; i++) {
			ins.time = max(ins.time, (int)moves[i].size());
		}
		ins.moves.resize(ins.time);
		if (spam_stdio)
			cerr << "New makespan: " << ins.time << endl;

		// Transpose moves back into time-major
		for (int i = 0; i < ins.n; i++) {
			for (int t = 0; t < ins.time; t++) {
				if (ins.moves[t].empty())
					ins.moves[t].resize(ins.n);

				if (t >= (int)moves[i].size()) {
					ins.moves[t][i] = 0;
				} else {
					ins.moves[t][i] = moves[i][t];
				}
			}
		}

		bool cont;
		do {
			cont = false;
			if (accumulate(ins.moves.back().begin(), ins.moves.back().end(), 0) ==
					0) {
				ins.moves.pop_back();
				ins.time--;
				cont = true;
			}
		} while (cont);

		return true;
	}
};

// returns if improvement found
bool run(instance& ins, int seconds, bool spam_stdio = true) {
	two_opt_solver gd(ins, spam_stdio);
	bool improve = gd.optimize(seconds);
	return improve;
}

} // namespace two_opt_solver
