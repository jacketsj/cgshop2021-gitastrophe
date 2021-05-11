#pragma once

#include <bits/stdc++.h>

#include "Instance.h"

using namespace std;

namespace two_perm_solver {
struct node {
	pt r; // robot (-1 means obstacle, 0 means empty, i means i-1th robot
	int t;		 // time of the graph node
	node(pt r, int time = 0) : r(r), t(time) {}
	const bool operator<(const node& o) const {
		if (t != o.t)
			return t > o.t;
		return r < o.r;
	}
};

struct two_perm_solver {
	using state = pair<pt, int>;

	instance& ins;
	vector<vector<int>> moves; // first index is robot (transpose of move matrix)
	unordered_map<state, int> blocked_f;
	unordered_map<state, int> blocked_b;
	set<pt> blocked_s;
	bool spam_stdio;

	two_perm_solver(instance& _ins) : ins(_ins), spam_stdio(true) {
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

	two_perm_solver(instance& _ins, bool _spam_stdio)
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

	bool allowed(node s, int d) {
		node ns(s.r + dxy[d], s.t + 1);
		// Obstacles
		if (blocked_s.count(ns.r))
			return false;

		// Some robot is already there
		if (blocked_f.count(state(ns.r, ns.t)))
			return false;

		// Some robot is there right now and not moving away in the same dir
		if (blocked_f.count(state(ns.r, s.t)) &&
				blocked_f[state(ns.r, s.t)] != d)
			return false;

		// Some robot is moving into our spot and we're not moving away
		if (blocked_b.count(state(s.r, s.t)) && blocked_b[state(s.r, s.t)] != d)
			return false;

		return true;
	}

	int actual_time(const vector<int>& moveset) {
		int T = moveset.size();
		while (T > 0 && moveset[T-1] == 0)
			T--;
		return T;
	}

	// Find path thats near the old path
	// (must evaluate to true on "in_bounds")
	vector<int> find_path(int r, int maxMS, double time_left, map<pt, bool> in_bounds) {
		map<node, int> dist;
		map<node, int> par;

		auto t = ins.target[r];
		auto comp = [&](const node& u, const node& v) {
			int s0 = dist[u] + abs(u.r - t);
			int s1 = dist[v] + abs(v.r - t);
			if (s0 != s1)
				return s0 < s1;
			return u < v;
		};

		node s(ins.start[r], 0);
		set<node, decltype(comp)> q(comp);
		dist[s] = 0;
		q.insert(s);

		// Max time for search is remaining time
		auto start_time = clock();
		bool found = false;
		while (!q.empty() && clock() - start_time < CLOCKS_PER_SEC * time_left) {
			s = *q.begin();
			q.erase(q.begin());
			if (s.r == t && s.t >= ins.time) {
				found = true;
				//cerr << "Nodes expanded: " << dist.size() << endl;
				break;
			}
			if (s.t >= ins.time) {
				continue;
			}

			// cerr << s.r << " " << s.t << endl;
			for (int d = 0; d <= 4; d++) {
				if (allowed(s, d)) {
					node ns(s.r + dxy[d], s.t + 1);
					if (!in_bounds.count(ns.r))
						continue;

					int ndist = dist[s] + 1;
					if (!dist.count(ns) || ndist < dist[ns]) {
						q.erase(ns);
						dist[ns] = ndist;
						par[ns] = d;
						q.insert(ns);
					}
				}
			}
		}

		if (!found) {
			// cerr << "Path not found!" << endl;
			return vector<int>();
		}
		
		// cerr << "Found path!" << endl;
		vector<int> res;
		while (s.t != 0) {
			auto move = par[s];
			res.push_back(move);
			s.r = s.r - dxy[move];
			s.t--;
		}
		reverse(res.begin(), res.end());
		
		// if (spam_stdio) 
		// 	cerr << "Actual time: " << actual_time(res) << endl;
		if (actual_time(res) >= maxMS)
			return vector<int>();

		return res;
	}

	map<pt, bool> create_bounds(pt p, const vector<int>& moveset, int R = 8) {
		map<pt, bool> res;
		for (int i = -1; i < (int) moveset.size(); i++) {
			if (i >= 0) p = p + dxy[moveset[i]];
			for (int x = p.x - R/2; x <= p.x + R/2; x++) {
				for (int y = p.y - R/2; y <= p.y + R/2; y++) {
					res[pt(x, y)] = true;
				}
			}
		}
		return res;
	}

	// Tries to swap the order the two paths go in
	// i.e. freezes the first path and then puts down the second
	// or freezes the second and then puts down the first.
	bool solve_two(int r0, int r1, double time_left) {
		int L0 = actual_time(moves[r0]), L1 = actual_time(moves[r1]);
		int prevMS = max(L0, L1);
		if (L0 < L1)
			swap(r0, r1);

		// if (spam_stdio) {
		// 	cerr << "Trying to find path for " << ins.start[r0] << " " << ins.start[r1] << endl;
		// 	cerr << "Current pair-span: " << prevMS << endl;
		// }
		
		// Save the previous moves
		auto pmove0 = moves[r0], pmove1 = moves[r1];

		// Now lets try putting down the longer path first
		// and see if this decreases the makespan

		// remove the two robots' paths from moveset
		remove_path(r0);
		remove_path(r1);

		moves[r0] = find_path(r0, prevMS, time_left, create_bounds(ins.start[r0], pmove0));
		int span = -1;

		if (moves[r0].empty())
			goto fail;
		
		add_path(r0);
		moves[r1] = find_path(r1, prevMS, time_left, create_bounds(ins.start[r1], pmove1));

		// New pair-span
		span = max(actual_time(moves[r0]), actual_time(moves[r1]));

		// If not feasible, reverse things
		if (moves[r1].empty()) {
			remove_path(r0);

			fail:
			moves[r0] = pmove0;
			moves[r1] = pmove1;
			add_path(r0);
			add_path(r1);

			// if (spam_stdio) 
			// 	cerr << "Failed to find better span. Swap produced: " << span << endl;
			return false;
		} else {
			// Otherwise we add the new path
			add_path(r1);
			if (spam_stdio)
				cerr << "Better span found: " << span << endl;
			return true;
		}
	}

	bool optimize(int seconds) {
		if (spam_stdio)
			cerr << "Given a compute budget of " << seconds << " seconds." << endl;
		auto ti = clock();

		start:
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
		bool changed = false;
		while (r < (int) pairs.size() && clock() - ti < seconds * CLOCKS_PER_SEC) {
			int a = pairs[r].first, b = pairs[r].second;
			r++;

			if (r % (pairs.size() / 10) == 1)
				cerr << 100 * r / pairs.size() << " percent done." << endl;

			// reoptimize wrt to two robots
			double curr_time = (clock() - ti) / double(CLOCKS_PER_SEC);
			changed = changed || solve_two(a, b, seconds - curr_time);
		}

		// Clean-up moves
		if (spam_stdio)
			cerr << "Old makespan: " << ins.time << endl;
		ins.time = 0;
		for (int i = 0; i < ins.n; i++) {
			ins.time = max(ins.time, (int)moves[i].size());
		}
		ins.moves.resize(ins.time);

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


		if (spam_stdio)
			cerr << "New makespan: " << ins.time << endl;

		if (changed && clock() - ti < seconds * CLOCKS_PER_SEC) {
			cerr << "Extra time leftover. Restarting process..." << endl;
			goto start;
		}

		return true;
	}
};

// returns if improvement found
bool run(instance& ins, int seconds, bool spam_stdio = true) {
	two_perm_solver gd(ins, spam_stdio);
	bool improve = gd.optimize(seconds);
	return improve;
}

} // namespace two_perm_solver
