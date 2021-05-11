#pragma once

#include <bits/stdc++.h>

#include "Instance.h"

using namespace std;

namespace k_perm_solver {
struct node {
	pt r;	// robot (-1 means obstacle, 0 means empty, i means i-1th robot
	int t; // time of the graph node
	node(pt r, int time = 0) : r(r), t(time) {}
	const bool operator<(const node& o) const {
		if (t != o.t)
			return t > o.t;
		return r < o.r;
	}
	bool operator==(const node& o) const { return tie(r, t) == tie(o.r, o.t); }
};
} // namespace k_perm_solver

namespace std {
template <> struct hash<k_perm_solver::node> {
	size_t operator()(const k_perm_solver::node& n) const {
		return (hash<pt>()(n.r) << 20) ^ n.t;
	}
};
} // namespace std

namespace k_perm_solver {
struct k_perm_solver {
	using state = pair<pt, int>;

	static constexpr int GRID = 500;
	static constexpr int OFF = 200;
  template <class V>
  struct arrmap {
    vector<array<array<V, GRID>, GRID>> arr;
    arrmap(int t): arr(t+1) {
      for (auto& vv : arr) {
        for (auto& v : vv) {
          v.fill((V) -1);
        }
      }
    }
    V& operator[](const state& s) {
      return arr[s.second][s.first.x+OFF][s.first.y+OFF];
    }
    V operator[](const state& s) const {
      return arr[s.second][s.first.x+OFF][s.first.y+OFF];
    }
    bool count(const state& s) const {
      return (*this)[s] != -1;
    }
    void erase(const state& s) {
      (*this)[s] = -1;
    }
  };

	instance& ins;
	vector<vector<int>> moves; // first index is robot (transpose of move matrix)
	vector<vector<int>> closeness; // 2d array of robot-robot path closeness
	arrmap<char> blocked_f;
	arrmap<short> blocked_r;
	arrmap<char> blocked_b;
	vector<bitset<GRID>> blocked_s;
	// set<pt> blocked_s;
	bool makespan;
	bool spam_stdio;

	/*
		k_perm_solver(instance& _ins, bool _makespan)
			: ins(_ins), blocked_s(1000), makespan(_makespan), spam_stdio(false) {
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
				blocked_s[p.x+OFF][p.y+OFF] = 1;
			}
		}*/

	k_perm_solver(instance& _ins, bool _makespan, bool _spam_stdio)
			: ins(_ins), blocked_f(2*ins.time), blocked_r(2*ins.time), blocked_b(2*ins.time), blocked_s(GRID), makespan(_makespan),
				spam_stdio(_spam_stdio)  {
		if (!verify(ins)) {
			cerr << "Requires valid initial sequence" << endl;
			assert(false);
		}

		// allow for increasing makespan
		ins.time *= 1.1;
		ins.moves.resize(ins.time, vector<int>(ins.n));

		moves.resize(ins.n);
		// convert dense moves into per-robot moves
		for (int t = 0; t < ins.time; t++) {
			for (int j = 0; j < ins.n; j++) {
				moves[j].push_back(ins.moves[t][j]);
			}
		}
		// make closeness array
		closeness.resize(ins.n);
		for (int i = 0; i < ins.n; i++) {
			closeness[i].resize(ins.n);
		}

		// for each robot, add moves as obstacles
		for (int i = 0; i < ins.n; i++) {
			add_path(i);
		}

		// block the obstacles
		for (pt p : ins.obstacle) {
			assert(0 <= p.x + OFF && p.x + OFF < GRID && 0 <= p.y + OFF &&
						 p.y + OFF < GRID);
			blocked_s[p.x + OFF][p.y + OFF] = 1;
		}
	}

	void add_closeness(int r, pt p, int t) {
		for (int d = 0; d <= 4; d++) {
			pt op = p + dxy[d];
			if (blocked_f.count(state(op, t))) {
				int other_r = blocked_r[state(op, t)];
        if (r < 0 || r >= ins.n) {
          cerr << "BAD r " << r << " vs " << ins.n << endl;
        }
        if (other_r < 0 || other_r >= ins.n) {
          cerr << "BAD other_r " << other_r << " vs " << ins.n << endl;
        }
        assert(r != -1);
        assert(r < ins.n);
        assert(other_r != -1);
        assert(other_r < ins.n);
				if (other_r == r)
					continue;
				closeness[other_r][r]++;
				closeness[r][other_r]++;
			}
		}
	}
	void remove_closeness(int r, pt p, int t) {
		for (int d = 0; d <= 4; d++) {
			pt op = p + dxy[d];
			if (blocked_f.count(state(op, t))) {
				int other_r = blocked_r[state(op, t)];
				if (other_r == r)
					continue;
				closeness[other_r][r]--;
				closeness[r][other_r]--;
			}
		}
	}

	void add_path(int r) {
		pt p = ins.start[r];
		int L = max((int)moves[r].size(), ins.time);
		for (int t = 0; t <= L; t++) {
			if (t < (int)moves[r].size()) {
				blocked_f[state(p, t)] = moves[r][t];
				blocked_r[state(p, t)] = r;
				add_closeness(r, p, t);
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
				blocked_r.erase(state(p, t));
				remove_closeness(r, p, t);
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
		if (blocked_s[ns.r.x + OFF][ns.r.y + OFF])
			return false;

		// Some robot is already there
		if (blocked_f.count(state(ns.r, ns.t)))
			return false;

		// Some robot is there right now and not moving away in the same dir
		if (blocked_f.count(state(ns.r, s.t)) && blocked_f[state(ns.r, s.t)] != d)
			return false;

		// Some robot is moving into our spot and we're not moving away
		if (blocked_b.count(state(s.r, s.t)) && blocked_b[state(s.r, s.t)] != d)
			return false;

		return true;
	}

	int actual_time(const vector<int>& moveset) {
		if (makespan) {
			int T = moveset.size();
			while (T > 0 && moveset[T - 1] == 0)
				T--;
			return T;
		} else {
			int d = 0;
			for (int x : moveset) {
				d += (x != 0);
			}
			return d;
		}
	}

	// Find path thats near the old path
	// (must evaluate to true on "in_bounds")
	int num_paths = 0;
	vector<int> find_path(int r, int maxMS, double time_left,
												const pair<int, vector<vector<int>>>& in_bounds_p) {
		num_paths++;
    const auto& [mx_id, in_bounds] = in_bounds_p;
		constexpr static int INF = 0x3f3f3f3f;
		vector dist(ins.time + 1, vector(mx_id + 1, INF));
		vector par(ins.time + 1, vector(mx_id + 1, -1));
		auto get = [&](const pt& p) { return in_bounds[p.x + OFF][p.y + OFF]; };
		// unordered_map<node, int> dist;
		// unordered_map<node, int> par;

		auto t = ins.target[r];
		/*
		auto comp = [&](const node& u, const node& v) {
			int s0 = dist[u] + abs(u.r - t);
			int s1 = dist[v] + abs(v.r - t);
			if (s0 != s1)
				return s1 < s0;
			return v < u;
		};*/

		node s(ins.start[r], 0);
		vector<vector<node>> q(ins.time + 1);
		// priority_queue<node, vector<node>, decltype(comp)> q(comp);
		dist[s.t][get(s.r)] = 0;
		q[abs(s.r - t)].push_back(s);

		// Max time for search is remaining time
		auto start_time = clock();
		int iter_count = 0;
		bool found = false;
		for (int curdist = abs(s.r - t); curdist <= ins.time; curdist++) {
			random_shuffle(q[curdist].begin(), q[curdist].end());
			while (!q[curdist].empty()) {
				if (iter_count++ % 1024 == 0 &&
						clock() - start_time >= CLOCKS_PER_SEC * time_left)
					goto end;
				s = q[curdist].back();
				q[curdist].pop_back();
				if (s.r == t && s.t >= ins.time) {
					found = true;
					// cerr << "Nodes expanded: " << dist.size() << endl;
					goto end;
				}
				if (s.t >= ins.time) {
					continue;
				}
				int cur = dist[s.t][get(s.r)];
				if (cur + abs(s.r - t) != curdist)
					continue;

				// cerr << s.r << " " << s.t << endl;
				for (int d = 4; d >= 0; d--) {
					if (allowed(s, d)) {
						node ns(s.r + dxy[d], s.t + 1);
						if (get(ns.r) == -1)
							continue;

						int ndist = cur + (makespan || d != 0);
						int pri = ndist + abs(ns.r - t);
						if (pri <= ins.time && (ndist < dist[ns.t][get(ns.r)])) {
							dist[ns.t][get(ns.r)] = ndist;
							par[ns.t][get(ns.r)] = d;
							q[pri].push_back(ns);
						}
					}
				}
			}
		}
	end:

		if (!found) {
			// cerr << "Path not found!" << endl;
			return vector<int>();
		}

		// cerr << "Found path!" << endl;
		vector<int> res;
		while (s.t != 0) {
			auto move = par[s.t][get(s.r)];
			res.push_back(move);
			s.r = s.r - dxy[move];
			s.t--;
		}
		reverse(res.begin(), res.end());

		// if (spam_stdio)
		// 	cerr << "Actual time: " << actual_time(res) << endl;
		if (makespan && actual_time(res) >= maxMS)
			return vector<int>();

		return res;
	}

	pair<int, vector<vector<int>>> create_bounds(pt p, const vector<int>& moveset,
																		int R = 8) {
		pair res(0, vector(GRID, vector<int>(GRID, -1)));
    auto& id = res.first;
		for (int i = -1; i < (int)moveset.size(); i++) {
			if (i >= 0)
				p = p + dxy[moveset[i]];
			for (int x = p.x - R / 2; x <= p.x + R / 2; x++) {
				for (int y = p.y - R / 2; y <= p.y + R / 2; y++) {
					if (res.second[x + OFF][y + OFF] == -1) {
						res.second[x + OFF][y + OFF] = id++;
					}
				}
			}
		}
		return res;
	}

	// Tries to swap the order the two paths go in
	// i.e. freezes the first path and then puts down the second
	// or freezes the second and then puts down the first.
	bool solve_k(vector<int> r, double time_left, int R) {
		vector<pair<int, int>> L(r.size());
		int prevMS = 0;
		for (int i = 0; i < (int)r.size(); i++) {
			L[i].first = actual_time(moves[r[i]]);
			L[i].second = r[i];
			if (makespan) {
				prevMS = max(prevMS, L[i].first);
			} else {
				prevMS += L[i].first;
			}
		}
		sort(L.rbegin(), L.rend());

		for (int i = 0; i < (int)r.size(); i++) {
			r[i] = L[i].second;
		}

		// Save the previous moves
		vector<vector<int>> pmove(r.size());
		for (int i = 0; i < (int)r.size(); i++) {
			pmove[i] = moves[r[i]];
			remove_path(r[i]);
		}

		int span = -1;
		int dist = 0;
		int ndone = 0;
		while (ndone < (int)r.size()) {
			auto moveset =
					find_path(r[ndone], prevMS, time_left,
										create_bounds(ins.start[r[ndone]], pmove[ndone], R));

			if (moveset.empty())
				break;

			moves[r[ndone]] = moveset;
			add_path(r[ndone]);
			ndone++;
		}

		// If not feasible, reverse things
		if (ndone < (int)r.size()) {
		fail:
			for (int i = 0; i < ndone; i++) {
				remove_path(r[i]);
			}

			for (int i = 0; i < (int)r.size(); i++) {
				moves[r[i]] = pmove[i];
				add_path(r[i]);
			}

			// if (spam_stdio)
			// 	cerr << "Failed to find better span. Swap produced: " << span << endl;
			return false;
		} else {
			// New pair-span
			if (makespan) {
				for (int i = 0; i < (int)r.size(); i++) {
					span = max(span, actual_time(moves[r[i]]));
				}
			} else {
				for (int i = 0; i < (int)r.size(); i++) {
					dist = dist + actual_time(moves[r[i]]);
				}
				if (dist >= prevMS) {
					goto fail;
				}
			}

			if (spam_stdio)
				cerr << "Better span found: " << prevMS << "->" << span << endl;
			return true;
		}
	}

	bool optimize(int seconds, int k, int R) {
		if (spam_stdio)
			cerr << "Given a compute budget of " << seconds
					 << " seconds. k, R = " << k << ", " << R << endl;
		auto ti = clock();

		srand(time(NULL));

    // recompute distances every time because moves changes
    vector<int> dist, dist_orig;
    vector<pair<int, int>> ranked;
    int last = 0;
    for (int i = 0; i < ins.n; i++) {
      int wt = actual_time(moves[i]);
      dist.push_back(wt * wt + last);
      last = dist.back();
      ranked.push_back({wt, i});
    }
    dist_orig = dist;
    sort(ranked.rbegin(), ranked.rend());

    		int lol = 0;
		while (clock() - ti < seconds * CLOCKS_PER_SEC) {
			vector<int> samples;

			dist = dist_orig;
			lol %= ins.n;
			bool sample_wrt_closeness = true;
			if (sample_wrt_closeness) {
				// have some nonzero probability to pick other entities
				int SCALING = sqrt(ins.n) + 1;
				vector<int> weights(ins.n, 1);
				// first sample is based on dist
				while ((int)samples.size() < k) {
					int id;
					if (false && samples.empty()) {
						id = ranked[lol++].second;
					} else {
					 	id = lower_bound(dist.begin(), dist.end(), rand() % dist.back()) -
									 dist.begin();
					}
					if (find(samples.begin(), samples.end(), id) == samples.end()) {
						samples.push_back(id);

						// update weight array
						for (int i = 0; i < ins.n; i++) {
							weights[i] += closeness[id][i] * SCALING;
						}

						// make sure we never sample things we sampled
						for (int i : samples) {
							weights[i] = 0;
						}

						// update distribution we draw from
						int last = 0;
						for (int i = 0; i < ins.n; i++) {
							dist[i] = weights[i] + last;
							last = dist[i];
						}
					}
				}
			} else {
				while ((int)samples.size() < k) {
					int id = lower_bound(dist.begin(), dist.end(), rand() % dist.back()) -
									 dist.begin();
					if (find(samples.begin(), samples.end(), id) == samples.end()) {
						samples.push_back(id);
					}
				}
			}

			// reoptimize wrt to two robots
			double curr_time = (clock() - ti) / double(CLOCKS_PER_SEC);
			solve_k(samples, seconds - curr_time, R);
		}

		// Clean-up moves
		if (spam_stdio)
			cerr << "Old makespan: " << ins.time/2 << endl;
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

		return true;
	}
};

// returns if improvement found
bool run(instance& ins, int seconds, bool makespan, int k = 3, int R = 8,
				 bool spam_stdio = false) {
	k_perm_solver gd(ins, makespan, spam_stdio);
	bool improve = gd.optimize(seconds, k, R);
	//cerr << gd.num_paths << " paths\n";
	return improve;
}

} // namespace k_perm_solver
