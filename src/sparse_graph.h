#pragma once

#include <algorithm>
#include <bits/stdc++.h>
#include <cassert>
#include <iostream>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>

#include "Instance.h"
#include "sorted_set.h"

using namespace std;

/* NOTE THIS IS DUPLICATE OF graph.h BUT USES SPARSE REPRESENTATION OF MOVES
 * Usage:
 *  -   initialize with `graph g(ins, [max time horizon]);` default max time
 * horizon is ins.maxt
 *  -   if you make changes to the graph that you want to push to the instace,
 *        call `g.update_instance();`
 *  -   if you make changes to underlying instance you want to update graph
 * with, call `g.build([max time horizon])`
 *  -   g.robots() stores the last known location of the robot
 */

namespace sparse_graph {
struct node {
	int r; // robot (-1 means obstacle, 0 means empty, i means i-1th robot
	int d; // direction that robot went in
	int t; // time of the graph node
	node(int robot = 0, int dir = 0, int time = 0) : r(robot), d(dir), t(time) {}
	void reset() { r = d = t = 0; }
	bool operator==(const node& o) const {
		return tie(r, d, t) == tie(o.r, o.d, o.t);
	}
} null(0, 0, 0);

inline bool operator<(const node& a, const node& b) { return a.t < b.t; }

struct sparse_graph {

	// Assume coordinates range from [-OFF, DIM-OFF]
	static const int DIM = 440;
	static const int OFF = 170;
	static const short SINF = 0x3f3f;
	static const int MAXT = 2000;
	int maxt;
	instance& ins;
	// g[p] stores a list of actions, either a move, or a stay still.
	// If last action was to stay still, robot is still there.
	vector<vector<sorted_set<node>>> g;
	vector<vector<char>> obst;
	vector<int> robots; // latest node we have for a robot
	mt19937 rng;

	// initialize graph
	void build(int t = -1, const set<int>& ignore = set<int>()) {
		rng = mt19937(time(0));
		maxt = max(t + 1, (int)ins.moves.size() + 1);
		g.clear();
		mem_dist.clear();
		mem_dist.resize(ins.n);
		g.resize(DIM, vector<sorted_set<node>>(DIM));
		robots.resize(ins.n);
		obst.resize(DIM, vector<char>(DIM));
		// fill obstacles
		for (pt& p : ins.obstacle) {
			// g[p.x + OFF][p.y + OFF].insert(node(-1));
			obst[p.x + OFF][p.y + OFF] = 1;
		}
		// add obstacles around borders
		for (int i = 0; i < DIM; i++) {
			/*
			g[i][0].insert(node(-1));
			g[0][i].insert(node(-1));
			g[DIM-1][i].insert(node(-1));
			g[i][DIM-1].insert(node(-1));*/
			obst[i][0] = 1;
			obst[0][i] = 1;
			obst[DIM - 1][i] = 1;
			obst[i][DIM - 1] = 1;
		}

		// fill paths
		vector<pt> locations = ins.start;
		int curt = 0;
		// add a no-move at the end to indicate robot is still here.
		ins.moves.push_back(vector<int>(ins.n, 0));
		for (auto& step : ins.moves) {
			for (int i = 0; i < ins.n; ++i) {
				// don't add things from the ignore set to the graph at all
				if (ignore.count(i) > 0)
					continue;

				pt p = locations[i];
				robots[i] = add_move(hsh(p, curt), i, step[i]);
				p = p + dxy[step[i]];
				locations[i] = p;
				if (!in_range(p)) {
					cerr << "ERROR : Out of range coordinate considered " << p.x << " "
							 << p.y << endl;
					cerr << "Consider changing limits in the code." << endl;
					assert(false);
				}
			}
			curt++;
		}
		ins.moves.pop_back(); // remove the no move at end we added.
		for (int r = 0; r < ins.n; r++) {
			compute_dist(r);
		}
	}

	sparse_graph(instance& _ins, int _maxt = -1) : ins(_ins) { build(_maxt); }
	sparse_graph(instance& _ins, const vector<size_t>& ignore_v, int _maxt = -1)
			: ins(_ins) {
		set<int> ignore(ignore_v.begin(), ignore_v.end());
		build(_maxt, ignore);
	}

	// hash state to single int for convenience
	inline int hsh(int x, int y, int t = 0) {
		return (x + OFF) + ((y + OFF) << 9) + (t << 18);
	}
	inline int hsh(const pt p, int t = 0) { return hsh(p.x, p.y, t); }

	// unhash state
	inline int get_x(int h) { return (h & ((1 << 9) - 1)) - OFF; }
	inline int get_y(int h) { return ((h & ((1 << 18) - 1)) >> 9) - OFF; }
	inline pt get_p(int h) { return pt(get_x(h), get_y(h)); }
	inline int get_t(int h) { return h >> 18; }

	inline bool is_obstacle(const pt& p) { return obst[p.x + OFF][p.y + OFF]; }
	inline bool in_range(const pt& p) { return !is_obstacle(p); }

	// get last notable move relative to time h
	// Requires that the point is in_range
	inline node get_last_move(int h) {
		pt p = get_p(h);
		if (g[p.x + OFF][p.y + OFF].empty())
			return null;
		int t = get_t(h);
		// search through action list at p
		auto pt = g[p.x + OFF][p.y + OFF].lower_bound(node(0, 0, t));
		if (pt != g[p.x + OFF][p.y + OFF].end() && pt->t == t)
			return *pt;
		if (pt == g[p.x + OFF][p.y + OFF].begin())
			return null; // no robot was here ever
		pt = prev(pt);
		if (pt->d == 0)
			return node(pt->r, 0, t); // the last robot that was here hasn't moved yet
		return null;
	}

	inline bool is_last_move(int h) {
		pt p = get_p(h);
		int t = get_t(h);
		return g[p.x + OFF][p.y + OFF].lower_bound(node(0, 0, t)) ==
					 g[p.x + OFF][p.y + OFF].end();
	}

	// adds move at pos-time h, robot r, direction d, assumes move is valid
	// return last state recorded
	inline int add_move(int h, int r, int d) {
		int t = get_t(h);
		pt p = get_p(h);
		if (d == 0) {
			node last = get_last_move(h);
			if (last.r == r + 1 && last.d == 0) {
				return hsh(p, last.t); // last move was same robot still
			}
		}
		maxt = max(maxt, t + 1);
		g[p.x + OFF][p.y + OFF].insert(node(r + 1, d, t));
		return hsh(p, t);
	}

	// bad pun
	// assumes move was in the graph
	inline void re_move(int h, int r, int d) {
		int t = get_t(h);
		pt p = get_p(h);
		node last = get_last_move(h);
		if (d == 0) {
			if (last.r == r && last.d == 0 && last.t < t) {
				return; // last move was same robot still
			}
		}
		// technically maxt may decrease, but we can't really tell from here
		g[p.x + OFF][p.y + OFF].erase(last);
	}

	// remove path taken of robot r, returns length of path for reasons
	int remove_path(int r, int time = 0) {
		pt p = ins.start[r];
		int lastt = get_t(robots[r]);
		vector<int> mv;
		int moves = 0;
		for (int t = 0; t <= lastt; t++) {
			int h = hsh(p, t);
			int dir = get_last_move(h).d;
			mv.push_back(dir);
			p = p + dxy[dir];
		}
		for (int t = lastt; t >= time; t--) {
			int h = hsh(p, t);
			re_move(h, r, mv[t]);
			if (mv[t]) {
				moves++;
			}
			if (t > 0)
				p = p - dxy[mv[t - 1]];
		}
		robots[r] = hsh(p, time);
		return moves;
	}

	// memoized robotless distance function
	vector<array<array<short, DIM>, DIM>> mem_dist;
	inline int dist(pt p, int r) {
		// if (mem_dist[r][p.x+OFF][p.y+OFF] != INF)
		return mem_dist[r][p.x + OFF][p.y + OFF];
	}
	array<pt, DIM * DIM> q;
	void compute_dist(int r) {
		auto& dist = mem_dist[r];
		for (auto& v : dist) {
			v.fill(SINF);
		}
		pt start = ins.target[r];
		dist[start.x + OFF][start.y + OFF] = 0;
		int qs = 0, qt = 0;
		q[qt++] = start;
		while (qs < qt) {
			pt p = q[qs++];
			for (int i = 1; i <= 4; i++) {
				pt np = p + dxy[i];
				if (is_obstacle(np))
					continue;
				if (dist[np.x + OFF][np.y + OFF] == SINF) {
					dist[np.x + OFF][np.y + OFF] = dist[p.x + OFF][p.y + OFF] + 1;
					q[qt++] = np;
				}
			}
		}
	}

	// checks if moving from p -> np at time t is valid (assumes moving robot is
	// not in the graph)
	bool valid_move(const pt& p, const pt& np, const int t,
									bool notsosure = false) {
		if (t + 1 == (int)maxt && !notsosure)
			return true;
		// check whether p->np is valid
		int ch = hsh(np, t);
		int nh = hsh(np, t + 1);
		pt dir = np - p;
		node gch = get_last_move(ch);
		node gnh = get_last_move(nh);
		if (gnh.r != 0) { // some other robot or obstacle is there at next time step
			return false;
		}
		if (gch.r != 0) {					 // some other robot or obstacle is there now
			if (dxy[gch.d] != dir) { // if its not going the same direction as us
				return false;
			}
		}
		// check whether something moving ->p is still valid
		nh = hsh(p, t + 1);
		gnh = get_last_move(nh);
		if (gnh.r != 0) { // some robot is following behind, into the same spot
			if (gnh.r == -1) {
				cerr << "wait how was this robot on an obstacle ??? " << endl;
				assert(false);
			}
			int otherd = -1;

			for (int d = 1; d < 5; d++) { // figure out where it came from
				if (get_last_move(hsh(p + dxy[d], t)).r == gnh.r) {
					otherd = opposite_dir[d];
				}
			}
			if (otherd == -1 || dir != dxy[otherd]) {
				return false;
			}
		}
		return true;
	}

	vector<array<array<short, DIM>, DIM>> dis;
	vector<array<array<char, DIM>, DIM>> pre;
	vector<pair<int, pt>> vis;
	bool find_best_randomized(int r, int startt = 0,
														bool ignore_regressions = true,
														bool false_means_fail = false, int MULT = 1) {
		if (dis.empty()) {
			dis.resize(MAXT);
			for (auto& v : dis) {
				memset(v.data(), SINF, sizeof(v));
			}
			pre.resize(MAXT);
			/*
			for (auto& v : pre) {
				memset(v.data(), -1, sizeof(v));
			}*/
		}
		int old_moves = remove_path(r, startt);
		// middle tuple is random
		vector<vector<int>> pq(40 * MAXT);
		/*
		priority_queue<tuple<int, int, int>, vector<tuple<int, int, int>>,
									 greater<tuple<int, int, int>>>
				pq;*/
		int start = robots[r];
		int target = hsh(ins.target[r], maxt - 1); // target hash
		pt s = get_p(start);
		dis[0][s.x + OFF][s.y + OFF] = dist(s, r) * MULT;
		pq[dis[0][s.x + OFF][s.y + OFF]].push_back(start);
		vis = {{0, s}};
		bool found = false;
		for (int curt = 0; curt < int(pq.size()); curt++) {
			shuffle(begin(pq[curt]), end(pq[curt]), rng);
			while (!pq[curt].empty()) {
				int h = pq[curt].back();
				pq[curt].pop_back();
				pt p = get_p(h);
				int t = get_t(h);
				if (p == ins.target[r] && is_last_move(h)) {
					target = hsh(ins.target[r], t);
					found = true;
					goto end;
				}
				if (dis[t][p.x + OFF][p.y + OFF] < curt)
					continue;
				int nt = t + 1;
				for (int dir = 0; dir < 5; dir++) {
					pt np = p + dxy[dir];
					int nh = hsh(np, nt);
					int ncost = curt + 1 + ((dir != 0) - dist(p, r) + dist(np, r)) * MULT;
					if (in_range(np) && dis[t + 1][np.x + OFF][np.y + OFF] > ncost &&
							valid_move(p, np, t)) {
						// assert(ncost < 2*MAXT);
						dis[t + 1][np.x + OFF][np.y + OFF] = ncost;
						vis.emplace_back(t + 1, np);
						pq[ncost].push_back(nh);
						pre[t + 1][np.x + OFF][np.y + OFF] = dir;
					}
				}
			}
		}
	end:
		int moves = 0;
		if (!found) {
			cerr << "NO PATH FOUND, PLEASE FIX YOUR SEED INSTANCE" << endl;
			return false;
		} else {
			// recover path backwards
			robots[r] = target;
			int cur = target;
			vector<int> mvs;
			while (cur != start) {
				pt p = get_p(cur);
				int t = get_t(cur);
				int pdir = pre[t][p.x + OFF][p.y + OFF];
				if (pdir)
					moves++;
				mvs.push_back(pdir);
				cur = hsh(p - dxy[pdir], t - 1);
			}
			reverse(mvs.begin(), mvs.end());
			mvs.push_back(0); // add a no-move at the end
			for (int d : mvs) {
				add_move(cur, r, d);
				pt p = get_p(cur);
				int t = get_t(cur);
				cur = hsh(p + dxy[d], t + 1);
			}
			if (startt == 0 && old_moves > moves) {
				// cerr << "Improved robot r = " << r << ", from " << old_moves
				// 		 << " moves to " << moves << " moves."
				// 		 << "(" << -old_moves + moves << ") for " << ins.name << endl;
			} else if (startt == 0 && old_moves < moves) {
				if (ignore_regressions) {
					// cerr << "Robot regression r = " << r << ", from " << old_moves
					// 		 << " moves to " << moves << " moves." << "(+" << -old_moves +
					// moves << ") " << endl;
				} else {
					cerr << "THIS IS BAD, FAILING robot r = " << r << ", from "
							 << old_moves << " moves to " << moves << " moves."
							 << "(+" << -old_moves + moves << ") " << endl;
					assert(false);
				}
			}
		}
		for (const auto& [t, p] : vis) {
			dis[t][p.x + OFF][p.y + OFF] = SINF;
		}
		return false_means_fail || old_moves != moves;
	}

	bool find_best(int r, int startt = 0, bool ignore_regressions = true) {
		const long long MULT = 10; // travelling has MULTx the cost of staying still
		int old_moves = remove_path(r, startt);
		priority_queue<pair<int, int>, vector<pair<int, int>>,
									 greater<pair<int, int>>>
				pq;
		unordered_map<int, long long> dis; // best distance
		unordered_map<int, int> pre;			 // previous move
		int start = robots[r];
		int target = hsh(ins.target[r], maxt - 1); // target hash
		dis[start] = dist(get_p(start), r) * MULT;
		pq.push({dis[start], start});
		bool found = false;
		while (!pq.empty()) {
			long long cost = pq.top().first;
			int h = pq.top().second;
			pt p = get_p(h);
			int t = get_t(h);
			if (p == ins.target[r] && is_last_move(h)) {
				target = hsh(ins.target[r], t);
				found = true;
				break;
			}
			pq.pop();
			if (dis[h] < cost)
				continue;
			int nt = t + 1;
			for (int dir = 0; dir < 5; dir++) {
				pt np = p + dxy[dir];
				int nh = hsh(np, nt);
				long long ncost =
						cost + 1 + ((dir != 0) - dist(p, r) + dist(np, r)) * MULT;
				if (in_range(np) && (!dis.count(nh) || dis[nh] > ncost) &&
						valid_move(p, np, t)) {
					dis[nh] = ncost;
					pq.push({dis[nh], nh});
					pre[nh] = dir;
				}
			}
		}
		int moves = 0;
		if (!found) {
			cerr << "NO PATH FOUND, PLEASE FIX YOUR SEED INSTANCE" << endl;
			return false;
		} else {
			// recover path backwards
			robots[r] = target;
			int cur = target;
			vector<int> mvs;
			while (cur != start) {
				pt p = get_p(cur);
				int t = get_t(cur);
				int pdir = pre[cur];
				if (pdir)
					moves++;
				mvs.push_back(pdir);
				cur = hsh(p - dxy[pdir], t - 1);
			}
			reverse(mvs.begin(), mvs.end());
			mvs.push_back(0); // add a no-move at the end
			for (int d : mvs) {
				add_move(cur, r, d);
				pt p = get_p(cur);
				int t = get_t(cur);
				cur = hsh(p + dxy[d], t + 1);
			}
			if (startt == 0 && old_moves > moves) {
				cerr << "Improved robot r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves."
						 << "(" << -old_moves + moves << ") for " << ins.name << endl;
			} else if (startt == 0 && old_moves < moves) {
				if (ignore_regressions) {
					// cerr << "Robot regression r = " << r << ", from " << old_moves
					// 		 << " moves to " << moves << " moves." << "(+" << -old_moves +
					// moves << ") " << endl;
				} else {
					cerr << "THIS IS BAD, FAILING robot r = " << r << ", from "
							 << old_moves << " moves to " << moves << " moves."
							 << "(+" << -old_moves + moves << ") " << endl;
					assert(false);
				}
			}
		}
		return old_moves != moves;
	}

	void update_instance() {
		cerr << "UPDATING INSTANCE " << endl;
		ins.moves.clear();
		vector<vector<int>> vv(maxt, vector<int>(ins.n, 0));
		for (int r = 0; r < ins.n; r++) {
			pt pos = ins.start[r];
			int lastt = get_t(robots[r]);
			for (int t = 0; t <= lastt; t++) {
				int d = get_last_move(hsh(pos, t)).d;
				vv[t][r] = d;
				pos = pos + dxy[d];
			}
		}
		vector<int> zeroes(ins.n, 0);
		// trim still frames
		for (auto& v : vv)
			if (v != zeroes)
				ins.moves.push_back(v);
		ins.time = ins.moves.size();
	}
};

} // namespace sparse_graph
