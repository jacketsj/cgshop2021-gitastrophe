#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>
#include <set>
#include <unordered_set>

#include "Instance.h"

using namespace std;

/* Usage:
 *  -   initialize with `graph g(ins, [max time horizon]);` default max time horizon is ins.maxt
 *  -   if you make changes to the graph that you want to push to the instace,
 *        call `g.update_instance();`
 *  -   if you make changes to underlying instance you want to update graph with,
 *        call `g.build([max time horizon])`
 *  -   g.robots() stores the last known location of the robot 
 */


struct node {
	int r; // robot (-1 means obstacle, 0 means empty, i means i-1th robot
	int d; // direction that robot went in
	node(int robot = 0, int dir = 0) : r(robot), d(dir) {}
	void reset() { r = d = 0; }
};

struct graph {
  // Assume coordinates range from [-OFF, DIM-OFF]
	const int DIM = 500;
	const int OFF = 200;
	int maxt;
	instance& ins;
	vector<node> g;
  vector<int> robots; 

	// hash state to single int for convenience
	inline int hsh(int x, int y, int t) {
		return (x + OFF) + (y + OFF) * DIM + t * DIM * DIM;
	}
	inline int hsh(pt p, int t) { return hsh(p.x, p.y, t); }

	// unhash state
	inline int get_x(int h) { return h % DIM - OFF; }
	inline int get_y(int h) { return (h % (DIM * DIM )) / DIM - OFF; }
	inline pt get_p(int h) { return pt(get_x(h), get_y(h)); }
	inline int get_t(int h) { return h / (DIM * DIM); }

	inline bool in_range(const pt& p) {
		if (p.x <= -OFF || p.x >= DIM - OFF || p.y <= -OFF || p.y >= DIM - OFF) {
			return false;
		}
		return true;
	}

  // memoized robotless distance function
  vector<vector<int>> mem_dist;
  int dist(pt p, int r) {
    if(!mem_dist[r].empty()) return mem_dist[r][hsh(p, 0)];
    mem_dist[r].resize(DIM * DIM, INF);
    int start = hsh(ins.target[r], 0);
    mem_dist[r][start] = 0;
    queue<int> q;
    q.push(start);
    while(!q.empty()) {
      int h = q.front();
      q.pop();
      pt p = get_p(h);
      for(int i = 1; i <= 4; i++) {
        pt np = p + dxy[i];
        int nh = hsh(np, 0);
        if (!in_range(np) || g[nh].r==-1) continue;
        if (mem_dist[r][nh] > mem_dist[r][h]+1) {
          mem_dist[r][nh] = mem_dist[r][h]+1;
          q.push(nh);
        }
      }
    }
    return dist(p, r);
  }

	// initialize graph
	void build(int t = -1) {
		maxt = max(t+1, (int)ins.moves.size()+1);
		g.clear();
    mem_dist.clear();
    mem_dist.resize(ins.n);
		g.resize(DIM * DIM * maxt, node());
    robots.resize(ins.n);
		// fill obstacles
		for (pt& p : ins.obstacle) {
			for (int t = 0; t < maxt; t++) {
				g[hsh(p, t)].r = -1;
			}
		}
		// fill paths
		vector<pt> locations = ins.start;
		int curt = 0;
		for (auto& step : ins.moves) {
			for (int i = 0; i < ins.n; ++i) {
				pt p = locations[i];
				g[hsh(p, curt)].r = i + 1;
				g[hsh(p, curt)].d = step[i];
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
		for (; curt < maxt; ++curt) {
			for (int i = 0; i < ins.n; ++i) {
				pt p = locations[i];
				g[hsh(p, curt)].r = i + 1;
				g[hsh(p, curt)].d = 0;
			}
		}
    for (int i = 0; i < ins.n; ++i) {
      robots[i] = hsh(locations[i], maxt-1);
    }
	}

  graph(instance& _ins, int _maxt = -1) : ins(_ins) {
    build(_maxt);
  }

  // checks if moving from p -> np at time t is valid (assumes moving robot is not in the graph)
	bool valid_move(const pt& p, const pt& np, const int t) {
		if (t + 1 == (int)maxt)
			return true;
		// check whether p->np is valid
		int ch = hsh(np, t);
		int nh = hsh(np, t + 1);
		pt dir = np - p;
		if (g[nh].r != 0) { // some other robot or obstacle is there at next time step
			return false;
		}
		if (g[ch].r != 0) {					 // some other robot or obstacle is there now
			if (dxy[g[ch].d] != dir) { // if its not going the same direction as us
				return false;
			}
		}
		// check whether something moving ->p is still valid
		nh = hsh(p, t + 1);
		if (g[nh].r != 0) { // some robot is following behind, into the same spot
			if (g[nh].r == -1) {
				cerr << "wait how was this robot on an obstacle ??? " << endl;
				assert(false);
			}
			int otherd = -1;

			for (int d = 1; d < 5; d++) { // figure out where it came from
				if (g[hsh(p + dxy[d], t)].r == g[nh].r) {
					otherd = opposite_dir[d];
				}
			}
			if (dir != dxy[otherd]) {
				return false;
			}
		}
		return true;
	}

  bool find_best_time(int r, int startt = 0, bool ignore_regressions = false) {
		int old_moves = remove_path(r, startt);
		priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
		vector<int> dis(g.size(), INF); // best distance
		vector<int> pre(g.size(), -1);	// previous move
		int start = robots[r];
		int target = hsh(ins.target[r], maxt - 1); // target hash
		dis[start] = dist(get_p(start), r);
		pq.push({dis[start], start});
		bool found = false;
		while (!pq.empty()) {
			int cost = pq.top().first;
			int h = pq.top().second;
			if (h == target) {
				found = true;
				break;
			}
			pq.pop();
			if (dis[h] < cost) continue;
			pt p = get_p(h);
			int t = get_t(h);
			int nt = t + 1;
			for (int dir = 0; dir < 5; dir++) {
				pt np = p + dxy[dir];
				int nh = hsh(np, nt);
				int ncost = cost + (1 - dist(p, r) + dist(np, r));
				if (nt < maxt && in_range(np) && dis[nh] > ncost && valid_move(p, np, t)) {
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
		} 
    else {
			// recover path backwards
      robots[r] = target;
			g[target].r = r + 1;
      g[target].d = 0;
			int cur = target;
			while (cur != start) {
				pt p = get_p(cur);
				int t = get_t(cur);
				int pdir = pre[cur];
				if (pdir) moves++;
				pt pp = p - dxy[pdir];
				auto pt = t - 1;
				int pcur = hsh(pp, pt);
				g[pcur] = node(r + 1, pdir);
				cur = pcur;
			}
			/*
			if (startt == 0 && old_moves > moves) {
				cerr << "Improved robot r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves." << "(+" << old_moves - moves << ") " << endl;
			} else if (startt == 0 && old_moves < moves) {
        if (ignore_regressions) {
				cerr << "Robot regression r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves." << "(" << old_moves - moves << ") " << endl;
        }
        else {
				cerr << "THIS IS BAD, FAILING robot r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves." << "(" << old_moves - moves << ") " << endl;
				assert(false);
        }
			}*/
		}
		return old_moves > moves;
	}

	// find shortest path for robot r, starting at time t, keeping all else before time t fixed
	// returns if improved
	bool find_best(int r, int startt = 0, bool ignore_regressions = false) {
		int old_moves = remove_path(r, startt);
		priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
		vector<int> dis(g.size(), INF); // best distance
		vector<int> pre(g.size(), -1);	// previous move
		int start = robots[r];
		int target = hsh(ins.target[r], maxt - 1); // target hash
		dis[start] = dist(get_p(start), r);
		pq.push({dis[start], start});
		bool found = false;
		while (!pq.empty()) {
			int cost = pq.top().first;
			int h = pq.top().second;
			if (h == target) {
				found = true;
				break;
			}
			pq.pop();
			if (dis[h] < cost) continue;
			pt p = get_p(h);
			int t = get_t(h);
			pt np = p;
			int nt = t + 1;
			int nh = hsh(np, nt);
			if (nt < maxt && dis[nh] > cost && valid_move(p, np, t)) {
				dis[nh] = cost;
				pre[nh] = 0;
				pq.push({dis[nh], nh});
			}
			for (int dir = 1; dir < 5; dir++) {
				np = p + dxy[dir];
				nh = hsh(np, nt);
				int ncost = cost + (1 - dist(p, r) + dist(np, r));
				if (nt < maxt && in_range(np) && dis[nh] > ncost && valid_move(p, np, t)) {
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
		} 
    else {
			// recover path backwards
      robots[r] = target;
			g[target].r = r + 1;
      g[target].d = 0;
			int cur = target;
			while (cur != start) {
				pt p = get_p(cur);
				int t = get_t(cur);
				int pdir = pre[cur];
				if (pdir) moves++;
				pt pp = p - dxy[pdir];
				auto pt = t - 1;
				int pcur = hsh(pp, pt);
				g[pcur] = node(r + 1, pdir);
				cur = pcur;
			}
			/*
			if (startt == 0 && old_moves > moves) {
				cerr << "Improved robot r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves." << endl;
			} else if (startt == 0 && old_moves < moves) {
        if (ignore_regressions) {
				//cerr << "Robot regression r = " << r << ", from " << old_moves
				//		 << " moves to " << moves << " moves." << endl;
        }
        else {
				cerr << "THIS IS BAD, FAILING robot r = " << r << ", from " << old_moves
						 << " moves to " << moves << " moves." << endl;
				assert(false);
        }
			}*/
		}
		return old_moves > moves;
	}

  // Verifies if path taken by robot r goes from source to target is valid
  // currently broken, please don't use!
  bool verify_path(int r, bool check_target = false) {
		pt p = ins.start[r];
    int lastt = get_t(robots[r]);
		for (int t = 0; t < lastt; t++) {
			if (g[hsh(p, t)].r != r + 1) {
        // something seriously went wrong
        cerr << "INVALID STATE AT " << t << " FOR ROBOT " << r << endl;
        assert(false);
      }
			int dir = g[hsh(p, t)].d;
      if (!valid_move(p, p + dxy[dir], t)) {
        return false;
      }
			if (dir) {
				p = p + dxy[dir];
			}
		}
		return (!check_target || p == ins.target[r]);
  }

	// remove path taken of robot r, returns length of path for reasons
	int remove_path(int r, int time = 0) {
		pt p = ins.start[r];
    int lastt = get_t(robots[r]);
		int moves = 0;
		vector<int> mv;
		for (int t = 0; t <= lastt; t++) {
			int dir = g[hsh(p, t)].d;
      if (t >= time) {
        g[hsh(p, t)].reset();
        if (t == time) robots[r] = hsh(p, t);
      }
			mv.push_back(dir);
			if (dir) {
				moves++;
				p = p + dxy[dir];
			}
		}
		return moves;
	}

	void update_instance() {
		cerr << "UPDATING INSTANCE " << endl;
		ins.moves.clear();
		vector<vector<int>> vv(maxt, vector<int>(ins.n, 0));
		for (int r = 0; r < ins.n; r++) {
			pt pos = ins.start[r];
      int lastt = get_t(robots[r]);
			for (int t = 0; t < maxt; t++) {
        int d = 0;
        if (t < lastt) {
          assert(g[hsh(pos, t)].r == r + 1);
          d = g[hsh(pos, t)].d;
        }
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
