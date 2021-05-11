#pragma once

#include <bits/stdc++.h>

#include "Instance.h"
#include "graph.h"

using namespace std;

namespace feasibility {

struct feasibility_optimizer {
	instance& ins;
	set<pt> obstacles;
	unordered_map<pt, int> robots;
	vector<pt> pos; // only the actual position in the expansion phase.
	int min_x, max_x, min_y, max_y;

	typedef unordered_map<pt, double> e_field;
	e_field repulsion;

	static int constexpr RADIUS = 6;
	static constexpr double DIFFUSE_RATE = 0.015; // Should be under 0.2
	static constexpr int BBOX_LEN = 150;

	bool is_obstacle(const pt& p) { return obstacles.count(p); }

	bool is_robot(const pt& p) { return robots.count(p); }

	bool in_boundary(const pt& p) {
		return min_x <= p.x && p.x <= max_x && min_y <= p.y && p.y <= max_y;
	}

	struct static_charge {
		pt c;
		e_field dist;
		void set_centre(pt x, feasibility_optimizer* opt) {
			c = x;
			queue<pt> q;
			for (int i = c.x - BBOX_LEN; i <= c.x + BBOX_LEN; i++) {
				auto p_u = pt(i, (int) c.y + sqrt(BBOX_LEN * BBOX_LEN - pow(i - c.x, 2.0)));
				auto p_d = pt(i, (int) c.y - sqrt(BBOX_LEN * BBOX_LEN - pow(i - c.x, 2.0)));
				q.push(p_u);
				dist[p_u] = 0;

				q.push(p_d);
				dist[p_d] = 0;
			}
			
			cerr << "Preparing radiation field." << endl;
			while (!q.empty()) {
				pt c = q.front();
				q.pop();

				for (int i = 1; i <= 4; i++) {
					pt nc = c + dxy[i];
					if (nc.x < -BBOX_LEN + x.x || nc.x > x.x + BBOX_LEN || 
						nc.y < -BBOX_LEN + x.y || nc.y > x.y + BBOX_LEN)
						continue;

					if (dist.count(nc) || opt->is_obstacle(nc)) 
						continue;

					q.push(nc);
					dist[nc] = dist[c] + 1;
				}
			}
		}

		double potential(pt x) {
			return dist.count(x) ? dist[x] - 0.01 * abs(c-x) : 0;
		}
	};
	static_charge charge;

	double random() {
		return (rand() % 1000000) / 1000000.;
	}

	void update_boundary() {
		min_x = max_x = 0;
		min_y = max_y = 0;

		for (auto& p : pos) {
			min_x = min(p.x - 5, min_x);
			min_y = min(p.y - 5, min_y);
			max_x = max(p.x + 5, max_x);
			max_y = max(p.y + 5, max_y);
		}

		for (auto& p : ins.target) {
			min_x = min(p.x - 5, min_x);
			min_y = min(p.y - 5, min_y);
			max_x = max(p.x + 5, max_x);
			max_y = max(p.y + 5, max_y);
		}

		for (auto& p : ins.obstacle) {
			min_x = min(p.x - 5, min_x);
			min_y = min(p.y - 5, min_y);
			max_x = max(p.x + 5, max_x);
			max_y = max(p.y + 5, max_y);
		}
	}

	feasibility_optimizer(instance& ins) : ins(ins) {
		for (auto& p : ins.obstacle) {
			obstacles.insert(p);
		}

		int id = 0;
		for (auto& p : ins.start) {
			pos.push_back(p);
			robots[p] = id;
			id++;
		}

		update_boundary();

		int cx = min_x + (max_x - min_x) / 2;
		int cy = min_y + (max_y - min_y) / 2;

		pt c(cx, cy);
		// double best_d = 10000;
		// pt best_start;
		// for (auto p : ins.target) {
		// 	if (best_d > norm(p-c)) {
		// 		best_d = norm(p-c);
		// 		best_start = c;
		// 	}
		// }
		charge.set_centre(c, this);
	}

	vector<int> find_path(pt s, pt t, int offset, int id) {
		bool found = false;
		unordered_map<pair<pt, int>, int> par;
		unordered_map<pair<pt, int>, bool> robot_map;
		auto tpos = ins.start;
		
		for (size_t time = 0; time <= ins.moves.size(); time++) {
			int c = 0;
			for (pt p : tpos) {
				if (c++ == id) {
					continue;
				}
				robot_map[{p, time}] = true;
			}

			if (time < ins.moves.size()) {
				for (int i = 0; i < ins.n; i++) {
					tpos[i] = tpos[i] + dxy[ins.moves[time][i]];
				}
			}
		}
		for (int i = 0; i < ins.n; i++) {
			assert(tpos[i] == pos[i]);
		}

		auto blocked = [&](pt p, int time) {
			if (time > (int) ins.moves.size()) {
				return robot_map.count({p, ins.moves.size()});
			} else {
				return robot_map.count({p, time});
			}
		};

		// Check reachability first
		priority_queue<tuple<double, pt>> qq;
		qq.push({-abs(s-t), t});
		unordered_map<pt, int> dist;
		while (!qq.empty()) {
			auto state = qq.top();
			qq.pop();
			pt c = get<1>(state);
			if (c == s) {
				found = true;
				// break;
			}

			for (int i = 0; i <= 4; i++) {
				pt nc = c + dxy[i];
				if (dist.count(nc) || !in_boundary(nc) || 
					(is_robot(nc) && robots[nc] != id) || is_obstacle(nc)) 
					continue;

				double D = -get<0>(state) + 1 + abs(nc - s) - abs(c - s);
				dist[nc] = dist[c] + 1;
				qq.push({-D, nc});
			}
		}
		if (!found) {
			cerr << "Path not found for robot " << id << ". Trying again later." << endl;
			return vector<int>();
		}

		priority_queue<tuple<double, pt, int>> q;
		q.push({-dist[s], s, offset});
		par[{s, offset}] = -1;
		int ftime = -1;
		while (!q.empty()) {
			auto state = q.top();
			q.pop();

			pt c = get<1>(state);
			if (c == t && get<2>(state) >= (int) ins.moves.size()) {
				found = true;
				ftime = get<2>(state);
				break;
			}

			for (int i = 0; i <= 4; i++) {
				pt nc = c + dxy[i];
				int time = get<2>(state) + 1;
				if (par.count({nc, time}) || !in_boundary(nc) || 
					blocked(nc, time) || blocked(nc, time-1) || 
					blocked(nc, time+1) || is_obstacle(nc)) 
					continue;

				double D = -get<0>(state) + 1 + dist[nc] - dist[c];
				q.push({-D, nc, time});
				par[{nc, time}] = i;
			}
		}
		assert(ftime != -1);

		vector<int> path;
		while (ftime != offset) {
			auto state = make_pair(t, ftime);
			path.push_back(par[state]);
			t = t + dxy[opposite_dir[par[state]]];
			ftime--;
		}
		reverse(path.begin(), path.end());
		return path;
	}

	void diffuse(e_field& F, int r) {
		while (r--) {
			vector<pt> pts;
			for (auto p : F) {
				pts.push_back(p.first);
			}

			auto nF = F;
			for (auto p : pts) {
				for (int i = 1; i <= 4; i++) {
					auto np = p + dxy[i];
					if (in_boundary(np) && !is_obstacle(np)) {
						nF[np] += F[p] * DIFFUSE_RATE;
					}
				}
			}
			F = nF;
		}
	}

	void expand_out(bool repulse) {
		// Model the robots as charges plus an external potential field
		repulsion.clear();
		if (repulse) {
			for (auto p : pos) {
				repulsion[p] = 1e4;
			}
			for (auto p : obstacles) {
				repulsion[p] = 1e4;
			}
			diffuse(repulsion, RADIUS);
		}

		set<tuple<double, int, int>> pot_moves;
		for (size_t q = 0; q < pos.size(); q++) {
			auto r = pos[q];
			double Ei = charge.potential(r) + repulsion[r];
			if (repulse) Ei = repulsion[r];
			for (int i = 0; i <= 4; i++) {
				// go in the direction of greatest energy decrease
				auto nr = r + dxy[i];
				if (is_obstacle(nr))
					continue;
				double Ef = charge.potential(nr) + repulsion[nr];
				if (repulse) Ef = repulsion[nr];
				double dE = Ef - Ei;

				if (dE < 0) {
					pot_moves.insert(make_tuple(dE, q, i));
				}
			}
		}

		if (repulse) {
			double M = get<0>(*pot_moves.begin());
			set<tuple<double, int, int>> temp_moves;
			for (auto m : pot_moves) {
				if (random() > get<0>(m) / M) continue;
				temp_moves.insert(m);
			}
			pot_moves = temp_moves;
		}

		unordered_map<pt, int> nrobots;
		unordered_map<int, int> next_move;
		int last = -1;
		while (last != int(pot_moves.size()) && next_move.size() < pos.size()) {
			last = pot_moves.size();
			set<tuple<double, int, int>> npot_moves;
			bool moved = false;
			for (auto m : pot_moves) {
				double dE;
				int q, i;
				std::tie(dE, q, i) = m;

				if (next_move.count(q)) {
					continue;
				}

				pt one_step = pos[q] + dxy[i];
				pt two_steps = pos[q] + dxy[i] + dxy[i];

				if (nrobots.count(one_step)) {
					continue;
				}

				if (i != 0 && is_robot(one_step) &&
						!(nrobots.count(two_steps) &&
							nrobots[two_steps] == robots[one_step])) {
					npot_moves.insert(m);
					continue;
				}

				if (!moved) {
					next_move[q] = i;
					nrobots[one_step] = q;
					moved = true;
				} else {
					npot_moves.insert(m);
				}
			}
			pot_moves = npot_moves;
		}

		vector<int> curr_move;
		robots.clear();
		for (size_t i = 0; i < pos.size(); i++) {
			curr_move.push_back(next_move[i]);
			pos[i] = pos[i] + dxy[next_move[i]];
			robots[pos[i]] = i;
		}
		ins.moves.push_back(curr_move);
		update_boundary();
	}

	bool fill_in() {
		set<pair<double, int>> not_done;

		queue<pt> q;
		unordered_map<pt, int> dist;

		for (int x = min_x; x <= max_x; x++) {
			q.push(pt(x, min_y));
			dist[pt(x, min_y)] = 0;
			q.push(pt(x, max_y));
			dist[pt(x, max_y)] = 0;
		}

		for (int y = min_y; y <= max_y; y++) {
			q.push(pt(max_x, y));
			dist[pt(max_x, y)] = 0;
			q.push(pt(min_x, y));
			dist[pt(min_x, y)] = 0;
		}

		while (!q.empty()) {
			pt c = q.front();
			q.pop();

			for (int i = 1; i <= 4; i++) {
				pt nc = c + dxy[i];
				if (dist.count(nc) || !in_boundary(nc) || is_obstacle(nc)) 
					continue;

				q.push(nc);
				dist[nc] = dist[c] + 1;
			}
		}

		for (size_t i = 0; i < ins.target.size(); i++) {
			if (pos[i] != ins.target[i]) {
				not_done.insert({-dist[ins.target[i]], i});
			}
		}

		int offset = ins.moves.size();
		size_t last = 0;

  if (true) {
		while (last != not_done.size()) {
			cerr << "Starting path assignment..." << endl;
			auto new_set = not_done;
			for (auto q : not_done) {
				int i = q.second;
				// size_t offset = ins.moves.size();
			
				auto path = find_path(pos[i], ins.target[i], offset, i);
				if (!path.empty()) {
					new_set.erase(q);
					
					robots.erase(pos[i]);
					for (size_t j = 0; j < path.size(); j++) {
						if (j + offset >= ins.moves.size()) {
							ins.moves.push_back(vector<int>(ins.n));
						}
						pos[i] = pos[i] + dxy[path[j]];
						ins.moves[offset+j][i] = path[j];
					}
					robots[pos[i]] = true;
					int ndone = not_done.size() - new_set.size();
					if (ndone % 10 == 0) {
						cerr << "Found a home for " << ndone << " robots" << endl;
					}
				}
			}
			last = not_done.size();
			not_done = new_set;
		}
    } else {
      // don't use this code right now, it seems to be slower.
      graph g(ins, offset*2 + 200); // add some extra time to find a path
      set<int> failed;
      int cnt = 0;
      for(auto q : not_done) {
        if(!g.find_best(q.second, offset)) {
          cerr << "FAILED "<< q.second <<endl;
          failed.insert(q.second);
        }
        cnt++;
        if (cnt) cerr << cnt << " robots have gone home " <<endl;
      }
      g.update_instance();
      ins.debug_write("debug.out");
      for(int i:failed) cerr << i << endl;
      if (!failed.empty()) assert(false);
    }

		if (last == 0) {
			cerr << "Feasible solution found!" << endl;
			return true;
		} else { 
			cerr << "There are " << last << " robots left with paths." << endl;
			return false;
		}
	}

	bool run() {
		int move = 0;
		int xl = min_x+2, xr = max_x-2, yl = min_y+2, yr = max_y-2;
		bool mode = 0;
		while (true) {
			int ninside = 0;

			double avg_free = 0;
			bool all_free = true;
			for (pt p : pos) {
				if (xl <= p.x && p.x <= xr && yl <= p.y && p.y <= yr) {
					ninside++;
				}

				int nsep = 0;
				for (int i = 1; i <= 4; i++) {
					if (!is_robot(p + dxy[i]))
						nsep++;
				}
				if (nsep < 4) {
					all_free = false;
				}
				avg_free += nsep;
			}
			avg_free /= pos.size();

			if (ninside == 0 && all_free) break;

			if (ninside == 0) {
				mode = 1;
			} else {
				mode = 0;
			}

			if (mode == 0) {
				expand_out(false);
				expand_out(false);
				expand_out(true);
				if (move++ % 10 == 0) {
					cerr << "Expansion step " << move 
						 << ". Num. left inside: " << ninside << endl;
				}
			} else {
				if (move++ % 10 == 0)
					cerr << "Extra expansion step needed. Step: " << move << " with average freedom " << avg_free << endl;

				expand_out(true);
				expand_out(true);
				expand_out(false);
			}
		}
		cerr << "Expansion phase complete." << endl;
		bool succ = fill_in();
		return succ;
	}
};

bool run_feasibility(instance& ins, int sec = 1) {
	feasibility_optimizer popt(ins);
	bool succ = popt.run();
	ins.time = ins.moves.size();
	return succ;
}

} // namespace feasibility
