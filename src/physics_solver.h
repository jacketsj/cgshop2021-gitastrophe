#pragma once

#include <bits/stdc++.h>

#include "Instance.h"

using namespace std;

namespace physics {

struct physics_optimizer {
	instance& ins;
	set<pt> obstacles;
	map<pt, int> robots;

	typedef map<pt, double> e_field;

	e_field repulsion;
	map<int, e_field> attraction;

	vector<double> mass;
	vector<pt> pos;
	int min_x, max_x, min_y, max_y;

	static constexpr double REPULSE_STR = 1.0;
	static constexpr double ATTRACT_STR = 1000.0;
	static constexpr int RADIUS = 5; // Smaller is faster
	static constexpr double DIFFUSE_RATE = 0.015; // Should be under 0.2
	static constexpr double TRANSMISSION_RATE = 0.5; // Should be under 1

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

	physics_optimizer(instance& ins) : ins(ins) {
		for (auto& p : ins.obstacle) {
			obstacles.insert(p);
		}

		int id = 0;
		mass.resize(ins.n);
		for (auto& p : ins.start) {
			pos.push_back(p);
			robots[p] = id;
			id++;
		}

		update_boundary();
	}

	bool is_obstacle(const pt& p) { return obstacles.count(p); }

	bool is_robot(const pt& p) { return robots.count(p); }

	bool in_boundary(const pt& p) {
		return min_x <= p.x && p.x <= max_x && min_y <= p.y && p.y <= max_y;
	}

	vector<pt> find_path(pt s, pt t) {
		// TODO: Speed up with A*
		queue<pt> q;
		q.push(s);

		bool found = false;
		map<pt, pt> par;
		while (!q.empty()) {
			pt c = q.front();
			q.pop();
			if (c == t) {
				found = true;
				break;
			}
			for (int i = 1; i <= 4; i++) {
				pt nc = c + dxy[i];
				if (par.count(nc) || !in_boundary(nc) || is_obstacle(nc)) 
					continue;

				if (is_robot(nc) && random() > TRANSMISSION_RATE)
					continue;

				q.push(nc);
				par[nc] = c;
			}
		}
		if (!found) return vector<pt>();

		vector<pt> path;
		while (t != s) {
			path.push_back(t);
			t = par[t];
		}
		path.push_back(t);
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

	void compute_attraction(int robot_id) {
		auto path = find_path(pos[robot_id], ins.target[robot_id]);

		int dist = 0;
		for (auto p : path) {
			attraction[robot_id][p] += -mass[robot_id] * ATTRACT_STR * dist * abs(pos[robot_id] - ins.target[robot_id]);
			dist++;
		}
		diffuse(attraction[robot_id], RADIUS);

		for (auto& q : attraction[robot_id]) {
			q.second += abs(q.first - ins.target[robot_id]) * ATTRACT_STR * 0.0005;
		}
	}

	void compute_energies() {
		attraction.clear();
		repulsion.clear();

		int id = -1;
		for (auto p : pos) {
			id++;

			//if (p == ins.target[id]) continue;
			double D = min(30, abs(pos[id] - ins.target[id]));
			repulsion[p] = mass[id] * REPULSE_STR * D;
			attraction[id][p] -= mass[id] * REPULSE_STR * D;
			diffuse(attraction[id], RADIUS);

			compute_attraction(id);
			if (p == ins.target[id]) continue;
			auto path = find_path(pos[id], ins.target[id]);
			for (auto q : path) {
				repulsion[q] += 0.2 * mass[id] * REPULSE_STR;
			}
		}
		diffuse(repulsion, RADIUS);
	}

	bool done() {
		for (size_t i = 0; i < pos.size(); i++) {
			if (pos[i] != ins.target[i]) {
				return false;
			}
		}
		return true;
	}

	void plan_step(int t) {
		// static double t = 0;
		// t++;

		for (size_t i = 0; i < pos.size(); i++) {
			if (pos[i] == ins.target[i])
				mass[i] = 1;//min(30, abs(pos[i] - ins.target[i]) + 1);
			else mass[i] = pow(sqrt(mass[i]) + 1, 2.0);
		}
		update_boundary();
		compute_energies();

		//begin:
		set<tuple<double, int, int>> pot_moves;
		for (size_t q = 0; q < pos.size(); q++) {
			auto r = pos[q];
			double Ei = repulsion[r] + attraction[q][r];
			for (int i = 0; i <= 4; i++) {
				// go in the direction of greatest energy decrease
				// energy = repulsion + attraction
				auto nr = r + dxy[i];
				if (is_obstacle(nr))
					continue;
				double Ef = repulsion[nr] + attraction[q][nr];
				double dE = Ef - Ei;

				if (dE <= 0) {
					pot_moves.insert(make_tuple(dE, q, i));
				}
			}
		}

		map<pt, int> nrobots;
		map<int, int> next_move;
		while (!pot_moves.empty() && next_move.size() < pos.size()) {
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
		//if (next_move.empty()) goto begin;

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

	bool run() {
		const int LIMIT = 400;
		for (int move = 0; move < LIMIT; ++move) {
			if (move % 10 == 0)
				cerr << "Planning step " << move + 1 << endl;
			plan_step(move);
			if (done()) {
				ins.time = ins.moves.size();
				return true;
			}
		}
		return false;
	}
};

bool run_physics(instance& ins, int sec = 1) {
	physics_optimizer popt(ins);
	bool succ = popt.run();
	ins.time = ins.moves.size();
	return succ;
}

} // namespace physics
