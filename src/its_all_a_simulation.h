#pragma once

#include <iostream>
#include <random>
#include <vector>

#include "Instance.h"
#include "sparse_graph.h"

using std::cerr;
using std::cout;
using std::endl;
using std::vector;

namespace simulation {
struct state {
	int t;
	vector<pt> pos;
	vector<int> wanted_moves;
	vector<int> planned_moves;
	// unordered_map<int, unordered_map<int, int>> robot_positions;
	state(instance& ins)
			: t(0), pos(ins.start), wanted_moves(ins.n), planned_moves(ins.n) {}
};
} // namespace simulation

struct simulation_optimizer {
	instance& ins;
	simulation::state s;
	sparse_graph::sparse_graph g;
	simulation_optimizer(instance& _ins) : ins(_ins), s(_ins), g(_ins) {}

	bool run_step() {
		bool finished = true;

		// computed wanted moves
		for (int r = 0; r < ins.n; ++r) {
			int& bdir = s.wanted_moves[r];
			pt bp = s.pos[r];
			auto bd = g.dist(bp, r);
			for (int dir = 1; dir < 5; ++dir) {
				pt p = s.pos[r] + dxy[dir];
				auto d = g.dist(p, r);
				if (g.in_range(p) && g.valid_move(s.pos[r], s.pos[r] + dxy[dir], s.t) &&
						d < bd) {
					bdir = dir;
					bp = p;
				}
				// TODO add random case for dist=dist
			}
		}
		// compute planned moves
		vector<int> robot_order(ins.n);
		for (int r = 0; r < ins.n; ++r)
			robot_order.push_back(r);
		// TODO add random shuffle
		// random_shuffle(robot_order.begin(), robot_order.end());
		for (int r : robot_order) {
			int dir = 0;
			if (g.valid_move(s.pos[r], s.pos[r] + dxy[s.wanted_moves[r]], s.t)) {
				dir = s.wanted_moves[r];
			}
			s.planned_moves[r] = dir;
			g.add_move(g.hsh(s.pos[r], s.t), r, dir);
			s.pos[r] = s.pos[r] + dxy[dir];
			if (s.pos[r] != ins.target[r])
				finished = false;
		}
		// solve deadlocks at some point
		vector<int> zeroes(ins.n, 0);
		if (!finished && s.planned_moves == zeroes) {
			g.update_instance();
			ins.debug_write("debug.out");
			cerr << "Deadlock, solution not coded yet" << endl;
			assert(false);
		}

		++s.t;
		s.planned_moves = zeroes;
		s.wanted_moves = zeroes;

		for (int r = 0; r < ins.n; ++r)
			g.robots[r] = g.hsh(s.pos[r], s.t);

		if (finished)
			g.update_instance();

		return finished;
	}
};

namespace simulation {
bool run_sim(instance& ins, int sec = 1) {
	simulation_optimizer so(ins);
	clock_t start_time = clock();
	int steps = 0;
	while (clock() - start_time < sec * CLOCKS_PER_SEC) {
		// if (steps % 100 == 0)
		cout << "Steps run: " << steps << '\n';
		steps++;
		if (so.run_step()) {
			cout << "Finished running optimizer!" << endl;
			return true;
		}
	}
	cout << "Ran out of time" << endl;
	return false;
}
}; // namespace simulation
