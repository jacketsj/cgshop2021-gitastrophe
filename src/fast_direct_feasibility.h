#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>

#include "Instance.h"
#include "matching.h"
#include "sparse_graph.h"

using namespace std;

namespace fast_direct_feasibility {

int min_y(const instance& ins) {
	int ret = 0;
	for (auto& p : ins.obstacle) {
		ret = min(p.y, ret);
	}

	for (auto& p : ins.target) {
		ret = min(p.y, ret);
	}

	for (auto& p : ins.start) {
		ret = min(p.y, ret);
	}
	return ret;
}

int max_y(const instance& ins) {
	int ret = 0;
	for (auto& p : ins.obstacle) {
		ret = max(p.y, ret);
	}

	for (auto& p : ins.target) {
		ret = max(p.y, ret);
	}

	for (auto& p : ins.start) {
		ret = max(p.y, ret);
	}
	return ret;
}

int min_x(const instance& ins) {
	int ret = 0;
	for (auto& p : ins.obstacle) {
		ret = min(p.x, ret);
	}

	for (auto& p : ins.target) {
		ret = min(p.x, ret);
	}

	for (auto& p : ins.start) {
		ret = min(p.x, ret);
	}
	return ret;
}

int max_x(const instance& ins) {
	int ret = 0;
	for (auto& p : ins.obstacle) {
		ret = max(p.x, ret);
	}

	for (auto& p : ins.target) {
		ret = max(p.x, ret);
	}

	for (auto& p : ins.start) {
		ret = max(p.x, ret);
	}
	return ret;
}

struct simple_rect_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	simple_rect_filler(pt starting_point, pt delta, int M, bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	pt next() {
		auto ret = current;
		if (x_first) {
			current.x += dx;
			if (current.x * sign > M * sign) {
				current.x = xm;
				current.y += dy;
			}
		} else {
			current.y += dy;
			if (current.y * sign > M * sign) {
				current.y = ym;
				current.x += dx;
			}
		}
		return ret;
	}
};

struct rect_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	rect_filler(pt starting_point, pt delta, int M, bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	pt next() {
		auto ret = current;
		if (x_first) {
			current.x += dx;
			if (current.x * sign > M * sign) {
				xm -= dx;
				M += dx;
				current.x = xm;
				current.y += dy;
			}
		} else {
			current.y += dy;
			if (current.y * sign > M * sign) {
				ym -= dy;
				M += dy;
				current.y = ym;
				current.x += dx;
			}
		}
		return ret;
	}
};

// don't use this one
struct strange_pseudodiamond_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	strange_pseudodiamond_filler(pt starting_point, pt delta, int M,
															 bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	pt next() {
		auto ret = current;
		if (x_first) {
			current.x += dx;
			if (current.x * sign > M * sign) {
				if (rand() % 5 < 2) {
					xm += dx;
					M -= dx;
				}
				current.x = xm;
				current.y += dy;
			}
		} else {
			current.y += dy;
			if (current.y * sign > M * sign) {
				if (rand() % 5 < 2) {
					ym += dy;
					M -= dy;
				}
				current.y = ym;
				current.x += dx;
			}
		}
		return ret;
	}
};

struct hexagon_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	hexagon_filler(pt starting_point, pt delta, int M, bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	pt next() {
		auto ret = current;
		int closeness = 0;
		if (x_first) {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(xm - current.x), abs(M - current.x));
			ret.y += closeness * dy / abs(dy * dy);

			current.x += dx;
			if (current.x * sign > M * sign) {
				xm -= dx;
				M += dx;
				current.x = xm;
				current.y += dy;
			}
		} else {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(ym - current.y), abs(M - current.y));
			ret.x += closeness * dx / abs(dx * dx);

			current.y += dy;
			if (current.y * sign > M * sign) {
				ym -= dy;
				M += dy;
				current.y = ym;
				current.x += dx;
			}
		}

		return ret;
	}
};

struct diamond_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	diamond_filler(pt starting_point, pt delta, int M, bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	bool has_queued = false;
	pt queued;
	pt next() {
		if (has_queued) {
			has_queued = false;
			return queued;
		}
		auto ret = current;
		int closeness = 0;
		if (x_first) {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(xm - current.x), abs(M - current.x));
			ret.y += closeness * dy / abs(dy);
			queued = ret;
			queued.y += dy;

			current.x += dx;
			if (current.x * sign > M * sign) {
				xm -= dx;
				M += dx;
				current.x = xm;
				current.y += dy;
			}
		} else {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(ym - current.y), abs(M - current.y));
			ret.x += closeness * dx / abs(dx);
			queued = ret;
			queued.x += dx;

			current.y += dy;
			if (current.y * sign > M * sign) {
				ym -= dy;
				M += dy;
				current.y = ym;
				current.x += dx;
			}
		}
		has_queued = true;
		return ret;
	}
};

struct sparse_diamond_filler {
	pt current;
	int xm, ym, M;
	int dx, dy;
	int sign; // equal to d_major/abs(d_major)
	bool x_first;
	sparse_diamond_filler(pt starting_point, pt delta, int M, bool x_first) {
		current = starting_point;
		dx = delta.x;
		dy = delta.y;
		xm = current.x;
		ym = current.y;
		this->M = M;
		this->x_first = x_first;
		if (x_first) {
			sign = dx / abs(dx);
		} else {
			sign = dy / abs(dy);
		}
	}
	pt next() {
		auto ret = current;
		int closeness = 0;
		if (x_first) {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(xm - current.x), abs(M - current.x));
			ret.y += closeness * dy / dy;

			current.x += dx;
			if (current.x * sign > M * sign) {
				xm -= dx;
				M += dx;
				current.x = xm;
				current.y += dy;
			}
		} else {
			// compute transform by determining how "close to center" we are
			closeness = min(abs(ym - current.y), abs(M - current.y));
			ret.x += closeness * dx / abs(dx * dx);

			current.y += dy;
			if (current.y * sign > M * sign) {
				ym -= dy;
				M += dy;
				current.y = ym;
				current.x += dx;
			}
		}

		return ret;
	}
};

void assign_grid_positions(vector<pt>& grid_pos, const vector<pt>& p1,
													 const vector<pt>& p2, double _dist_power = 1) {
	assert(grid_pos.size() >= p1.size() && p1.size() == p2.size());

	size_t n_big = grid_pos.size(); // The number of total grid positions we
																	// generate (more than the number of robots)
	size_t n = p1.size();

	vector<int> agent_permutation(p1.size()); // maps agents to their node index
	for (size_t i = 0; i < agent_permutation.size(); ++i)
		agent_permutation[i] = i;
	random_shuffle(agent_permutation.begin(), agent_permutation.end());

	// random_device r;
	// default_random_engine gen{r()};
	// uniform_real_distribution<double> eps_dist(-0.01, 0.01);
	/*
	// -distance, random number, grid position, agent
	priority_queue<tuple<int, int, size_t, size_t>> possibilities;
	vector<int> assignments(n, -1); // assigns agent to grid position
	*/
	vector<vector<short>> cost(n_big, vector<short>(n_big));
	for (size_t i = 0; i < n_big; ++i) {
		for (size_t a = 0; a < n; ++a) {
			int dist1 = dist(p1[a], grid_pos[i]);
			int dist2 = dist(p2[a], grid_pos[i]);
			// int distance = dist1 * dist1 + dist2 + dist2;
			short distance = short(dist1 + dist2);
			cost[i][agent_permutation[a]] = distance;
			// cost[i][a] = pow(double(distance), dist_power) + eps_dist(gen);
			//
			// possibilities.emplace(-distance, rand(), i, a);
		}
		// the remaining n_big-n values will default to 0, which is ok
	}
	vector<int> pos_assignments(n), agent_assignments(n);
	matchingpls(cost, pos_assignments, agent_assignments);
	/*
	int num_assigns = 0;
	vector<bool> used(n); // has grid position been filled by agent?
	// greedily pair up closest agents and grid positions first
	while (!possibilities.empty()) {
		int distance, r;
		size_t i, a;
		tie(distance, r, i, a) = possibilities.top();
		possibilities.pop();
		if (!used[i] && assignments[a] == -1) {
			++num_assigns;
			assignments[a] = i;
			used[i] = true;
		}
	}
	*/
	vector<pt> new_grid_pos(n);
	for (size_t a = 0; a < n; ++a) {
		new_grid_pos[a] = grid_pos[agent_assignments[agent_permutation[a]]];
		/*
		assert(assignments[a] >= 0);
		new_grid_pos[a] = grid_pos[assignments[a]];
		*/
	}
	grid_pos = new_grid_pos;
}

typedef hexagon_filler filler_in_use;

int EXTRAS = 2; // factor of extra grid positions (increasing slows down
								// matching, but potentially improves quality of solution)

pair<instance, instance> split_instance(const instance& ins) {
	pair<instance, instance> ret = pair<instance, instance>(ins, ins);

	const int six = 1;
	// first need to find bounds
	int my = min_y(ins) - six; // this is the starting point of the bottom 'grid'
	int mx = min_x(ins) - six; // this is the starting point of the left 'grid'
	int My = max_y(ins) + six; // this is the starting point of the top 'grid'
	int Mx = max_x(ins) + six; // this is the starting point of the right 'grid'
	// rect_filler top(pt(mx + six, My), pt(2, 2), Mx - six, true);
	// rect_filler bottom(pt(mx + six, my), pt(2, -2), Mx - six, true);
	// rect_filler left(pt(mx, my + six), pt(-2, 2), My - six, false);
	// rect_filler right(pt(Mx, my + six), pt(2, 2), My - six, false);
	vector<filler_in_use> rects = {
			filler_in_use(pt(mx + six, My), pt(2, 2), Mx - six, true),
			filler_in_use(pt(mx + six, my), pt(2, -2), Mx - six, true),
			filler_in_use(pt(mx, my + six), pt(-2, 2), My - six, false),
			filler_in_use(pt(Mx, my + six), pt(2, 2), My - six, false)};
	// vector<std::reference_wrapper<rect_filler>>
	//		rects; //({top, bottom, left, right});
	// rects.emplace_back(top);
	// rects.emplace_back(bottom);
	// rects.emplace_back(left);
	// rects.emplace_back(right);
	// now make a giant square out of the agents (with gaps)
	vector<pt> grid_pos;
	// int height = int(sqrt(ins.n));
	// int grid_bottom = my - height;
	// int current_y = my;
	// int current_x = 0;
	for (int i = 0; i < ins.n * EXTRAS; ++i) {
		// grid_pos.emplace_back(current_y, current_x);
		grid_pos.push_back(rects[i % rects.size()].next());
		// grid_pos.push_back(rects[(i * rects.size()) / ins.n].next());
		// current_y -= 2;
		// if (current_y < grid_bottom) {
		//	current_y = my;
		//	current_x += 2;
		//}
	}

	assign_grid_positions(grid_pos, ins.start, ins.target);
	// random_shuffle(grid_pos.begin(), grid_pos.end());

	ret.first.target = grid_pos;
	ret.second.start = grid_pos;
	reverse_instance(ret.first);

	return ret;
}

// assumes the start locations are the ones from the actual instance (not
// intermediates) assumes intermediate locations are targets finds the shortest
// distance from any intermediate to each agent start location
vector<int> find_ordering(instance& ins) {
	sparse_graph::sparse_graph g(ins);
	// use dist(p, 0) for distance from robot 0 start
	// distance, random number, robot
	vector<tuple<int, int, int>> dist_and_robot;
	for (int i = 0; i < ins.n; ++i) {
		int d = g.dist(ins.start[i], 0);
		for (int j = 1; j < ins.n; ++j)
			d = min(d, g.dist(ins.start[i], j));
		dist_and_robot.emplace_back(d, rand(), i);
	}
	sort(dist_and_robot.rbegin(), dist_and_robot.rend());
	vector<int> ordering;
	for (auto& tup : dist_and_robot)
		ordering.push_back(get<2>(tup));
	return ordering;
}

bool find_paths(instance& ins, const vector<int>& order) {
	// come up with a hopefully ok maxt until something better can be done (TODO)
	// int maxt = ins.n * (max_y(ins) - min_y(ins));
	int maxt = 0;
	cout << "Using maxt=" << maxt << endl;
	sparse_graph::sparse_graph g(ins, maxt);
	bool success = true;
	int count = 0;
	for (int i : order) {
		if (count++ % 20 == 0)
			cerr << "Finding path no " << count << endl;

		success = success && g.find_best_randomized(i, 0, true, true, 1);

		if (!success) {
			cerr << "Failed to find path no " << count - 1 << " (for agent " << i
					 << ") in instance " << ins.name << endl;
			cerr << "halting: maxt=" << maxt << " (is this too small?)" << endl;
			g.update_instance();
			ins.name = "debug";
			ins.debug_write("debug.out");
			ins.write_input("debug");
			assert(false);
		}
	}
	g.update_instance();
	return success;
}

bool find_paths(instance& ins) {
	reverse_instance(ins);
	vector<int> order = find_ordering(ins);
	reverse_instance(ins);
	return find_paths(ins, order);
}

// takes valid instance and greedily improves paths taken by entitities
struct fast_feasibility {
	instance& ins;

	pair<instance, instance> ins_split; // two halves

	fast_feasibility(instance& _ins)
			: ins(_ins), ins_split(split_instance(ins)) {}

	bool run() {
		bool ret = find_paths(ins_split.first);

		reverse_instance(ins_split.second);
		vector<int> order = find_ordering(ins_split.second);
		reverse_instance(ins_split.second);

		if (!ret)
			return ret;

		reverse_instance(ins_split.first);
		ins.moves = combine_movesets(ins_split.first, ins_split.second);
		ins.time = ins.moves.size();
		reverse_instance(ins_split.first);

		ret = ret && find_paths(ins, order);
		// bool ret = find_paths(ins_split.first) && find_paths(ins_split.second);
		if (ret) {
			// reverse_instance(ins_split.first);
			// ins.moves = combine_movesets(ins_split.first, ins_split.second);
			// reverse_instance(ins_split.first);

			// ins.time = ins.moves.size();

			// cout << "Finished finding paths and combining moveset, about to verify
			// " 				"the solution"
			//		 << endl;
			// bool verified = verify(ins);
			// cout << "verified? " << verified << endl;
			// assert(verified);
		}
		return ret;
	}
};

bool run_fast_direct_feasibility(instance& ins, bool reverse = false) {
	if (reverse)
		reverse_instance(ins);
	fast_feasibility ff(ins);
	bool success = ff.run();
	if (reverse)
		reverse_instance(ins);
	if (success)
		cout << "Fast DIRECT feasibility success! Makespan=" << score(ins, true)
				 << ", distance=" << score(ins, false) << '\n';
	else
		cout << "Fast DIRECT feasibility failed :(" << '\n';
	return success;
}

} // namespace fast_direct_feasibility
