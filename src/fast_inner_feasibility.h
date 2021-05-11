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

namespace fast_inner_feasibility {

static const short SINF = 0x3f3f;

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

// Can be fed an arbitrary set of 'grid positions', which may even include
// those in the interior. Still needs them to be spaced correctly though.
void assign_grid_positions(vector<pt>& grid_pos, const vector<pt>& p1,
													 const vector<pt>& p2, vector<pt>& outside_pts,
													 instance& ins, double _dist_power = 1) {
	assert(grid_pos.size() >= p1.size() && p1.size() == p2.size());

	instance outside_computer;
	outside_computer.clear();
	outside_computer.n = outside_pts.size();
	outside_computer.start = outside_pts;
	outside_computer.target = outside_pts;
	outside_computer.m = ins.m;
	outside_computer.obstacle = ins.obstacle;
	sparse_graph::sparse_graph oc_graph(outside_computer);
	map<pt, int> innerness;
	int mx = min_x(ins);
	int Mx = max_x(ins);
	int my = min_y(ins);
	int My = max_y(ins);
	auto in_bounds = [&](const pt& p) {
		return p.x >= mx && p.x <= Mx && p.y >= my && p.y <= My;
	};
	auto set_innerness = [&](const pt& p) {
		innerness[p] = SINF;
		if (!in_bounds(p))
			innerness[p] = -1;
		else
			for (int i = 0; i < int(outside_pts.size()); ++i)
				innerness[p] = min(innerness[p], oc_graph.dist(p, i));
	};
	for (auto& p : grid_pos)
		set_innerness(p);
	for (const auto& p : p1)
		set_innerness(p);
	for (const auto& p : p2)
		set_innerness(p);

	size_t n_big = grid_pos.size(); // The number of total grid positions we
																	// generate (more than the number of robots)
	size_t n = p1.size();

	vector<int> agent_permutation(p1.size()); // maps agents to their node index
	for (size_t i = 0; i < agent_permutation.size(); ++i)
		agent_permutation[i] = i;
	random_shuffle(agent_permutation.begin(), agent_permutation.end());

	vector<vector<double>> cost(n_big, vector<double>(n_big));
	const double INF = 1e9; // std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < n_big; ++i) {
		for (size_t a = 0; a < n; ++a) {
			int dist1 = dist(p1[a], grid_pos[i]);
			int dist2 = dist(p2[a], grid_pos[i]);
			// int distance = dist1 * dist1 + dist2 + dist2;
			double distance = double(dist1 + dist2);
			if (innerness[p1[a]] > innerness[grid_pos[i]] &&
					innerness[p2[a]] > innerness[grid_pos[i]])
				cost[i][agent_permutation[a]] = distance * distance;
			else
				cost[i][agent_permutation[a]] = INF;

			// cost[i][a] = pow(double(distance), dist_power) + eps_dist(gen);
		}
		// the remaining n_big-n values will default to 0, which is ok
	}
	vector<int> pos_assignments(n), agent_assignments(n);
	matchingpls(cost, pos_assignments, agent_assignments);

	vector<pt> new_grid_pos(n);
	for (size_t a = 0; a < n; ++a) {
		new_grid_pos[a] = grid_pos[agent_assignments[agent_permutation[a]]];
		assert(
				cost[agent_assignments[agent_permutation[a]]][agent_permutation[a]] !=
				INF);
	}
	grid_pos = new_grid_pos;
}

// returns a set of all reachable even points
// excludes points if they are within adjacent to a wall
vector<pt> even_points(instance& ins) {
	sparse_graph::sparse_graph g(ins);
	vector<pt> ret;
	map<pt, bool> near_wall;
	for (pt& w : ins.obstacle)
		for (int dir = 1; dir < 9; ++dir)
			near_wall[w + dxy[dir]] = true;
	int needed_buffer = 0; // is a (100+needed_buffer*2)^2-100^2 box enough to
												 // fit all robots well? 60 suffices, but might as well
												 // increase it a bit in case it's helpful
	int mx = min_x(ins) - needed_buffer;
	int Mx = max_x(ins) + needed_buffer;
	int my = min_y(ins) - needed_buffer;
	int My = max_y(ins) + needed_buffer;
	for (int x = mx; x <= Mx; x += 4)
		for (int y = my; y <= My; y += 4) {
			pt p(x, y);
			if (g.dist(p, 0) < SINF && !near_wall[p])
				ret.push_back(p);
		}
	return ret;
}

typedef hexagon_filler filler_in_use;

int EXTRAS = 2; // factor of extra grid positions (increasing slows down
								// matching, but potentially improves quality of solution)

pair<instance, instance> split_instance(instance& ins,
																				vector<pt>& outside_pts) {
	pair<instance, instance> ret = pair<instance, instance>(ins, ins);

	const int six = 2;
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

	auto ep = even_points(ins);
	for (auto& p : ep)
		grid_pos.push_back(p);
	// auto grid_pos = even_points(ins);

	assign_grid_positions(grid_pos, ins.start, ins.target, outside_pts, ins);
	// random_shuffle(grid_pos.begin(), grid_pos.end());

	ret.first.target = grid_pos;
	ret.second.start = grid_pos;
	reverse_instance(ret.first);

	return ret;
}

// Note: Assumes instance is backwards from what it is in fast_feasibility/ff
// and fast_direct_feasibility/fff assumes the TARGET locations are the ones
// from the actual instance (not intermediates) assumes intermediate locations
// are starts (doesn't use them)
vector<int> find_ordering(instance& ins, vector<pt>& outside_pts) {
	sparse_graph::sparse_graph g(ins);
	// use dist(p, 0) for distance from robot 0 start
	// distance, random number, robot
	vector<tuple<int, int, int>> dist_and_robot;
	for (int i = 0; i < ins.n; ++i) {
		int d = g.dist(outside_pts[0], i);
		for (int j = 1; j < ins.n; ++j)
			d = min(d, g.dist(outside_pts[j], i));
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

bool find_paths(instance& ins, vector<pt>& outside_pts) {
	vector<int> order = find_ordering(ins, outside_pts);
	return find_paths(ins, order);
}

vector<pt> all_points(instance& ins) {
	sparse_graph::sparse_graph g(ins);
	vector<pt> ret;
	int mx = min_x(ins) - 2;
	int Mx = max_x(ins) + 2;
	int my = min_y(ins) - 2;
	int My = max_y(ins) + 2;
	for (int x = mx; x <= Mx; ++x)
		for (int y = my; y <= My; ++y) {
			pt p(x, y);
			if (g.dist(p, 0) < SINF)
				ret.push_back(p);
		}
	return ret;
}

vector<pt> compute_outside_pts(instance& ins) {
	vector<pt> ret;
	instance blocked;
	blocked.clear();
	pt example_outside_pt(min_x(ins) - 1, min_y(ins) - 1);
	blocked.n = 1;
	blocked.start.push_back(example_outside_pt);
	blocked.target.push_back(example_outside_pt);
	blocked.obstacle = ins.obstacle;
	for (auto& p : ins.start)
		blocked.obstacle.push_back(p);
	for (auto& p : ins.target)
		blocked.obstacle.push_back(p);
	blocked.m = blocked.obstacle.size();

	sparse_graph::sparse_graph bg(blocked);
	for (pt& p : all_points(ins)) {
		if (bg.dist(p, 0) < SINF)
			ret.push_back(p);
	}
	return ret;
}

// takes valid instance and greedily improves paths taken by entitities
struct fast_feasibility {
	instance& ins;
	vector<pt> outside_pts;

	pair<instance, instance> ins_split; // two halves

	fast_feasibility(instance& _ins)
			: ins(_ins), outside_pts(compute_outside_pts(ins)),
				ins_split(split_instance(ins, outside_pts)) {}

	bool run() {
		bool ret = find_paths(ins_split.first, outside_pts);

		vector<int> order = find_ordering(ins_split.second, outside_pts);

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

bool run_fast_inner_feasibility(instance& ins, bool reverse = false) {
	if (reverse)
		reverse_instance(ins);
	fast_feasibility ff(ins);
	bool success = ff.run();
	if (reverse)
		reverse_instance(ins);
	if (success)
		cout << "Fast INNER feasibility success! Makespan=" << score(ins, true)
				 << ", distance=" << score(ins, false) << '\n';
	else
		cout << "Fast INNER feasibility failed :(" << '\n';
	return success;
}

} // namespace fast_inner_feasibility
