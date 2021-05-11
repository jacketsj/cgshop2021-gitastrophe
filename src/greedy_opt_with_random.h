#pragma once

#include <algorithm>
#include <ctime>
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#include "Instance.h"

using std::cerr;
using std::map;
using std::shuffle;
using std::unordered_map;
using std::vector;

namespace greedy {
struct state {
	vector<pt> positions;
	vector<int> planned_moves;
	unordered_map<int, unordered_map<int, int>> robot_positions;
#ifdef FLOYD
	map<pt, map<pt, pair<int, int>>> dist;
#endif
};
} // namespace greedy

struct greedy_optimizer {
	instance& ins;
	mt19937& g;
	greedy::state s;
	unordered_map<int, unordered_map<int, bool>> obstacles;
	greedy_optimizer(instance& ins, std::mt19937& g) : ins(ins), g(g) {
		for (auto& p : ins.obstacle)
			obstacles[p.x][p.y] = true;
		for (auto& p : ins.start) {
			s.positions.push_back(p);
			s.planned_moves.push_back(0);
			int i = s.positions.size() - 1;
			s.robot_positions[s.positions[i].x][s.positions[i].y] = i;
		}
#ifdef FLOYD
		run_fw();
#endif
	}
	bool done() {
		for (int i = 0; i < ins.n; ++i)
			if (s.positions[i] != ins.target[i])
				return false;
		// cout << "done apparently?" << endl;
		// cout << s.positions[0].x << ',' << s.positions[0].y << endl;
		// cout << ins.target[0].x << ',' << ins.target[0].y << endl;
		// cout << s.positions[1].x << ',' << s.positions[1].y << endl;
		// cout << ins.target[1].x << ',' << ins.target[1].y << endl;
		return true;
	}
	bool adjacent(const pt& p1, const pt& p2) { return dist(p1, p2) == 1; }
#ifdef FLOYD
	// bool is_finite_dist(const pt& p1, const pt& p2) {
	//	int k = 0;
	//	if (collide_obstacle(p1))
	//		++k;
	//	if (collide_obstacle(p2))
	//		++k;
	//	if (adjacent(p1, p2)) {
	//		s.dist[p1][p2] = make_pair(k, 1);
	//		return true;
	//	}
	//	return s.dist.count(p1) > 0 && s.dist[p1].count(p2) > 0;
	//}
	pair<int, int> add(pair<int, int> a, pair<int, int> b) {
		return make_pair(a.first + b.first, a.second + b.second);
	}
	pair<int, int> get_dist(pt& p1, pt& p2) {
		if (s.dist.count(p1) > 0 && s.dist[p1].count(p2) > 0)
			return s.dist[p1][p2];
		return make_pair(ins.n, 0);
	}
#endif
	bool collide_obstacle(const pt& p) { return obstacles[p.x][p.y]; }
	bool robot_in_place(const pt& p) {
		return s.robot_positions.count(p.x) > 0 &&
					 s.robot_positions[p.x].count(p.y) > 0;
	}
	bool collide_robot(const pt& p) { return obstacles[p.x][p.y]; }
#ifdef FLOYD
	void run_fw() { // floyd-warshall
		return;
		if (s.dist.size() != 0 && g() % 4 != 0) // don't always run
			return;

		s.dist.clear();
		// for (pt& p : s.positions) {
		//	s.dist[p][p] = make_pair(0, 0);
		//	// s.dist[p][p] = collide_obstacle(p) ? make_pair(1, 0) : make_pair(0,
		// 0);
		//}
		vector<pt> points;
		int xmin = 0, ymin = 0, xmax = 0, ymax = 0;
		for (pt& p : s.positions) {
			xmin = min(xmin, p.x);
			ymin = min(ymin, p.y);
			xmax = max(xmax, p.x);
			ymax = max(ymax, p.y);
		}

		for (int x = xmin - 2; x <= xmax + 2; ++x)
			for (int y = ymin - 2; y <= ymax + 2; ++y)
				points.emplace_back(x, y);

		for (pt& p : points)
			s.dist[p][p] = make_pair(0, 0);

		for (pt& p : points) {
			for (int i = 1; i < 5; ++i) {
				pt p0 = p + dxy[i];
				// s.dist[p][p0] = make_pair(0, 1);
				s.dist[p][p0] = (collide_obstacle(p0) || robot_in_place(p0))
														? make_pair(1, 1)
														: make_pair(0, 1);
			}
		}

		for (pt& k : points)
			for (pt& i : points)
				for (pt& j : points)
					s.dist[i][j] =
							min(get_dist(i, j), add(get_dist(i, k), get_dist(k, j)));
		// if (is_finite_dist(i, k) && is_finite_dist(k, j) &&
		//		((is_finite_dist(i, j) &&
		//			s.dist[i][j] > add(s.dist[i][k], s.dist[k][j])) ||
		//		 !is_finite_dist(i, j))) {
		//	s.dist[i][j] = add(s.dist[i][k], s.dist[k][j]);
		//}
	}
#endif
	void do_step() {
		s.robot_positions.clear();
		// bool something_moved = false;
		// for (auto i : s.planned_moves)
		//	if (i > 0) {
		//		something_moved = true;
		//		break;
		//	}
		// if (!something_moved) {
		//	for (int i = 0; i < ins.n; ++i)
		//		s.robot_positions[s.positions[i].x][s.positions[i].y] = i;
		//	return;
		//}
		ins.moves.emplace_back(ins.n);
		for (int i = 0; i < ins.n; ++i) {
			s.positions[i] = s.positions[i] + dxy[s.planned_moves[i]];
			ins.moves[ins.moves.size() - 1][i] = s.planned_moves[i];
			s.planned_moves[i] = 0;
			s.robot_positions[s.positions[i].x][s.positions[i].y] = i;
		}
		vector<int> moveless(ins.n, 0);
		if (ins.moves.back() == moveless)
			ins.moves.pop_back();
#ifdef FLOYD
		run_fw();
#endif
	}
	bool valid_move(int robot, int dir) {
		if (dir == 0)
			return true;
		auto p = s.positions[robot] + dxy[dir];
		if (collide_obstacle(p))
			return false;
		if (s.robot_positions.count(p.x) > 0 &&
				s.robot_positions[p.x].count(p.y) > 0) {
			return s.planned_moves[s.robot_positions[p.x][p.y]] == dir;
		}
		return true;
	}
	int random_move(int robot) {
		vector<int> dirs = {0, 1, 2, 3, 4};
		shuffle(dirs.begin(), dirs.end(), g);
		for (int i : dirs) {
			if (valid_move(robot, i))
				return i;
		}
		return -1;
	}
	int dir_of_robot_in_place(const pt& p) { return s.robot_positions[p.x][p.y]; }
	bool robot_nearby(int robot) {
		pt p = s.positions[robot];
		vector<int> dirs = {1, 2, 3, 4};
		for (int dir : dirs)
			if (robot_in_place(p + dxy[dir]))
				return true;
		return false;
	}
	bool still_robot_nearby(int robot) {
		pt p = s.positions[robot];
		vector<int> dirs = {1, 2, 3, 4};
		for (int dir : dirs)
			if (robot_in_place(p + dxy[dir]) &&
					dir_of_robot_in_place(p + dxy[dir]) == 0)
				return true;
		return false;
	}
	bool good_dir(int robot, int dir) {
#ifndef FLOYD
		pt gooddir = ins.target[robot] - s.positions[robot];
		return dot(gooddir, dxy[dir]) > 0;
#endif
#ifdef FLOYD
		pair<int, int> best_dist(ins.n + ins.m + 1,
														 0); // start with something larger than everything
		for (int i = 0; i < 5; ++i) {
			pair<int, int> d = s.dist[s.positions[robot] + dxy[i]][ins.target[robot]];
			best_dist = min(best_dist, d);
		}
		pair<int, int> d = s.dist[s.positions[robot] + dxy[dir]][ins.target[robot]];
		return best_dist == d;
#endif
	}

	int random_good_move(int robot) {
		// only good move when on target is to stay still sortof
		//&& !robot_nearby(robot))
		if (s.positions[robot] == ins.target[robot])
			return 0;
		vector<int> dirs = {1, 2, 3, 4};
		shuffle(dirs.begin(), dirs.end(), g);
#ifndef FLOYD
		if (robot_nearby(robot) && !still_robot_nearby(robot))
			dirs.push_back(0);
#endif
		for (int i : dirs) {
			// if (good_dir(robot, i) && valid_move(robot, i)) {
			if (!ins.moves.empty() && opposite_dir[ins.moves.back()[i]] == i)
				continue;
			// TODO: not opposite of last move (ins.moves.back()[i])
			if (good_dir(robot, i) && valid_move(robot, i)) {
				return i;
			}
		}
		return -1;
	}
	void random_step() {
		vector<int> robs;
		for (int i = 0; i < ins.n; ++i)
			robs.push_back(i);
		shuffle(robs.begin(), robs.end(), g);
		for (int i = 0; i < ins.n; ++i) {
			int robot = robs[i];
			int dir = random_good_move(robot);
#ifndef FLOYD
			if (dir == -1 || g() % ins.n == 0) // or if some randomness? TODO
				dir = random_move(robot);
#endif
			if (dir == -1)
				dir = 0;
			s.planned_moves[robot] = dir;
			auto p = s.positions[robot] + dxy[dir];
			// no way for a future position robot to share a direction, so this is
			// enough to stop collisions at new positions for both robots
			s.robot_positions[p.x][p.y] = robot;
		}
		do_step();
	}
	bool run() {
		const int LIMIT = 400;
		for (int move = 0; move < LIMIT; ++move) {
			random_step();
			if (done()) {
				ins.time = ins.moves.size();
				return true;
			}
		}
		return false;
	}

	int score() {
		int ret = 0;
		for (int i = 0; i < ins.n; ++i) {
			ret += abs(ins.target[i] - s.positions[i]);
		}
		return -ret;
	}
};

namespace greedy {
bool try_a_bunch(instance& ins, int sec = 1) {
	cout << "about to try for " << sec << " seconds" << endl;
	std::random_device rd;
	std::mt19937 g(rd());
	clock_t start_time = clock();
	bool ever_succeeded = false;
	// for (int t = 0; t < A_BUNCH; ++t)
	while (clock() - start_time < sec * CLOCKS_PER_SEC) {
		instance ins_copy = ins;
		greedy_optimizer gopt(ins_copy, g);
		if (gopt.run()) {
			ever_succeeded = true;
			if (ins.time == 0 || ins.time > ins_copy.time) {
				if (ins.time == 0)
					cout << "found a valid solution" << endl;
				ins = ins_copy;
			}
		}
	}
	cout << "done trying a bunch, best time=" << ins.time
			 << " (0 means failed to find a sln)" << endl;
	// cout << "trying a bunch took "
	//		 << double(clock() - start_time) / CLOCKS_PER_SEC << " seconds." <<
	// endl;
	return ever_succeeded;
}

bool a_start_sortof(instance& ins, int sec = 1) {
	cout << "about to 'a star' (not really) for " << sec << " seconds" << endl;
	std::random_device rd;
	std::mt19937 g(rd());
	clock_t start_time = clock();
	bool ever_succeeded = false;
	// for (int t = 0; t < A_BUNCH; ++t)
	while (clock() - start_time < sec * CLOCKS_PER_SEC) {
		instance ins_copy = ins;
		greedy_optimizer gopt(ins_copy, g);
		if (gopt.run()) {
			ever_succeeded = true;
			if (ins.time == 0 || ins.time > ins_copy.time) {
				if (ins.time == 0)
					cout << "found a valid solution" << endl;
				ins = ins_copy;
			}
		}
	}
	cout << "done trying a bunch, best time=" << ins.time
			 << " (0 means failed to find a sln)" << endl;
	// cout << "trying a bunch took "
	//		 << double(clock() - start_time) / CLOCKS_PER_SEC << " seconds." <<
	// endl;
	return ever_succeeded;
}
} // namespace greedy
