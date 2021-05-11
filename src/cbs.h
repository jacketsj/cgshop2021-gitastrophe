#pragma once

#include <algorithm>
#include <ctime>
#include <deque>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <unordered_map>
#include <variant>
#include <vector>

#include "Instance.h"

using std::cerr;
using std::deque;
using std::make_optional;
using std::map;
using std::optional;
using std::queue;
using std::set;
using std::shuffle;
using std::unordered_map;
using std::variant;
using std::vector;

namespace cbs {

template <class... Ts> struct overloaded : Ts... {
	using Ts::operator()...;
};																													 // (1)
template <class... Ts> overloaded(Ts...)->overloaded<Ts...>; // (2)

struct move {
	pt s; // start
	int dir;
	int time; // arrival time
	// exists just for vectors (which for some reason need a default constructor?)
	move() : s(0, 0), dir(-1), time(-1) {}
	move(const pt& s, int dir, int time) : s(s), dir(dir), time(time) {}
	bool operator<(const move& oth) const {
		return tie(s, dir, time) < tie(oth.s, oth.dir, oth.time);
	}
	pt dest() { return s + dxy[dir]; }
};
struct loc {
	pt s;
	int time;
	loc(const pt& s, int time) : s(s), time(time) {}
	bool operator<(const loc& oth) const {
		return tie(s, time) < tie(oth.s, oth.time);
	}
};
struct conflict {
	int a1, a2;
	variant<loc, pair<move, move>> c;
	conflict(int a1, int a2, loc c_loc) : a1(a1), a2(a2), c(c_loc) {}
	conflict(int a1, int a2, move c_move1, move c_move2)
			: a1(a1), a2(a2), c(make_pair(c_move1, c_move2)) {}

	void print() {
		cout << "agents: " << a1 << ' ' << a2 << ", ";
		if (holds_alternative<loc>(c))
			cout << "location: " << get<loc>(c).s.x << ' ' << get<loc>(c).s.y
					 << " time=" << get<loc>(c).time << endl;
		else {
			move m1, m2;
			tie(m1, m2) = get<pair<move, move>>(c);
			cout << "move1: " << m1.s.x << ' ' << m1.s.y << ' ' << m1.dir
					 << " time=" << m1.time << endl;
			cout << "move2: " << m2.s.x << ' ' << m2.s.y << ' ' << m2.dir
					 << " time=" << m2.time << endl;
		}
	}
};

bool use_makespan = true;
struct cbs_instance {
	int time;
	// each is a vector of agents
	vector<vector<move>> paths;
	vector<set<loc>> agent_loc_constraints;
	vector<set<move>> agent_move_constraints;
	int distance = 0;
	bool operator<(const cbs_instance& oth) const {
		// THIS IS WHERE TO CHANGE FOR MAKESPAN VS DISTANCE
		// change to sum of path sizes for distance
		if (use_makespan)
			return time > oth.time;
		else
			return distance > oth.distance;
	}
	cbs_instance(int n)
			: time(0), paths(n), agent_loc_constraints(n), agent_move_constraints(n) {
	}
	void remove_path(int i) {
		for (auto& mv : paths[i]) {
			if (mv.dir != 0)
				--distance;
		}
		paths[i].clear();
	}
};

vector<move> reverse_path(pt dest, int time,
													unordered_map<int, map<pt, move>>& visited, int& dist,
													int still_time) {
	vector<move> ret;
	while (time > 0) {
		auto vis_entry = visited[min(time, still_time)][dest];
		ret.emplace_back(vis_entry);
		dest = vis_entry.s;
		if (vis_entry.dir != 0)
			++dist;
		--time;
	}
	reverse(ret.begin(), ret.end());
	return ret;
}

struct cbs {
	instance& ins;

	// vector indexed by agents, todo queue of pairs of points and distances
	vector<queue<pair<pt, int>>> score_todo;
	vector<map<pt, int>> score_distances;

	set<pt> obstacles;
	priority_queue<cbs_instance>
			ci_queue; // queue of instances, TODO: best-first search
	cbs(instance& ins) : ins(ins), score_todo(ins.n), score_distances(ins.n) {
		for (pt& p : ins.obstacle)
			obstacles.insert(p);

		for (int i = 0; i < ins.n; ++i) {
			score_distances[i][ins.target[i]] = 0;
			for (int dir = 1; dir < 5; ++dir) {
				score_todo[i].emplace(ins.target[i] + dxy[dir], 1);
			}
		}

		// initialize first ci entry
		cbs_instance ci(ins.n);
		for (int i = 0; i < ins.n; ++i) {
			pathfind_agent(ci, i);
		}
		ci_queue.push(ci);
	}

	bool l1_score = false;
	int agent_score(const pt& p, int agent) {
		if (l1_score)
			return abs(p - ins.target[agent]);
		else {
			queue<pair<pt, int>>& todo = score_todo[agent];
			map<pt, int>& distances = score_distances[agent];
			if (obstacles.count(p) > 0)
				return INF;
			if (distances.count(p))
				return distances[p];
			while (!todo.empty()) {
				pt cur = todo.front().first;
				int dist = todo.front().second;
				todo.pop();
				if (distances.count(cur) > 0 || obstacles.count(cur) > 0)
					continue;
				distances[cur] = dist;
				for (int dir = 1; dir < 5; ++dir) {
					todo.emplace(cur + dxy[dir], dist + 1);
				}
				if (cur == p)
					return distances[p];
			}
		}
		cout << "bad stuff happened, test case is impossible? p=" << p.x << ','
				 << p.y << endl;
		cout << "target=" << ins.target[agent].x << ',' << ins.target[agent].y
				 << endl;
		assert(false);
		return -1;
	}

	bool pathfind_agent_distance(cbs_instance& ci, int agent) {
		// path already exists
		if (!ci.paths[agent].empty())
			return true;

		auto& mv_cons = ci.agent_move_constraints[agent];
		auto& lc_cons = ci.agent_loc_constraints[agent];

		// visited stores move that took us to pt
		unordered_map<int, map<pt, move>> visited;
		// queue of score + -time + -dist + move
		// time is added as secondary measure, no other reason
		priority_queue<tuple<int, int, move>> todo;
		auto score = [&](const pt& p) { return agent_score(p, agent); };
		for (int dir = 0; dir < 5; ++dir) {
			if (dir == 0)
				todo.emplace(-score(ins.start[agent]), 0,
										 move(ins.start[agent], dir, 1));
			else
				todo.emplace(-(1 + score(ins.start[agent])), -1,
										 move(ins.start[agent], dir, 1));
		}
		while (!todo.empty()) {
			auto dist = -get<1>(todo.top());
			move cur = get<2>(todo.top());
			todo.pop();
			if (mv_cons.count(cur) > 0)
				continue;
			pt p = cur.s + dxy[cur.dir];
			if (obstacles.count(p) > 0 || lc_cons.count(loc(p, cur.time)) > 0 ||
					visited[min(cur.time, ci.time + 1)].count(p) > 0)
				continue;
			visited[min(cur.time, ci.time + 1)][p] = cur;
			if (p == ins.target[agent] && cur.time >= ci.time) {
				// store newly found path for agent in cbs instance
				ci.paths[agent] =
						reverse_path(p, cur.time, visited, ci.distance, ci.time + 1);
				ci.time = max(ci.time, cur.time);
				return true;
			}
			// cout << "dist=" << dist << ", score(p)=" << score(p) << endl;
			for (int dir = 0; dir < 5; ++dir) {
				if (dir == 0)
					todo.emplace(-(dist + score(p)), -dist, move(p, dir, cur.time + 1));
				else
					todo.emplace(-(dist + 1 + score(p)), -dist - 1,
											 move(p, dir, cur.time + 1));
			}
		}
		return false;
	}

	bool pathfind_agent_makespan(cbs_instance& ci, int agent) {
		// path already exists
		if (!ci.paths[agent].empty())
			return true;

		auto& mv_cons = ci.agent_move_constraints[agent];
		auto& lc_cons = ci.agent_loc_constraints[agent];

		// bad A*
		// visited stores move that took us to pt
		unordered_map<int, map<pt, move>> visited;
		priority_queue<pair<int, move>> todo; // queue of (A*) score + move
		auto score = [&](const pt& p) { return agent_score(p, agent); };
		for (int dir = 0; dir < 5; ++dir)
			todo.emplace(-score(ins.start[agent]), move(ins.start[agent], dir, 1));
		while (!todo.empty()) {
			move cur = todo.top().second;
			todo.pop();
			if (mv_cons.count(cur) > 0)
				continue;
			pt p = cur.s + dxy[cur.dir];
			if (obstacles.count(p) > 0 || lc_cons.count(loc(p, cur.time)) > 0 ||
					visited[cur.time].count(p) > 0)
				continue;
			visited[cur.time][p] = cur;
			if (p == ins.target[agent] && cur.time >= ci.time) {
				// store newly found path for agent in cbs instance
				ci.paths[agent] =
						reverse_path(p, cur.time, visited, ci.distance, cur.time);
				ci.time = max(ci.time, cur.time);
				return true;
			}
			for (int dir = 0; dir < 5; ++dir)
				todo.emplace(-(cur.time + score(p)), move(p, dir, cur.time + 1));
		}
		return false;
	}

	// returns true if succeeded in finding a path (or if already existed)
	bool pathfind_agent(cbs_instance& ci, int agent) {
		if (use_makespan)
			return pathfind_agent_makespan(ci, agent);
		else
			return pathfind_agent_distance(ci, agent);
	}

	void extend_path(vector<move>& path, int agent) {
		// extend the path to not move, handle empty path case
		if (path.empty())
			path.emplace_back(ins.start[agent], 0, path.size() + 1);
		else
			path.emplace_back(path.back().dest(), 0, path.size() + 1);
	}

	// simulate and check for any conflicts (vertex, edge, or turn/geometric
	// conflicts)
	// returns nullopt if none found
	// returns the first conflict
	optional<conflict> find_conflicts(cbs_instance& ci) {
		vector<pt> locations = ins.start;
		map<pt, int> agent_at_point;
		// initialize agent_at_point
		for (int i = 0; i < ins.n; ++i)
			agent_at_point[locations[i]] = i;

		for (int t = 0; t < ci.time; ++t) {
			vector<pt> new_locations = locations;
			map<pt, int> new_agent_at_point;
			// only vertex conflicts need to be checked in the first iteration, since
			// they are problematic for the contents of new_agent_at_point
			for (int i = 0; i < ins.n; ++i) {
				if (int(ci.paths[i].size()) <= t) // extend paths
					extend_path(ci.paths[i], i);
				new_locations[i] = ci.paths[i][t].dest();
				if (new_agent_at_point.count(new_locations[i]) > 0) { // vertex conflict
					auto ret = conflict(i, new_agent_at_point[new_locations[i]],
															loc(new_locations[i], t + 1));
					// cout << "found vertex conflict! ";
					// ret.print();
					return make_optional(ret);
				}
				new_agent_at_point[new_locations[i]] = i;
			}
			// check for edge conflicts and turn conflicts (same thing basically)
			for (int i = 0; i < ins.n; ++i) {
				if (agent_at_point.count(new_locations[i]) > 0 &&
						ci.paths[agent_at_point[new_locations[i]]][t].dir !=
								ci.paths[i][t].dir) {
					auto ret =
							conflict(i, agent_at_point[new_locations[i]], ci.paths[i][t],
											 ci.paths[agent_at_point[new_locations[i]]][t]);
					// cout << "found edge conflict! ";
					// ret.print();
					return make_optional(ret);
				}
			}
			locations = new_locations;
			agent_at_point = new_agent_at_point;
		}
		// cout << "found no conflict" << endl;
		return nullopt;
	}
	void store_movements_in_ins(cbs_instance& ci) {
		for (int t = 0; t < ci.time; ++t) {
			vector<int> move(ins.n);
			for (int i = 0; i < ins.n; ++i)
				move[i] = ci.paths[i][t].dir;
			ins.moves.push_back(move);
		}
		ins.time = ins.moves.size();
	}
	// somehow turns first conflict found into corresponding constraints, and
	// makes both branches?
	bool use_disjoint_split = true;
	bool process_next_cbs_instance() {
		assert(!ci_queue.empty()); // hopefully everything has a solution
		cbs_instance ci = ci_queue.top();
		ci_queue.pop();
		auto conflict_opt = find_conflicts(ci);
		if (conflict_opt.has_value()) {
			auto con = conflict_opt.value();
			cbs_instance ci2 = ci;
			ci.remove_path(con.a1);
			ci2.remove_path(con.a2);
			// TODO: disjoint splitting
			// (for ci2, add constraints for EVERYTHING except a1, and also maybe add
			// a REQUIREMENT to a1 later)
			visit(
					overloaded{
							[&](std::monostate&) {},
							[&](loc& l) {
								ci.agent_loc_constraints[con.a1].insert(l);
								if (use_disjoint_split)
									for (int i = 0; i < ins.n; ++i) {
										if (i != con.a1)
											ci2.agent_loc_constraints[i].insert(l);
									}
								else
									ci2.agent_loc_constraints[con.a2].insert(l);
							},
							[&](pair<move, move>& p_move) {
								ci.agent_move_constraints[con.a1].insert(p_move.first);
								if (use_disjoint_split)
									for (int i = 0; i < ins.n; ++i) {
										if (i != con.a1)
											ci2.agent_move_constraints[i].insert(p_move.second);
									}
								else
									ci2.agent_move_constraints[con.a2].insert(p_move.second);
							},
					},
					con.c);
			if (pathfind_agent(ci, con.a1))
				ci_queue.push(ci);
			if (pathfind_agent(ci2, con.a2))
				ci_queue.push(ci2);
			return false;
		} else {
			// found valid set of movements!
			store_movements_in_ins(ci);
			return true;
		}
	}
};

bool search(instance& ins, int sec = 1) {
	clock_t start_time = clock();
	cbs solver(ins);
	int num_instances_processed = 0;
	while (clock() - start_time < sec * CLOCKS_PER_SEC) {
		// cout << "processing another cbs instance!" << endl;
		++num_instances_processed;
		if (solver.process_next_cbs_instance()) {
			cout << "processed " << num_instances_processed << " WITH SUCCESS!! :)"
					 << endl;
			return true;
		}
	}
	cout << "processed " << num_instances_processed
			 << " instances with no luck :(" << endl;
	return false;
}
} // namespace cbs
