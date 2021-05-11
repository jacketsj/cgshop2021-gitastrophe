#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "Instance.h"

using namespace std;

struct node {
	int x, y, t, d;
	// (robot, xpos, ypos, time, dir)
	// dir == 5 is undecided
	node() {}
	node(int x, int y, int t, int d) : x(x), y(y), t(t), d(d) {}
	const bool operator<(const node& o) const {
		return make_tuple(t, x, y, d) < make_tuple(o.t, o.x, o.y, o.d);
	}
};

struct state {
	int r;
	node n;
	// (robot, xpos, ypos, time, dir)
	// dir == 5 is undecided
	state() {}
	state(int r, int x, int y, int t, int d) : r(r), n(x, y, t, d) {}
	const bool operator<(const state& o) const {
		if (r != o.r) return r < o.r;
		return n < o.n;
	}
};

int main() {
	string filename;
	cin >> filename;
	filename = remove_ext(filename);

	GRBEnv env(true);
	env.start();
	GRBModel model(env);

	map<state, GRBVar> Y;
	map<node, GRBLinExpr> Z;
	instance ins;
	ins.read(filename);


	const int minx = -5, maxx = 15, miny = -5, maxy = 15;
	auto in_bounds = [&](int x, int y) {
		return minx <= x && x <= maxx && miny <= y && y <= maxy;
	};

	// flow constraints
	int maxT = 30;
	
	for (int t = maxT; t >= 0; t--) {
		cerr << t << endl;
		for (int x = minx; x <= maxx; x++) {
			for (int y = miny; y <= maxy; y++) {
				// Each robot moves in the direction they've decided in
				for (int d = 0; d <= 5; d++) {
					vector<GRBVar> terms;
					vector<double> coeffs;
					for (int r = 0; r < ins.n; r++) {
						state s(r, x, y, t, d);
						// stringstream ss;
						// ss << "(" << x << "," << y << "," << t << "," << d << ")";
						Y[s] = model.addVar(0, 1, 0, GRB_BINARY);//, ss.str());
						
						if (d != 5 && t < maxT && in_bounds(x + dx[d], y + dy[d])) {
							state ns(r, x + dx[d], y + dy[d], t + 1, 5);
							// stringstream ss;
							// ss << "Adding constraint. Most move to " << x + dx[d] << " " << y + dy[d] << " " << d 
								 // << " from " << x << " " << y << " " << d;
							assert(Y.count(ns));
							model.addConstr(Y[s] <= Y[ns]);//, ss.str());

							// assert(Z.count(s.n));
							terms.push_back(Y[s]);
							coeffs.push_back(1);
						}
					}

					// Each state can only be occupied by at most 1 robot
					node curr(x, y, t, d);
					Z[curr].addTerms(&coeffs[0], &terms[0], terms.size());
					model.addConstr(Z[curr] <= 1);
				}
			}
		}
	}

	// Each square must take in a robot if its active
	for (int t = maxT; t >= 1; t--) {
		cerr << t << endl;
		for (int x = minx; x <= maxx; x++) {
			for (int y = miny; y <= maxy; y++) {
				// Each robot moves in the direction they've decided in
				for (int r = 0; r < ins.n; r++) {
					state s(r, x, y, t, 5);
					GRBLinExpr expr;
					for (int d = 0; d <= 4; d++) {
						if (in_bounds(x - dx[d], y - dy[d])) {
							state ns(r, x - dx[d], y - dy[d], t - 1, d);
							assert(Y.count(ns));
							expr += Y[ns];
						}
					}
					assert(Y.count(s));
					model.addConstr(Y[s] <= expr);
				}
			}
		}
	}
	// cerr << Z.size() << " " << 11 * 11 * 6 * 13 << endl;

	for (int t = maxT-1; t >= 0; t--) {
		cerr << t << endl;
		for (int x = minx; x <= maxx; x++) {
			for (int y = miny; y <= maxy; y++) {
				// // No two robots can collide, unless they are moving in the same direction
				if (t < maxT) {
					for (int d0 = 1; d0 <= 4; d0++) {
						assert(Z.count(node(x, y, t, d0)));
						GRBLinExpr cons = Z[node(x, y, t, d0)];
						for (int d1 = 0; d1 <= 4; d1++) {
							if (d0 == d1) continue;
							if (!in_bounds(x + dx[d0], y + dy[d0])) continue;
							
							assert(Z.count(node(x + dx[d0], y + dy[d0], t, d1)));
							cons += Z[node(x + dx[d0], y + dy[d0], t, d1)];
						}
						model.addConstr(cons <= 1);
					}
				}

				// Each robot must pick a direction
				for (int r = 0; r < ins.n; r++) {
					GRBLinExpr expr;
					for (int d = 0; d <= 4; d++) {
						state u(r, x, y, t, d);
						assert(Y.count(u));
						expr += Y[u];
					}
					state v(r, x, y, t, 5);
					assert(Y.count(v));
					// stringstream ss;
					// ss << "The value at " << v.n.x << " " << v.n.y << " " << v.n.t << " must diffuse";
					model.addConstr(expr == Y[v]);//, ss.str());
				}
			}
		}
	}

	// Each robot must go somewhere
	for (int t = maxT-1; t >= 0; t--) {
		for (int i = 0; i < ins.n; i++) {
			vector<GRBVar> terms;
			vector<double> coeffs;
			for (int x = minx; x <= maxx; x++) {
				for (int y = miny; y <= maxy; y++) {
					terms.push_back(Y[state(i, x, y, t, 5)]);
					coeffs.push_back(1);				
				}
			}

			GRBLinExpr expr;
			expr.addTerms(&coeffs[0], &terms[0], terms.size());
			model.addConstr(expr == 1);//, "Each robot must go somewhere");
		}
	}

	cerr << "Done adding variables" << endl;

	// input output objectives
	for (int i = 0; i < ins.n; i++) {
		pt s = ins.start[i];
		pt t = ins.target[i];
		state st(i, s.x, s.y, 0, 5);
		state en(i, t.x, t.y, maxT, 5);
		// cerr << "Start " << i << ": " << s.x << " " << s.y << endl;
		// cerr << "End " << i << ": " << t.x << " " << t.y << endl;
		assert(Y.count(st) && Y.count(en));
		model.addConstr( Y[st] == 1);//, "start is 1" );
		model.addConstr( Y[en] == 1);//, "start is 1" );
	}

	for (int i = 0; i < ins.n; i++) {
		for (int x = minx; x <= maxx; x++) {
			for (int y = miny; y <= maxy; y++) {
				if (x != ins.start[i].x || y != ins.start[i].y) {
					state st(i, x, y, 0, 5);
					assert(Y.count(st));
					model.addConstr( Y[st] == 0 );//, "non-start is 0" );
				}

				if (x != ins.target[i].x || y != ins.target[i].y) {
					state en(i, x, y, maxT, 5);
					assert(Y.count(en));
					model.addConstr( Y[en] == 0 );//, "non-start is 0");
				}
			}
		}
	}

	// // makespan minimizing objective
	// vector<GRBVar> d(maxT+1);
	// cerr << d.size() << endl;
	// for (int t = maxT; t >= 0; t--) {
	// 	d[t] = model.addVar(0, 1, 1, GRB_BINARY);//, "d" + to_string(t));
	// 	GRBLinExpr expr;
	// 	for (int r = 0; r < ins.n; r++) {
	// 		for (int x = minx; x <= maxx; x++) {
	// 			for (int y = miny; y <= maxy; y++) {
	// 				for (int d = 1; d <= 4; d++) {
	// 					expr += Y[state(r, x, y, t, d)];
	// 				}
	// 			}
	// 		}
	// 	}
	// 	model.addConstr( ins.n * 30 * 30 * 4 * d[t] >= expr );
	// }

	cerr << "About to optimize..." << endl;
	model.set(GRB_IntParam_SolutionLimit, 1);
	model.set(GRB_IntParam_MIPFocus, 1);
	// model.set(GRB_DoubleParam_TimeLimit, 60);
	model.optimize();
	// model.computeIIS();
	// model.write("test.ilp");
	
	// for (auto kv : Y) {
	// 	auto s = kv.first;
	// 	if (kv.second.get(GRB_DoubleAttr_X) > 0) {
	// 		cerr << s.r << " " << s.n.t << " " << s.n.x << " " << s.n.y << " " << s.n.d << endl;
	// 	}
	// }

	// Let's trace the paths
	ins.time = maxT;
	ins.moves.resize(maxT);
	vector<pt> c = ins.start;
	for (int t = 0; t < maxT; t++) {
		ins.moves[t].resize(ins.n);
		for (int i = 0; i < ins.n; i++) {
			//cerr << "looking for move at time " << t << " for robot " << i << endl;
			//cerr << "current position: " << c[i].x << " " << c[i].y << endl;
			int d = 0;
			while (true) {
				assert(Y.count(state(i, c[i].x, c[i].y, t, d)));
				auto& var = Y[state(i, c[i].x, c[i].y, t, d)];
				if (var.get(GRB_DoubleAttr_X) > 0) {
					break;
				}
				d++;
			}
			c[i] = c[i] + dxy[d];
			ins.moves[t][i] = d;
		}
	}
	ins.write();
	return 0;
}