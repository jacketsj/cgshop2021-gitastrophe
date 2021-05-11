#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>

#include "Instance.h"
#include "sparse_graph.h"
#include "graph.h"

using namespace std;

namespace greedy_improver {


// takes valid instance and greedily improves paths taken by entitities
struct greedy_improver {
  graph g;
  instance& ins;
  sparse_graph::sparse_graph sg;
  bool sparse;

  // initialization of both versions shouldn't take too long
	greedy_improver(instance& _ins, bool s = true) : g(_ins), ins(_ins), sg(_ins), sparse(s) {
		if (!verify(ins)) {
			cerr << "WARNING: May not find solution if you don't start with a valid "
							"instance."
					 << endl;
			cerr << "Assuming maximum time needed is " << g.maxt << endl;
		}
	}

  vector<int> mvs;
  void init_mvs() {
    mvs.clear();
    mvs.resize(ins.n);
    for(auto v: ins.moves) {
      for(int i=0;i<ins.n;i++) {
        if (v[i]!=0) mvs[i]++;
      }
    }
  }

	// round robin a bunch
	// returns if score improved at all.
	bool run_rr(int seconds, bool backwards = false, bool random = false, bool ordered = false) {
		const bool update_every_round = false;
    cerr << "running on " << (sparse? "sparse":"dense") << " graph." <<endl;
    if (backwards) { // no need to build if not backwards
      reverse_instance(ins);
      if (sparse) sg.build();
      else g.build();
    }
    // sample runtime of one iteration
		auto start_time = chrono::system_clock::now();
		bool improve = (sparse? sg.find_best(0) : g.find_best(0));
    // limit is for test cases that are small, so they run for 1s before write
		int LIMIT = max(1, 1000/int(std::chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now() - start_time).count()));
    int R = 0;
		while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now() - start_time).count() < 1000*seconds) {
			if (R%10==1) cerr << "RUNNING ROUND " << R
        << " FOR " << ins.name << ", CUR SCORE IS: " << score(ins, 0) << " AND " << score(ins, 1) << 
					" HAVE " << (improve? "": "NOT") << " IMPROVED." << endl;
      vector<int> perm;
      for (int r=0;r<ins.n;r++) perm.push_back(r);
      if (random) {
        auto rng = mt19937();
        shuffle(perm.begin(), perm.end(), rng);
      }
			for (int i = 0; i < LIMIT; i++) {
        if (ordered) { // instance should be updated at this point
          init_mvs();
          // sort in decreasing order of pathlength
          sort(perm.begin(), perm.end(), [&](int a, int b) {
                return mvs[a] > mvs[b];
              });
        }
				for (int r = 0; r < ins.n; r++) {
          improve |= (sparse? sg.find_best(perm[r]) : g.find_best(perm[r]));
				}
        if (!improve) break;
			}
			if (!improve) {
        cerr << "*** Reached optimality for " << ins.name << endl;
        break; // break early if no improvement found.
      }
			if (update_every_round) {
				if (sparse) sg.update_instance();
				else g.update_instance();
				if (backwards) {
					reverse_instance(ins);
				}
				ins.write();
				if (backwards) reverse_instance(ins);
			}
			++R;
		}
    if (sparse) sg.update_instance();
    else g.update_instance();
    if (backwards) reverse_instance(ins);
		ins.write(); //write if we didn't
		return improve;
	}
};

// returns if improvement found
bool round_robin_improve(instance& ins, int seconds, bool sparse = false) {
	greedy_improver gd(ins, sparse);
	bool improve = gd.run_rr(seconds);
	return improve;
}

bool random_improve(instance& ins, int seconds, bool sparse = false) {
	greedy_improver gd(ins, sparse);
	bool improve = gd.run_rr(seconds, false, true);
	return improve;
}

bool longest_improve(instance& ins, int seconds, bool sparse = false) {
	greedy_improver gd(ins, sparse);
	bool improve = gd.run_rr(seconds, false, false, true);
	return improve;
}

} // namespace greedy_improver
