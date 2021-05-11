#pragma once

#include <bits/stdc++.h>

#include "Instance.h"

using namespace std;

namespace k_perm_solver_alt {
  struct node {
    pt r;	// robot (-1 means obstacle, 0 means empty, i means i-1th robot
    int t; // time of the graph node
    node(pt r, int time = 0) : r(r), t(time) {}
    const bool operator<(const node& o) const {
      if (t != o.t)
        return t > o.t;
      return r < o.r;
    }
    bool operator==(const node& o) const { return tie(r, t) == tie(o.r, o.t); }
  };
} // namespace k_perm_solver_alt

namespace std {
  template <> struct hash<k_perm_solver_alt::node> {
    size_t operator()(const k_perm_solver_alt::node& n) const {
      return (hash<pt>()(n.r) << 20) ^ n.t;
    }
  };
} // namespace std

namespace k_perm_solver_alt {
  struct k_perm_solver_alt {
    using state = pair<pt, int>;

    static constexpr int GRID = 500;
    static constexpr int OFF = 200;
    template <class V>
      struct arrmap {
        vector<array<array<V, GRID>, GRID>> arr;
        arrmap(int t): arr(t+1) {
          for (auto& vv : arr) {
            for (auto& v : vv) {
              v.fill((V) -1);
            }
          }
        }
        V& operator[](const state& s) {
          return arr[s.second][s.first.x+OFF][s.first.y+OFF];
        }
        V operator[](const state& s) const {
          return arr[s.second][s.first.x+OFF][s.first.y+OFF];
        }
        bool count(const state& s) const {
          return (*this)[s] != -1;
        }
        void erase(const state& s) {
          (*this)[s] = -1;
        }
      };

    instance& ins;
    vector<vector<int>> moves; // first index is robot (transpose of move matrix)
    arrmap<char> blocked_f;
    arrmap<short> blocked_r;
    arrmap<char> blocked_b;
    vector<bitset<GRID>> blocked_s;

    vector<int> span;

    bool spam_stdio;

    k_perm_solver_alt(instance& _ins, bool _spam_stdio)
      : ins(_ins), blocked_f(ins.time), blocked_r(ins.time), blocked_b(ins.time), blocked_s(GRID),
      spam_stdio(_spam_stdio)  {
        if (!verify(ins)) {
          cerr << "Requires valid initial sequence" << endl;
          assert(false);
        }

        moves.resize(ins.n);
        // convert dense moves into per-robot moves
        for (int t = 0; t < ins.time; t++) {
          for (int j = 0; j < ins.n; j++) {
            moves[j].push_back(ins.moves[t][j]);
          }
        }

        // for each robot, add moves as obstacles
        for (int i = 0; i < ins.n; i++) {
          add_path(i);
        }

        // block the obstacles
        for (pt p : ins.obstacle) {
          assert(0 <= p.x + OFF && p.x + OFF < GRID && 0 <= p.y + OFF &&
              p.y + OFF < GRID);
          blocked_s[p.x + OFF][p.y + OFF] = 1;
        }

        // precompute the spans
        span.resize(ins.n);
        for (int i = 0; i < ins.n; i++) {
          span[i] = actual_time(moves[i]);
        }
      }

    void add_path(int r) {
      pt p = ins.start[r];
      int L = max((int)moves[r].size(), ins.time);
      for (int t = 0; t <= L; t++) {
        if (t < (int)moves[r].size()) {
          blocked_f[state(p, t)] = moves[r][t];
          blocked_r[state(p, t)] = r;
          p = p + dxy[moves[r][t]];
          blocked_b[state(p, t)] = moves[r][t];
        } else {
          blocked_f[state(p, t)] = 0;
        }
      }
      assert(p == ins.target[r]);
    }

    // removes the path for a robot
    void remove_path(int r) {
      pt p = ins.start[r];
      int L = max((int)moves[r].size(), ins.time);
      for (int t = 0; t <= L; t++) {
        if (t < (int)moves[r].size()) {
          blocked_f.erase(state(p, t));
          blocked_r.erase(state(p, t));
          p = p + dxy[moves[r][t]];
          blocked_b.erase(state(p, t));
        } else {
          blocked_f.erase(state(p, t));
        }
      }
      assert(p == ins.target[r]);
    }

    bool allowed(node s, int d) {
      node ns(s.r + dxy[d], s.t + 1);
      // Obstacles
      if (blocked_s[ns.r.x + OFF][ns.r.y + OFF])
        return false;

      // Some robot is already there
      if (blocked_f.count(state(ns.r, ns.t)))
        return false;

      // Some robot is there right now and not moving away in the same dir
      if (blocked_f.count(state(ns.r, s.t)) && blocked_f[state(ns.r, s.t)] != d)
        return false;

      // Some robot is moving into our spot and we're not moving away
      if (blocked_b.count(state(s.r, s.t)) && blocked_b[state(s.r, s.t)] != d)
        return false;

      return true;
    }

    bool quantum_allowed(node s, int d, int maxMS, int& phased_robot) {
      node ns(s.r + dxy[d], s.t + 1);
      // Obstacles
      if (blocked_s[ns.r.x + OFF][ns.r.y + OFF])
        return false;

      auto lucky = [&](int i){
        return span[i] < maxMS && (rand() % (maxMS - span[i] + 1));
        // return (rand() % 55378008) / 55378008.0 < 1.0 - double(span[i]) / maxMS;
      };

      // Some robot is already there
      if (blocked_f.count(state(ns.r, ns.t))) {
        phased_robot = blocked_r[state(ns.r, ns.t)];
        if (lucky(phased_robot)) {
          return true;
        }
        return false;
      }

      // Some robot is there right now and not moving away in the same dir
      if (blocked_f.count(state(ns.r, s.t)) && blocked_f[state(ns.r, s.t)] != d) {
        phased_robot = blocked_r[state(ns.r, s.t)];
        if (lucky(phased_robot)) {
          return true;
        }
        return false;
      }

      // Some robot is moving into our spot and we're not moving away
      if (blocked_b.count(state(s.r, s.t)) && blocked_b[state(s.r, s.t)] != d) {
        phased_robot = blocked_r[state(s.r, ns.t)];
        if (lucky(phased_robot)) {
          return true;
        }
        return false;
      }

      return true;
    }

    int actual_time(const vector<int>& moveset) {
      int T = moveset.size();
      while (T > 0 && moveset[T - 1] == 0)
        T--;
      return T;
    }

    // Find path thats near the old path
    // (must evaluate to true on "in_bounds")
    int num_paths = 0;
    vector<int> find_path(int r, int maxMS, double time_left,
        const pair<int, vector<vector<int>>>& in_bounds_p) {
      num_paths++;
      const auto& [mx_id, in_bounds] = in_bounds_p;
      constexpr static int INF = 0x3f3f3f3f;
      vector dist(ins.time + 1, vector(mx_id + 1, INF));
      vector par(ins.time + 1, vector(mx_id + 1, -1));
      auto get = [&](const pt& p) { return in_bounds[p.x + OFF][p.y + OFF]; };

      auto t = ins.target[r];

      node s(ins.start[r], 0);
      vector<vector<node>> q(ins.time + 1);
      dist[s.t][get(s.r)] = 0;
      q[abs(s.r - t)].push_back(s);

      // Max time for search is remaining time
      auto start_time = clock();
      int iter_count = 0;
      bool found = false;
      for (int curdist = abs(s.r - t); curdist <= ins.time; curdist++) {
        while (!q[curdist].empty()) {
          if (iter_count++ % 1024 == 0 &&
              clock() - start_time >= CLOCKS_PER_SEC * time_left)
            goto end;
          s = q[curdist].back();
          q[curdist].pop_back();
          if (s.r == t && s.t >= ins.time) {
            found = true;
            // cerr << "Nodes expanded: " << dist.size() << endl;
            goto end;
          }
          if (s.t >= ins.time) {
            continue;
          }
          int cur = dist[s.t][get(s.r)];
          if (cur + abs(s.r - t) != curdist)
            continue;

          // cerr << s.r << " " << s.t << endl;
          for (int d = 0; d <= 4; d++) {
            if (allowed(s, d)) {
              node ns(s.r + dxy[d], s.t + 1);
              if (get(ns.r) == -1)
                continue;

              int ndist = cur + 1;
              int pri = ndist + abs(ns.r - t);
              if (pri <= ins.time && (ndist < dist[ns.t][get(ns.r)])) {
                dist[ns.t][get(ns.r)] = ndist;
                par[ns.t][get(ns.r)] = d;
                q[pri].push_back(ns);
              }
            }
          }
        }
      }
end:

      if (!found) {
        // cerr << "No path found" << endl;
        return vector<int>();
      }

      vector<int> res;
      while (s.t != 0) {
        auto move = par[s.t][get(s.r)];
        res.push_back(move);
        s.r = s.r - dxy[move];
        s.t--;
      }
      reverse(res.begin(), res.end());

      // cerr << "Actual time: " << actual_time(res) << endl;
      if (actual_time(res) >= maxMS)
        return vector<int>();

      return res;
    }

    pair<int, vector<vector<int>>> create_bounds(pt p, const vector<int>& moveset,
        int R = 8) {
      pair res(0, vector(GRID, vector<int>(GRID, -1)));
      auto& id = res.first;
      for (int i = -1; i < (int)moveset.size(); i++) {
        if (i >= 0)
          p = p + dxy[moveset[i]];
        for (int x = p.x - R / 2; x <= p.x + R / 2; x++) {
          for (int y = p.y - R / 2; y <= p.y + R / 2; y++) {
            if (res.second[x + OFF][y + OFF] == -1) {
              res.second[x + OFF][y + OFF] = id++;
            }
          }
        }
      }
      return res;
    }

    // Tries to swap the order the two paths go in
    // i.e. freezes the first path and then puts down the second
    // or freezes the second and then puts down the first.
    bool solve_k(vector<int> r, double time_left, int R) {
      vector<pair<int, int>> L(r.size());
      int prevMS = 0;
      for (int i = 0; i < (int)r.size(); i++) {
        L[i].first = actual_time(moves[r[i]]);
        L[i].second = r[i];
        prevMS = max(prevMS, L[i].first);
        cerr << L[i].first << " ";
      }
      cerr << endl;
      sort(L.rbegin(), L.rend());

      for (int i = 0; i < (int)r.size(); i++) {
        r[i] = L[i].second;
      }

      // Save the previous moves
      vector<vector<int>> pmove(r.size());
      for (int i = 0; i < (int)r.size(); i++) {
        pmove[i] = moves[r[i]];
        remove_path(r[i]);
      }

      int span = -1;
      int ndone = 0;
      while (ndone < (int)r.size()) {
        auto moveset =
          find_path(r[ndone], prevMS, time_left,
              create_bounds(ins.start[r[ndone]], pmove[ndone], R));

        if (moveset.empty())
          break;

        moves[r[ndone]] = moveset;
        add_path(r[ndone]);
        ndone++;
      }

      // If not feasible, reverse things
      if (ndone < (int)r.size()) {
        for (int i = 0; i < ndone; i++) {
          remove_path(r[i]);
        }

        for (int i = 0; i < (int)r.size(); i++) {
          moves[r[i]] = pmove[i];
          add_path(r[i]);
        }

        return false;
      } else {
        // New pair-span
        for (int i = 0; i < (int)r.size(); i++) {
          span = max(span, actual_time(moves[r[i]]));
        }

        if (spam_stdio)
          cerr << "Better span found: " << prevMS << "->" << span << endl;
        return true;
      }
    }

    vector<int> find_blocker(const int st, const int k, const int R) {
      auto pmove = moves[st];
      remove_path(st);

      auto in_bounds = create_bounds(ins.start[st], pmove, R).second;
      auto get = [&](const pt& p) { return in_bounds[p.x + OFF][p.y + OFF]; };
      // We need randomness in find_blocker, so the way I'll implement that
      // is by allowing `st` to phase through at most 1 robot r with probability
      // 1 - span(r) / span(st).
      typedef pair<int, node> bstate;

      const auto t = ins.target[st];
      auto comp = [&](const bstate& u, const bstate& v) {
        if (u.second == v.second) 
          return u.first < v.first;
        
        int t0 = u.second.t, t1 = v.second.t;
        int d0 = t0 + abs(u.second.r - t), d1 = t1 + abs(v.second.r - t);
        if (d0 != d1) 
          return d0 < d1;
        
        return u.second.r < v.second.r;
      };

      map<bstate, int> dist;
      map<bstate, pair<int, int>> par;

      set<bstate, decltype(comp)> q(comp);
      bstate s(0, node(ins.start[st], 0));
      // cerr << "starting at " << s.second.r.x << " " << s.second.r.y << endl;
      // cerr << "ending at " << t.x << " " << t.y << endl;

      dist[s] = 0;
      q.insert(s);

      bool found = false;
      while (!q.empty()) {
        s = *q.begin();
        auto n = s.second;
        q.erase(q.begin());

        // We're looking to reach the goal 1 time unit earlier
        // so ins.time - 1 here
        if (n.r == t && n.t >= ins.time - 1) {
          found = true;
          // cerr << "Nodes expanded: " << dist.size() << endl;
          break;
        }
 
        if (n.t >= ins.time - 1) {
          continue;
        }

        for (int d = 0; d <= 4; d++) {
          node ns(n.r + dxy[d], n.t + 1);
          if (get(ns.r) == -1)
            continue;

          int phased_robot = -1;
          bool skip_one = s.first < k && quantum_allowed(n, d, span[st], phased_robot);
          bool skip_zero = allowed(n, d);
          // if (skip_one && !skip_zero)
          //   cerr << "Quantum tunnel! Skipped robot " << phased_robot << " " << st << endl;

          if (skip_one || skip_zero) {
            int pri = ns.t + abs(ns.r - t);
            int b = s.first + !skip_zero;
            auto nst = bstate(b, ns);
            if (pri <= ins.time-1 && (!dist.count(nst) || ns.t < dist[nst])) {
              q.erase(nst);
              dist[nst] = ns.t;
              par[nst].first = d;
              par[nst].second = -1;
              if (!skip_zero) {
                assert(phased_robot != -1);
                par[nst].second = phased_robot;
              }
              q.insert(nst);
            }
          }
        }
      }

      add_path(st);
      if (!found) {
        // cerr << "Failed to find a blocker" << endl;
        return vector<int>();
      }

      set<int> res;
      // cerr << "Found a blocker! " << s.first << " " << s.second.t << endl;
      while (s.second.t != 0) {
        auto move = par[s].first;
        if (par[s].second != -1) {
          res.insert(par[s].second);
          s.first--;
        }
        s.second.r = s.second.r - dxy[move];
        s.second.t--;
      }
      // cerr << s.second.r.x << " " << s.second.r.y << endl;
      // cerr << ins.start[st].x << " " << ins.start[st].y << endl;
      assert(s.second.r == ins.start[st]);
      // cerr << s.first << endl;
      return vector<int>(res.begin(), res.end());
    }

    bool optimize(int seconds, int k, int R) {
      if (spam_stdio)
        cerr << "Given a compute budget of " << seconds
          << " seconds. k, R = " << k << ", " << R << endl;
      auto ti = clock();

      srand(time(NULL));

      // recompute distances every time because moves changes
      priority_queue<pair<int, int>> ranked;
      for (int i = 0; i < ins.n; i++) {
        ranked.push({span[i], i});
      }
      
      while (ranked.size() && clock() - ti < seconds * CLOCKS_PER_SEC) {
        auto top = ranked.top();
        int st = top.second;
        // Now find a size k "blocking set", a set of k robots which would
        // decrease the span by k when removed. For now let's just implement
        // k = 1 and find the robot with the smallest span that decreases the
        // span by 1 when removed. This should be randomized to some extent,
        // since if a particular choice of blocking set fails, we should still
        // try to decrease the span of this robot with respect to some other
        // blocking set.
        auto blocker = find_blocker(st, k-1, R);

        // Failed to find an adequate blocker
        if (blocker.empty()) {
          continue;
        }
        cerr << "Found a blocker of size: " << blocker.size() << endl;
        auto samples = blocker;
        samples.push_back(st);

        // reoptimize wrt to two robots
        double curr_time = (clock() - ti) / double(CLOCKS_PER_SEC);
        solve_k(samples, seconds - curr_time, R);
        span[st] = actual_time(moves[st]);
        
        ranked.pop();
        ranked.push({span[st], st});
      }

      // Clean-up moves
      if (spam_stdio)
        cerr << "Old makespan: " << ins.time << endl;
      ins.time = 0;
      for (int i = 0; i < ins.n; i++) {
        ins.time = max(ins.time, (int)moves[i].size());
      }
      ins.moves.resize(ins.time);
      if (spam_stdio)
        cerr << "New makespan: " << ins.time << endl;

      // Transpose moves back into time-major
      for (int i = 0; i < ins.n; i++) {
        for (int t = 0; t < ins.time; t++) {
          if (ins.moves[t].empty())
            ins.moves[t].resize(ins.n);

          if (t >= (int)moves[i].size()) {
            ins.moves[t][i] = 0;
          } else {
            ins.moves[t][i] = moves[i][t];
          }
        }
      }

      bool cont;
      do {
        cont = false;
        if (accumulate(ins.moves.back().begin(), ins.moves.back().end(), 0) ==
            0) {
          ins.moves.pop_back();
          ins.time--;
          cont = true;
        }
      } while (cont);

      return true;
    }
  };

  // returns if improvement found
  bool run(instance& ins, int seconds, int k = 3, int R = 8,
      bool spam_stdio = false) {
    k_perm_solver_alt gd(ins, spam_stdio);
    bool improve = gd.optimize(seconds, k, R);
    //cerr << gd.num_paths << " paths\n";
    return improve;
  }

} // namespace k_perm_solver_alt
