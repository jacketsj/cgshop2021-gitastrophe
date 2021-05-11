#include "../Instance.h"
#include "gurobi_c++.h"
#include <bits/stdc++.h>
using namespace std;

struct Model {
  vector<vector<vector<vector<vector<GRBVar>>>>> vars; // vars[t][i][x][y][d] = indicator for whether robot i is at (x, y) and going in direction d at time t
  vector<GRBVar> backs; // sink->source edges
  int mnx, mny, mxx, mxy;
  instance ins;
  GRBEnv env;
  unique_ptr<GRBModel> model;
  Model(const instance& ins_): ins(ins_), env(true), model() {
    env.set("LogFile", "ilp.log");
    env.start();
    const int INF = 0x3f3f3f3f;
    mnx = INF;
    mny = INF;
    mxx = -INF;
    mxy = -INF;
    for (pt p : ins.start) {
      mnx = min(mnx, p.x);
      mxx = max(mxx, p.x);
      mny = min(mny, p.y);
      mxy = max(mxy, p.y);
    }
    for (pt p : ins.target) {
      mnx = min(mnx, p.x);
      mxx = max(mxx, p.x);
      mny = min(mny, p.y);
      mxy = max(mxy, p.y);
    }
    for (pt p : ins.obstacle) {
      mnx = min(mnx, p.x);
      mxx = max(mxx, p.x);
      mny = min(mny, p.y);
      mxy = max(mxy, p.y);
    }
    mnx -= 2;
    mny -= 2;
    mxx += 2;
    mxy += 2;
  }
  bool in_grid(int x, int y) {
    return mnx <= x && x <= mxx && mny <= y && y <= mxy;
  }
  void build_model(int T) {
    cerr << "build_model(" << T << ")\n";
    model = make_unique<GRBModel>(env);
    cerr << T << " " << ins.n << " " << mnx << " " << mxx << " " << mny << " " << mxy << endl;
    cerr << "allocating " << 1LL * T * ins.n * (mxx - mnx + 1) * (mxy - mny + 1) * 5 << " variables\n";
    vars = vector(T, vector(ins.n, vector(mxx - mnx + 1, vector(mxy - mny + 1, vector<GRBVar>(5)))));
    int cs = 0;
    for (int t = 0; t < T; t++) {
      for (int i = 0; i < ins.n; i++) {
        for (int x = mnx; x <= mxx; x++) {
          for (int y = mny; y <= mxy; y++) {
            for (int d = 0; d < 5; d++) {
              vars[t][i][x-mnx][y-mny][d] = model->addVar(0., 1., 0., GRB_BINARY, "x");
            }
          }
        }
      }
    }
    backs = vector<GRBVar>(ins.n);
    for (int i = 0; i < ins.n; i++) {
      backs[i] = model->addVar(0., 1., 0., GRB_BINARY, "x");
    }
    // capacity constraint
    for (int t = 0; t < T; t++) {
      for (int x = mnx; x <= mxx; x++) {
        for (int y = mny; y <= mxy; y++) {
          for (int d = 0; d < 5; d++) {
            GRBLinExpr expr;
            for (int i = 0; i < ins.n; i++) {
              expr += vars[t][i][x-mnx][y-mny][d];
            }
            model->addConstr(expr <= 1, "capacity");
            cs++;
          }
        }
      }
    }
    // flow conservation constraint
    for (int t = 0; t < T; t++) {
      for (int i = 0; i < ins.n; i++) {
        for (int x = mnx; x <= mxx; x++) {
          for (int y = mny; y <= mxy; y++) {
            GRBLinExpr out, in;
            for (int d = 0; d < 5; d++) {
              out += vars[t][i][x-mnx][y-mny][d];
              int nx = x + dx[d], ny = y + dy[d];
              if (in_grid(nx, ny)) {
                in += vars[t][i][nx-mnx][ny-mny][opposite_dir[d]];
              }
            }
            if (t == 0 && pt(x, y) == ins.start[i]) {
              in += backs[i];
            }
            if (pt(x, y) == ins.target[i]) {
              out += backs[i];
            }
            model->addConstr(out == in, "flow conservation");
            cs++;
          }
        }
      }
    }
    // disallow robots entering same cell
    for (int t = 0; t < T; t++) {
      for (int x = mnx; x <= mxx; x++) {
        for (int y = mny; y <= mxy; y++) {
          GRBLinExpr expr;
          for (int i = 0; i < ins.n; i++) {
            for (int d = 0; d < 5; d++) {
              expr += vars[t][i][x-mnx][y-mny][d];
            }
            if (pt(x, y) == ins.target[i]) {
              expr += backs[i];
            }
          }
          model->addConstr(expr <= 1, "vertex capacity");
          cs++;
        }
      }
    }
    // disallow robots bumping into each other
    for (int t = 0; t < T; t++) {
      for (int x = mnx; x <= mxx; x++) {
        for (int y = mny; y <= mxy; y++) {
          for (int d = 0; d < 5; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (!in_grid(nx, ny)) continue;
            for (int i = 0; i < ins.n; i++) {
              for (int j = i+1; j < ins.n; j++) {
                model->addConstr(vars[t][i][x-mnx][y-mny][d] + vars[t][j][nx-mnx][ny-mny][opposite_dir[d]] <= 1, "geometric restriction");
                cs++;
              }
            }
          }
        }
      }
    }
    GRBLinExpr obj;
    for (int i = 0; i < ins.n; i++) {
      obj += backs[i];
    }
    model->setObjective(obj, GRB_MAXIMIZE);
    cerr << "finish build_model with " << cs << " constraints" << endl;
  }
  void run() {
    model->optimize();
  }
};
