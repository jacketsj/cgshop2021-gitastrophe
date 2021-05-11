#include "model.h"
#include <bits/stdc++.h>
using namespace std;

int main() {
  string filename;
  while (cin >> filename) {
    filename = remove_ext(filename);
    instance ins;
    ins.read(filename);
    Model m(ins);
    m.build_model(10);
    m.run();
    cerr << m.model->get(GRB_DoubleAttr_ObjVal) << endl;
    return 0;
  }
}
