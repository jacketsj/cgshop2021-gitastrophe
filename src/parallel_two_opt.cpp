#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"
#include "physics_solver.h"
#include "two_opt_solver.h"

using namespace std;

const int NUM_THREADS = 16;

mutex mtx;
unordered_set<string> active;
unordered_set<string> done;

void run(const string& file, bool makespan) {
	{
		unique_lock l(mtx);
		if (active.count(file))
			return;
		active.insert(file);
	}
	const int seconds = 800 * NUM_THREADS;
	instance ins;
	ins.read(file);
	ins.read_out(makespan);
	int oldscore = score(ins, makespan);
	int time = clock();
	two_opt_solver::run(ins, seconds, false); // returns something, unused
	int newscore = score(ins, makespan);
	if (oldscore > newscore) {
		cout << "Found improvement for " << file << " from " << oldscore << " to "
				 << newscore << " in " << double(clock() - time) / CLOCKS_PER_SEC
				 << endl;
		ins.write(); // no need to write, greedy improver writes anyways
	} else {
		done.insert(file + to_string(makespan));
	}
	{
		unique_lock l(mtx);
		active.erase(file);
	}
}

// runs file to 2-optimize distance of an instance
int main(int argc, char* argv[]) {
	mt19937 rng(time(0));
	string filename;
	vector<string> fs;
	while (cin >> filename) {
		filename = remove_ext(filename);
		fs.push_back(filename);
	}
	thread_pool tp(NUM_THREADS);
	while (1) {
		for (const auto& file : fs) {
			bool makespan = rng() % 2;
			if (done.count(file + to_string(makespan)))
				continue;
			{
				unique_lock l(tp.m);
				tp.qfull.wait(l, [&tp]() { return tp.q.size() < tp.QSZ; });
			}
			cerr << (int)fs.size() * 2 - done.size() << " instances remaining."
					 << endl;
			packaged_task<void()> p(bind(run, file, makespan));
			tp.add(move(p));
		}
	}
}
