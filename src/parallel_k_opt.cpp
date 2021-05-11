#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"
#include "k_perm_solver.h"

using namespace std;

int NUM_THREADS = 40;

mutex mtx;
unordered_set<string> active;
unordered_set<string> done;

void run(const string& file, bool makespan, bool custom) {
	{
		unique_lock l(mtx);
		if (active.count(file))
			return;
		active.insert(file);
	}
	cerr << "Optimizing " << file <<endl;
	const int seconds = 600 * NUM_THREADS;
	instance ins;
	if (custom)
		ins.read_custom(file);
	else {
		ins.read(file);
		ins.read_out(makespan);
	}
	int oldscore = score(ins, makespan);
	int time = clock();
	if (makespan) {
		const int R = 22;
		const int N = 7;
		// run 1 opt a bunch first
		k_perm_solver::run(ins, seconds / 3, true, 1, R);
		for (int k = 1; k <= N; k++) {
			k_perm_solver::run(ins, seconds / (3 * N), true, k, R);
		}
		for (int k = N; k >= 1; k--) {
			k_perm_solver::run(ins, seconds / (3 * N), true, k, R);
		}
	} else {
		const int R = 20;
		const int N = 4;
		for (int k = 1; k <= N; k++) {
			k_perm_solver::run(ins, seconds / N, false, k, R);
		}
		for (int k = N; k >= 1; k--) {
			k_perm_solver::run(ins, seconds / N, false, k, R);
		}
	}

	int newscore = score(ins, makespan);
	cout << file << " done: " << oldscore << " -> " << newscore << endl;
	ins.write();
	ins.improve_only_debug_write(file, makespan);
	if (oldscore >= newscore) {
		if (oldscore > newscore) {
			cout << "Found improvement for " << file << " from " << oldscore << " to "
					 << newscore << " in " << double(clock() - time) / CLOCKS_PER_SEC
					 << endl;
			done.erase(file + to_string(makespan));
		} else {
			done.insert(file + to_string(makespan));
		}
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
	bool custom = false;
	bool makespan = true;
	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			if (string(argv[i]) == string("-c"))
				custom = true;
			else if (string(argv[i]) == string("-d"))
				makespan = false;
			else
				NUM_THREADS = stoi(string(argv[i]));
		}
	}

	mt19937 rng(time(0));
	string filename;
	vector<string> fs;
	while (cin >> filename) {
		// filename = remove_ext(filename); // this happens in read anyway
		fs.push_back(filename);
	}
	thread_pool tp(NUM_THREADS);
	while (1) {
		for (const auto& file : fs) {
			// bool makespan = 1; // rng() % 2; // TODO revert this at some point
			{
				unique_lock l(tp.m);
				tp.qfull.wait(l, [&tp]() { return tp.q.size() < tp.QSZ; });
			}
			cerr << (int)fs.size() - done.size() << " instances remaining." << endl;
			packaged_task<void()> p(bind(run, file, makespan, custom));
			tp.add(move(p));
		}
	}
}
