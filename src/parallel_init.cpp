#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"
#include "fast_feasibility.h"
#include "greedy_improve.h"

using namespace std;

const int NUM_THREADS = 10;

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
	const bool dense = 0;
	const int seconds = 320 * NUM_THREADS;
	instance ins;
	ins.read(file);
	// ins.read_out(makespan);
	int oldscore = score(ins, makespan);
	int time = clock();
	fast_feasibility::run_fast_feasibility(ins);
	ins.write();

	string redoloc = OUTPATH + string("redo3/") + ins.name + OUTEXT;
	cerr << "Completed re-initialization of " << ins.name << ", writing it to "
			 << redoloc << endl;
	ins.debug_write(redoloc);

	// greedy_improver::round_robin_improve(ins, seconds, !dense);
	int newscore = score(ins, makespan);
	if (oldscore > newscore) {
		cout << "Found improvement [starting from initialization] for " << file
				 << " from " << oldscore << " to " << newscore << " in "
				 << double(clock() - time) / CLOCKS_PER_SEC << endl;
	}
	ins.write();
	/*
	else {
		done.insert(file + to_string(makespan));
	}
	*/
	done.insert(file +
							to_string(makespan)); // for now, always finish after one run
	{
		unique_lock l(mtx);
		active.erase(file);
	}
}

// runs file to optimize distance of an instance
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
			// cerr << (int)fs.size() * 2 - done.size() << " instances remaining."
			// 		 << endl;
			packaged_task<void()> p(bind(run, file, makespan));
			tp.add(move(p));
		}
	}
}
