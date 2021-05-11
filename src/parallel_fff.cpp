#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"
#include "fast_direct_feasibility.h"

using namespace std;

int NUM_THREADS = 20;

mutex mtx;
unordered_set<string> active;
unordered_set<string> done;

void run(const string& file, bool reverse) {
	{
		unique_lock l(mtx);
		if (active.count(file))
			return;
		active.insert(file);
	}
	instance ins;
	ins.read(file);
	string redoloc = OUTPATH + string("redo4/") + ins.name + OUTEXT;
	cerr << "RUN " << file << endl;

	fast_direct_feasibility::run_fast_direct_feasibility(ins, reverse);
	ins.write();

	cerr << "Completed DIRECT re-initialization of " << ins.name
			 << ", writing it to " << redoloc << endl;
	ins.improve_only_debug_write(redoloc, true);
	// ins.debug_write(redoloc);

	ins.write();

	{
		unique_lock l(mtx);
		active.erase(file);
		done.insert(file); // for now, always finish after one run
	}
}

// runs file to optimize distance of an instance
int main(int argc, char* argv[]) {
	mt19937 rng(time(0));
	bool forever = false;
	bool reverse = false;
	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			if (string(argv[i]) == string("-r"))
				reverse = true;
			else if (string(argv[i]) == string("-f"))
				forever = true;
			else
				NUM_THREADS = stoi(string(argv[i]));
		}
	}
	string filename;
	vector<string> fs;
	while (cin >> filename) {
		filename = remove_ext(filename);
		fs.push_back(filename);
	}
	cerr << "RUNNING ON FILES: ";
	for (auto& f : fs) cerr << f << " "; cerr << endl;
	/*
	vector<thread> ts;
	for (const auto& file : fs) {
		ts.emplace_back(bind(run, file, reverse));
	}
	for (thread& t : ts)
		t.join();
	*/
	thread_pool tp(NUM_THREADS);
	do {
		for (const auto& file : fs) {
			{
				unique_lock l(tp.m);
				tp.qfull.wait(l, [&tp]() { return tp.q.size() < tp.QSZ; });
			}
			{
				unique_lock l(mtx);
				if (done.count(file))
					continue;
			}
			packaged_task<void()> p(bind(run, file, reverse));
			tp.add(move(p));
		}
	} while (forever);
}
