#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"
#include "greedy_improve.h"

using namespace std;

const int NUM_THREADS = 8;

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
	const int seconds = 10;
	instance ins;
	ins.read(file);
	ins.read_out(makespan);
	int oldscore = score(ins, makespan);
	int time = clock();
	bool improved = greedy_improver::longest_improve(ins, seconds, !dense);
	int newscore = score(ins, makespan);
	if (oldscore > newscore) {
		cout << "Found improvement for " << file << " from "<< oldscore << " to " <<  newscore << 
			" in " << double(clock()-time)/CLOCKS_PER_SEC << endl;
		//ins.write(); // no need to write, greedy improver writes anyways
	}
	else {
		done.insert(file+to_string(makespan));
	}
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
			bool makespan = rng()%2;
			if (done.count(file+to_string(makespan)))
			 	continue;
			{
				unique_lock l(tp.m);
				tp.qfull.wait(l, [&tp]() { return tp.q.size() < tp.QSZ; });
			}
			if ((int)fs.size()*2 - done.size()> NUM_THREADS) // don't spam io if busywait
				cerr << (int)fs.size()*2 - done.size() << " instances remaining." << endl;
			packaged_task<void()> p(bind(run, file, makespan));
			tp.add(move(p));
		}
		if (2*fs.size() == done.size()) break;
	}
}
