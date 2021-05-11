#include <iostream>
#include <string>

#include "Instance.h"
#include "ThreadPool.h"

using std::cin;
using std::string;

void run(string name) {
	instance i;
	i.read_custom(name);
	cerr << "Saving " << name << endl;// << " has score of " << score(i, 1) << ", " << score(i, 0) << endl;

	i.write();
}

int main(int argc, char* argv[]) {
	string name;
	thread_pool tp(8);
	while (cin >> name) {
			{
				unique_lock l(tp.m);
				tp.qfull.wait(l, [&tp]() { return tp.q.size() < tp.QSZ; });
			}
			packaged_task<void()> p(bind(run, name));
			tp.add(move(p));
	}
}
