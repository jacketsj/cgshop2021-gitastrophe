#pragma once

#include <bits/stdc++.h>
using namespace std;

struct thread_pool {
  static const int QSZ = 10;
  vector<thread> ts;
	queue<packaged_task<void()>> q;
	mutex m;
  condition_variable qempty;
  condition_variable qfull;
	bool active;
	thread_pool(int threads) {
		active = 1;
		auto loop = [&]() {
			while (active) {
				bool has = 0;
				packaged_task<void()> p;
				{
					lock_guard<mutex> l(m);
					if (!q.empty()) {
						has = 1;
						p = move(q.front());
						q.pop();
            qempty.notify_one();
            qfull.notify_one();
					}
				}
				if (has) p();
				else this_thread::yield();
			}
		};
    ts.resize(threads);
		for (int i = 0; i < threads; i++) {
			ts[i] = thread(loop);
		}
	}
	void add(packaged_task<void()>&& p) {
		lock_guard<mutex> l(m);
		q.push(move(p));
	}
	~thread_pool() {
    {
      unique_lock l(m);
      qempty.wait(l, [this]() { return q.empty(); });
    }
		active = 0;
		for (auto& t : ts) {
			if (t.joinable()) t.join();
		}
	}
};

