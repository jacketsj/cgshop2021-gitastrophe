#include <bits/stdc++.h>

#include "Instance.h"
#include <dirent.h>

using namespace std;

string bounds_file = "../src/score-bounds.txt";
ofstream out(bounds_file, ofstream::out | ofstream::app);
map<string, map<int, int>> dcache;
map<string, pair<double, double>> scores;

struct dist_comp {
	instance& ins;
	set<pt> obstacles;
	dist_comp(instance& ins) : ins(ins) {
		for (pt p : ins.obstacle) {
			obstacles.insert(p);
		}
	}

	int dist(pt s, pt t) {
		map<pt, int> D;
		auto comp = [&](const pt& u, const pt& v) {
			int s0 = D[u] + abs(u - t);
			int s1 = D[v] + abs(v - t);
			if (s0 != s1)
				return s0 < s1;
			return u < v;
		};

		set<pt, decltype(comp)> q(comp);
		D[s] = 0;
		q.insert(s);

		while (!q.empty()) {
			auto c = *q.begin();
			q.erase(q.begin());
			if (c == t)
				break;
			for (int i = 1; i <= 4; i++) {
				auto nc = c + dxy[i];
				if (obstacles.count(nc))
					continue;

				if (!D.count(nc) || D[nc] > D[c] + 1) {
					q.erase(nc);
					D[nc] = D[c] + 1;
					q.insert(nc);
				}
			}
		}
		return D[t];
	}
};

int bound(instance& ins, bool makespan) {
	int val = 0;
	dist_comp D(ins);
	for (int i = 0; i < ins.n; i++) {
		int d;
		if (dcache.count(ins.name) && dcache[ins.name].count(i)) {
			d = dcache[ins.name][i];
		} else {
			d = D.dist(ins.start[i], ins.target[i]);
			out << ins.name << " " << i << " " << d << endl;
			dcache[ins.name][i] = d;
		}
		if (makespan)
			val = max(val, d);
		else
			val = val + d;
	}
	return val;
}

void compute_scores(string dirname, bool makespan) {
	struct dirent* entry = nullptr;
	DIR* dp = nullptr;

	dp = opendir(dirname.c_str());
	if (dp != nullptr) {
		while ((entry = readdir(dp))) {
			if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
				continue;

			instance ins;
			ins.read(string(entry->d_name));
			ins.read_out(makespan);

			double bd = double(bound(ins, makespan)) / score(ins, makespan);
			if (makespan)
				scores[string(entry->d_name)].first = bd;
			else
				scores[string(entry->d_name)].second = bd;
			fprintf(stderr, "%s %f %i\n", entry->d_name, bd, makespan);
		}
	}
	closedir(dp);
}

int main() {
	// Load up the cache
	ifstream in(bounds_file);
	string ins_name;
	int id, dist;
	while (in >> ins_name >> id >> dist) {
		dcache[ins_name][id] = dist;
	}

	string msfiles = "../output/makespan";
	string dsfiles = "../output/distance";

	compute_scores(msfiles, true);
	compute_scores(dsfiles, false);
	vector<pair<string, pair<double, double>>> vscores(begin(scores),
																										 end(scores));
	sort(begin(vscores), end(vscores), [](const auto& a, const auto& b) {
		return a.second.first < b.second.first;
	});

	cout << setw(50) << "File Name ";
	cout << "| Makespan score |";
	cout << " Distance score";
	cout << endl;
	cout << setw(50 + 33) << setfill('-') << '-' << endl;
	cout.fill(' ');

	double totm = 0, totd = 0;
	for (auto kv : vscores) {
		cout << setw(49) << kv.first << " ";
		cout << "| ";
		cout << setw(14) << setprecision(6) << kv.second.first;
		cout << " | ";
		cout << setw(14) << setprecision(6) << kv.second.second;
		cout << endl;

		totm += kv.second.first;
		totd += kv.second.second;
	}
	cout << setw(50 + 33) << setfill('-') << '-' << endl;
	cout.fill(' ');

	cout << setw(50) << "Total Score ";
	cout << "| ";
	cout << setw(14) << setprecision(6) << totm;
	cout << " | ";
	cout << setw(14) << setprecision(6) << totd;
	cout << endl;
	cout.fill(' ');
}
