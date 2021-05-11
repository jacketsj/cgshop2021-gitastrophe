#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

#include "../Instance.h"

using std::cout;
using std::endl;

using std::string;

struct fpt {
	double x, y;
	fpt(double x, double y) : x(x), y(y) {}
	fpt(const pt& p) : x(p.x), y(p.y) {}
	fpt(const pt& a, const pt& b, double interp) {
		x = interp * b.x + (1 - interp) * a.x;
		y = interp * b.y + (1 - interp) * a.y;
	}
	fpt(const fpt& a, const fpt& b, double interp) {
		x = interp * b.x + (1 - interp) * a.x;
		y = interp * b.y + (1 - interp) * a.y;
	}
};

struct frame {
	int n;
	vector<fpt> locations;
	frame(const frame& prev, const vector<int>& move) : n(prev.n) {
		// bool diff = false;
		for (int i = 0; i < n; ++i) {
			locations.emplace_back(prev.locations[i].x + dx[move[i]],
														 prev.locations[i].y + dy[move[i]]);
			// if (move[i] || move[i])
			//	diff = true;
		}
		// if (!diff)
		//	cout << "nothing moved!" << endl;
	}
	frame(const instance& i) {
		n = i.n;
		for (auto& p : i.start) {
			locations.emplace_back(p);
		}
	}
	frame(const frame& a, const frame& b, double interp) : n(a.n) {
		assert(interp >= 0 && interp <= 1);
		assert(a.n == b.n);
		for (int i = 0; i < n; ++i)
			locations.emplace_back(a.locations[i], b.locations[i], interp);
	}
	int xmin() {
		int ret = 0;
		for (auto& p : locations) {
			ret = min(ret, int(p.x));
		}
		return ret;
	}
	int ymin() {
		int ret = 0;
		for (auto& p : locations) {
			ret = min(ret, int(p.y));
		}
		return ret;
	}
	int xmax() {
		int ret = 0;
		for (auto& p : locations) {
			ret = max(ret, int(p.x));
		}
		return ret;
	}
	int ymax() {
		int ret = 0;
		for (auto& p : locations) {
			ret = max(ret, int(p.y));
		}
		return ret;
	}
	void renormalize(int origin_x, int origin_y) {
		for (auto& p : locations) {
			p.x -= origin_x;
			p.y -= origin_y;
		}
	}
};
