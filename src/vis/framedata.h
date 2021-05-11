#pragma once

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "../Instance.h"
#include "frame.h"

using std::string;
using std::vector;

struct framedata {
	instance i;
	int time;
	vector<frame> frames;
	int xmin, ymin, xmax, ymax;
	vector<pt> obstacles;
	framedata(const instance& i) : i(i) {
		obstacles = i.obstacle;
		xmin = 0;
		ymin = 0;
		xmax = 0;
		ymax = 0;
		for (auto& p : obstacles) {
			xmin = min(xmin, p.x);
			ymin = min(ymin, p.y);
			xmax = max(xmax, p.x);
			ymax = max(ymax, p.y);
		}
		simulate_frames();
		time = frames.size();
	}
	void simulate_frames() {
		frames.emplace_back(i);
		for (auto& move : i.moves) {
			// for (int i : move)
			//	cout << i << ' ';
			// cout << '\n';
			frames.emplace_back(frames.back(), move);
		}
		for (auto& f : frames) {
			xmin = min(xmin, f.xmin());
			ymin = min(ymin, f.ymin());
			xmax = max(xmax, f.xmax());
			ymax = max(ymax, f.ymax());
		}
		// now renormalize everything
		for (auto& f : frames) {
			f.renormalize(xmin, ymin);
		}
		for (auto& p : obstacles) {
			p.x -= xmin;
			p.y -= ymin;
		}
		xmax -= xmin;
		ymax -= ymin;
	}
	frame get_frame(double f) const {
		if (int(f) == time) {
			f -= 1e-8; // fix some epsilon errors
		}
		int base = int(f);
		assert(base < time);
		assert(base >= 0);
		int next = min(base + 1, time - 1);
		return frame(frames[base], frames[next], f - base);
	}
};
