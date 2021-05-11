#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "../Instance.h"
#include "col.h"
#include "frame.h"
#include "framedata.h"

using std::function;
using std::string;
using std::vector;

#include "CImg.h"
using namespace cimg_library;
// CImgDisplay* disp = NULL;
CImg<unsigned char>* img = NULL;
double scale_x, scale_y;
int W, H;
void init_image(int x, int y) {
	// disp = new CImgDisplay(x, y, name.c_str(), 0);
	W = x;
	H = y;
	img = new CImg<unsigned char>(x, y, 1, 3, 0);
	scale_x = 1;
	scale_y = 1;
}
// bool should_close() { return disp->is_closed() || disp->is_keyESC(); }
void draw_cell(col::col c, double x, double y, double scale, double mini = 1) {
	unsigned char c_arr[] = {(unsigned char)floor(c.r * 255),
													 (unsigned char)floor(c.g * 255),
													 (unsigned char)floor(c.b * 255)};
	double cent_x = x * scale + scale / 2, cent_y = y * scale + scale / 2;
	vector<double> coords = {scale_x * (cent_x - scale * mini / 2),
													 H - scale_y * (cent_y - scale * mini / 2),
													 scale_x * (cent_x + scale * mini / 2),
													 H - scale_y * (cent_y + scale * mini / 2)};
	img->draw_rectangle(coords[0], coords[1], coords[2], coords[3], c_arr);
	/*
	img->draw_rectangle(scale_x * x * scale, disp->height() - scale_y * y * scale,
																																									scale_x * (x + 1) * scale,
																																									disp->height() - scale_y * (y + 1) * scale, c_arr);
	*/
	unsigned char white[] = {255, 255, 255};
	if (mini == 1)
		img->draw_rectangle(coords[0], coords[1], coords[2], coords[3], white, 1,
												~0U);
	/*
	img->draw_rectangle(scale_x * x * scale, disp->height() - scale_y * y * scale,
																																									scale_x * (x + 1) * scale,
																																									disp->height() - scale_y * (y + 1) * scale, white, 1,
																																									~0U);
	*/
}
bool GOALS = true;
void draw_goal_path(col::col c, double x, double y, double xd, double yd,
										double scale) {
	double c_arr[] = {c.r * 255, c.g * 255, c.b * 255};
	double cent_x = x * scale + scale / 2, cent_y = H - (y * scale + scale / 2);
	double cent_xd = xd * scale + scale / 2,
				 cent_yd = H - (yd * scale + scale / 2);
	img->draw_line(cent_x, cent_y, cent_xd, cent_yd, c_arr);
}
void set_scale(double sx, double sy) {
	scale_x = sx;
	scale_y = sy;
}
void close_stuff() {}
void render_stuff() {
	char* s = reinterpret_cast<char*>(img->data() + (W * H));
	cout.write(s, W * H);
	s = reinterpret_cast<char*>(img->data() + 2 * (W * H));
	cout.write(s, W * H);
	s = reinterpret_cast<char*>(img->data());
	cout.write(s, W * H);

	img->fill(0);
}

struct dimensions {
	// internal dimensions
	int x, y;
	double scale_x, scale_y;
	dimensions(int x, int y) : x(x), y(y), scale_x(1), scale_y(1) {}
	void render_transform() { set_scale(scale_x, scale_y); }
	void render_notransform() { set_scale(1, 1); }
	static int round(double d) { return int(d + 0.5 + 1e-7); }
};

void draw_obstacles(const vector<pt>& obstacles, double cell_scale) {
	for (auto& p : obstacles)
		draw_cell(col::gray, p.x, p.y, cell_scale);
}

void draw_frame(const vector<col::col>& cols, frame f, double cell_scale,
								const instance& ins, int xmin, int ymin, double miniture = 1) {
	for (int i = 0; i < f.n; ++i)
		draw_cell(cols[i], f.locations[i].x, f.locations[i].y, cell_scale,
							miniture);

	if (miniture == 1 && GOALS) {
		// draw paths to endpoints
		for (int i = 0; i < f.n; ++i)
			draw_goal_path(cols[i], f.locations[i].x, f.locations[i].y,
										 ins.target[i].x - xmin, ins.target[i].y - ymin,
										 cell_scale);
	}
}

void render_gif(const framedata& f, int frames_per_move, bool goal_lines) {
	GOALS = goal_lines;

	// dimensions of window
	dimensions dim(1280, 720);
	// figure out dimensions based on f and some scaling factor or something
	double cell_scale = 1.0;
	int W = f.xmax + 1;
	int H = f.ymax + 1;
	cell_scale = min(double(1280) / double(W), double(720) / double(H));

	// set up our window and a few resources we need
	init_image(dim.x, dim.y);

	vector<col::col> cols;
	for (int i = 0; i < f.i.n; ++i) {
		cols.push_back(col::random());
	}

	// set to 0 for proper behaviour without weird trailing boxes
	int num_recent = 4;

	for (int t = 0; t < f.time; ++t) {
		for (int i = 0; i < frames_per_move; ++i) {
			// update the scaling accordingly (probably not necessary)
			dim.render_transform();

			double cur_frame = double(t) + double(i) / frames_per_move;

			// draw all obstacle blocks
			draw_obstacles(f.obstacles, cell_scale);

			// draw each robot ('frame')
			for (double lag = cur_frame - num_recent; lag <= cur_frame; lag += 0.04) {
				draw_frame(cols, f.get_frame(max(lag, double(0))), cell_scale, f.i,
									 f.xmin, f.ymin, double(1) / (cur_frame - lag + 1));
			}
			draw_frame(cols, f.get_frame(max(cur_frame, double(0))), cell_scale, f.i,
								 f.xmin, f.ymin, 1);

			// draw everything
			render_stuff();
		}
	}
};
