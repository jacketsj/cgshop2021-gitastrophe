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
CImgDisplay* disp = NULL;
CImg<double>* img = NULL;
double scale_x, scale_y;
void init_window(int x, int y, string name) {
	disp = new CImgDisplay(x, y, name.c_str(), 0);
	img = new CImg<double>(x, y, 1, 3, 0);
	scale_x = 1;
	scale_y = 1;
}
bool should_close() { return disp->is_closed() || disp->is_keyESC(); }
void get_window_size(int& x0, int& y0) {
	x0 = disp->width();
	y0 = disp->height();
}
void draw_cell(col::col c, double x, double y, double scale, double mini = 1) {
	double c_arr[] = {c.r * 255, c.g * 255, c.b * 255};
	double cent_x = x * scale + scale / 2, cent_y = y * scale + scale / 2;
	vector<double> coords = {
			scale_x * (cent_x - scale * mini / 2),
			disp->height() - scale_y * (cent_y - scale * mini / 2),
			scale_x * (cent_x + scale * mini / 2),
			disp->height() - scale_y * (cent_y + scale * mini / 2)};
	img->draw_rectangle(coords[0], coords[1], coords[2], coords[3], c_arr);
	/*
	img->draw_rectangle(scale_x * x * scale, disp->height() - scale_y * y * scale,
											scale_x * (x + 1) * scale,
											disp->height() - scale_y * (y + 1) * scale, c_arr);
	*/
	double white[] = {255, 255, 255};
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
	double cent_x = x * scale + scale / 2,
				 cent_y = disp->height() - (y * scale + scale / 2);
	double cent_xd = xd * scale + scale / 2,
				 cent_yd = disp->height() - (yd * scale + scale / 2);
	img->draw_line(cent_x, cent_y, cent_xd, cent_yd, c_arr);
}
void set_scale(double sx, double sy) {
	scale_x = sx;
	scale_y = sy;
}
void close_stuff() {}
void render_stuff() {
	img->display(*disp);
	img->fill(0);
}

/* Old SIGIL implementation (mostly equivalent, better window resizing):
#include <sl.h>
void init_window(int x, int y, string name) {
	slWindow(x, y, name.c_str(), false);
}
bool should_close() { return slShouldClose() || slGetKey(SL_KEY_ESCAPE); }
void get_window_size(int& x0, int& y0) { slGetWindowSize(&x0, &y0); }
void set_color(const col::col& c) { slSetForeColor(c.r, c.g, c.b, c.a); }
void draw_cell(col::col c, double x, double y, double scale) {
	set_color(c);
	slRectangleFill(x * scale + scale / 2, y * scale + scale / 2, scale, scale);
	set_color(col::white);
	slRectangleOutline(x * scale + scale / 2, y * scale + scale / 2, scale,
										 scale);
}
void set_scale(double sx, double sy) {
	slIdentity();
	slScale(sx, sy);
}
bool get_space_key() { return slGetKey(' '); }
bool get_left_key() { return slGetKey(SL_KEY_LEFT); }
// close the window and shut down SIGIL
void close_stuff() { slClose(); }
void render_stuff() { slRender(); }
*/

struct dimensions {
	// internal dimensions
	int x, y;
	double scale_x, scale_y;
	dimensions(int x, int y) : x(x), y(y), scale_x(1), scale_y(1) {}
	void update() {
		int x0, y0;
		get_window_size(x0, y0);
		scale_x = double(x0) / double(x);
		scale_y = double(y0) / double(y);
	}
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

enum play_state { PLAYING, PAUSED, BACKWARDS };

// const clock_t CLOCKS_PER_FRAME = CLOCKS_PER_SEC / speed;
struct player {
	play_state state;
	int max_frame;
	// clock_t cur_start_time;
	// int cur_start_frame;
	int finish_frame;
	int speed = 1;
	const int min_speed = 1;
	const int max_speed = 64;
	double cur_frame;
	clock_t prev_time;
	play_state last_state = PLAYING;
	player(int max_frame) : state(PAUSED), max_frame(max_frame) {
		// cur_start_frame = 0;
		// cur_start_time = 0;
		cur_frame = 0;
		finish_frame = max_frame;
		prev_time = clock();
	}
	enum key { SPACE, LEFT, RIGHT, UP, DOWN, KEY_L, NONE };
	map<key, bool> was_down;
	bool get_key(key k, unsigned key) {
		bool is = disp->is_key(key);
		bool ret = !was_down[k] && is;
		was_down[k] = is;
		return ret;
	}
	key get_pressed_key() {
		key ret = NONE;
		vector<unsigned> calls = {cimg::keySPACE,			 cimg::keyARROWLEFT,
															cimg::keyARROWRIGHT, cimg::keyARROWUP,
															cimg::keyARROWDOWN,	cimg::keyL};
		// vector<function<bool()>> calls = {
		//		disp->is_keySPACE, disp->is_keyARROWLEFT, disp->is_keyARROWRIGHT,
		//		disp->is_keyARROWUP, disp->is_keyARROWDOWN};
		int i = 0;
		for (int k = SPACE; k != NONE; k++) {
			if (get_key(static_cast<key>(k), calls[i])) {
				ret = static_cast<key>(k);
			}
			++i;
		}
		return ret;
	}

	/*
	void back_press() {
		if (state == PAUSED) {
			cout << "playing (backwards)!" << endl;
			state = BACKWARDS;
			cur_start_time = clock();
			finish_frame = 0;
		} else if (state == BACKWARDS) {
			finish_frame = floor(cur_start_frame -
													 double(clock() - cur_start_time) / CLOCKS_PER_FRAME);
		}
	}
	void forward_press() {
		if (state == PAUSED) {
			cout << "playing!" << endl;
			state = PLAYING;
			cur_start_time = clock();
			finish_frame = max_frame;
		} else if (state == PLAYING) {
			finish_frame = ceil(cur_start_frame +
													double(clock() - cur_start_time) / CLOCKS_PER_FRAME);
		}
	}
	*/
	void stop_if_finished() {
		if (state == PLAYING && cur_frame > finish_frame) {
			cout << "pausing!" << endl;
			state = PAUSED;
			cur_frame = finish_frame;
		} else if (state == BACKWARDS && cur_frame < finish_frame) {
			cout << "pausing (backwards)!" << endl;
			state = PAUSED;
			cur_frame = finish_frame;
		}
	}
	double get_frame() {
		update();
		return cur_frame;
	}
	double next_frame() { return min(double(max_frame), ceil(cur_frame)); }
	double prev_frame() { return max(double(0), floor(cur_frame)); }
	void update() {
		clock_t cur_time = clock();
		double delta_time = double(cur_time - prev_time) / double(CLOCKS_PER_SEC);
		prev_time = cur_time;

		switch (get_pressed_key()) {
		case NONE:
			break;
		case KEY_L:
			GOALS = !GOALS;
			break;
		case SPACE:
			// pause/play
			if (state == PAUSED) {
				state = last_state;
				if (state == PLAYING)
					finish_frame = max_frame;
				else
					finish_frame = 0;
			} else if (state == PLAYING) {
				finish_frame = next_frame();
			} else if (state == BACKWARDS) {
				finish_frame = prev_frame();
			}
			break;
		case LEFT:
			// start/stop backwards play
			if (state == PAUSED) {
				state = BACKWARDS;
				last_state = state;
				finish_frame = 0;
			} else if (state == PLAYING) {
				state = BACKWARDS;
				last_state = state;
				finish_frame = 0;
			} else if (state == BACKWARDS) {
				finish_frame = prev_frame();
			}
			break;
		case RIGHT:
			// start/stop forwards play
			if (state == PAUSED) {
				state = PLAYING;
				last_state = state;
				finish_frame = max_frame;
			} else if (state == BACKWARDS) {
				state = PLAYING;
				last_state = state;
				finish_frame = max_frame;
			} else if (state == PLAYING) {
				finish_frame = next_frame();
			}
			break;
		case UP:
			// increase speed up to maximum
			if (speed < max_speed)
				speed *= 2;
			break;
		case DOWN:
			// decrease speed down to minimum
			if (speed > min_speed)
				speed /= 2;
			break;
		}
		// now perform frame update and state transitions (e.g. do we pause here?)
		if (state == PAUSED) {
			return;
		} else if (state == PLAYING) {
			cur_frame += delta_time * speed;
			stop_if_finished();
			return;
		} else {
			cur_frame -= delta_time * speed;
			stop_if_finished();
			return;
		}
	}
};

void render(const framedata& f) {
	// dimensions of window
	dimensions dim(1280, 720);
	// figure out dimensions based on f and some scaling factor or something
	double cell_scale = 1.0;
	int W = f.xmax + 1;
	int H = f.ymax + 1;
	cell_scale = min(double(1280) / double(W), double(720) / double(H));

	cout << "cell scale=" << cell_scale << endl;

	// set up our window and a few resources we need
	init_window(dim.x, dim.y, "Grid-MAPF Visualizer");

	vector<col::col> cols;
	for (int i = 0; i < f.i.n; ++i) {
		cols.push_back(col::random());
	}

	// bool prev_pressed_space = false;
	// bool prev_pressed_left = false;
	player play(f.time - 1);
	// bool playing = false;
	// bool continue_playing_after_step = false;
	// int stop_playing_after_step;

	int num_recent =
			4; // set to 0 for proper behaviour without weird trailing boxes

	while (!should_close()) {
		// update the window dimensions
		dim.update();

		// update the scaling accordingly
		dim.render_transform();

		// figure out the current frame to draw
		/*
		if (!prev_pressed_space && get_space_key()) {
			prev_pressed_space = true;
			play.forward_press();
		} else if (prev_pressed_space && !get_space_key()) {
			prev_pressed_space = false;
		}
		if (!prev_pressed_left && get_left_key()) {
			prev_pressed_left = true;
			play.back_press();
		} else if (prev_pressed_left && !get_left_key()) {
			prev_pressed_left = false;
		}
		*/
		double cur_frame = play.get_frame();

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
	close_stuff();
};
