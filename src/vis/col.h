#pragma once

namespace col {
struct col {
	double r, g, b, a;
	col() : r(0), g(0), b(0), a(0) {}
	col(double r, double g, double b, double a) : r(r), g(g), b(b), a(a) {}
};

col white(1, 1, 1, 1);
col black(0, 0, 0, 1);
col gray(0.5f, 0.5f, 0.5f, 1);
col red(0.8, 0.15, 0.1, 1);
col green(0.1, 0.9, 0.2, 1);
col blue(0.1, 0.1, 0.85, 1);

col random() {
	return col(double(rand()) / RAND_MAX, double(rand()) / RAND_MAX,
						 double(rand()) / RAND_MAX, 1);
}
}; // namespace col
