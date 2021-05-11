#pragma once

#include <vector>
using namespace std;

// typedef vector<T> vd; /// include-line
// bool zero(double x) { return fabs(x) < 1e-10; }
template <typename T> bool zero(T x) { return fabs(x) < 1; }
template <typename T>
T matchingpls(const vector<vector<T>>& cost, vector<int>& L, vector<int>& R) {
	int n = cost.size(), mated = 0;
	vector<T> dist(n), u(n), v(n);
	vector<int> dad(n), seen(n);

	/// construct dual feasible solution
	for (int i = 0; i < n; i++) {
		u[i] = cost[i][0];
		for (int j = 1; j < n; j++)
			u[i] = min(u[i], cost[i][j]);
	}
	for (int j = 0; j < n; j++) {
		v[j] = cost[0][j] - u[0];
		for (int i = 1; i < n; i++)
			v[j] = min(v[j], T(cost[i][j] - u[i]));
	}

	/// find primal solution satisfying complementary slackness
	L = R = vector<int>(n, -1);
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++) {
			if (R[j] != -1)
				continue;
			if (zero(cost[i][j] - u[i] - v[j])) {
				L[i] = j;
				R[j] = i;
				mated++;
				break;
			}
		}

	for (; mated < n; mated++) { // until solution is feasible
		int s = 0;
		while (L[s] != -1)
			s++;
		fill(dad.begin(), dad.end(), -1);
		fill(seen.begin(), seen.end(), 0);
		for (int k = 0; k < n; k++)
			dist[k] = cost[s][k] - u[s] - v[k];

		int j = 0;
		for (;;) { /// find closest
			j = -1;
			for (int k = 0; k < n; k++) {
				if (seen[k])
					continue;
				if (j == -1 || dist[k] < dist[j])
					j = k;
			}
			seen[j] = 1;
			int i = R[j];
			if (i == -1)
				break;
			for (int k = 0; k < n; k++) { /// relax neighbors
				if (seen[k])
					continue;
				auto new_dist = dist[j] + cost[i][k] - u[i] - v[k];
				if (dist[k] > new_dist) {
					dist[k] = new_dist;
					dad[k] = j;
				}
			}
		}

		/// update dual variables
		for (int k = 0; k < n; k++) {
			if (k == j || !seen[k])
				continue;
			auto w = dist[k] - dist[j];
			v[k] += w, u[R[k]] -= w;
		}
		u[s] += dist[j];

		/// augment along path
		while (dad[j] >= 0) {
			int d = dad[j];
			R[j] = R[d];
			L[R[j]] = j;
			j = d;
		}
		R[j] = s;
		L[s] = j;
	}
	auto value = vector<T>(1)[0];
	for (int i = 0; i < n; i++)
		value += cost[i][L[i]];
	return value;
}
