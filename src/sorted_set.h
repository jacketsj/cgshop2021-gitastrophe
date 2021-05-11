#pragma once

#include <bits/stdc++.h>
using namespace std;

template <class T> struct sorted_set : vector<T> {
	typename vector<T>::iterator begin() { return vector<T>::begin(); }
	typename vector<T>::iterator end() { return vector<T>::end(); }
	typename vector<T>::const_iterator begin() const {
		return vector<T>::begin();
	}
	typename vector<T>::const_iterator end() const { return vector<T>::end(); }
	typename vector<T>::const_iterator lower_bound(const T& x) const {
		return ::lower_bound(begin(), end(), x);
	}
	void insert(const T& x) {
		auto it = lower_bound(x);
		if (it == end() || it->t != x.t) {
			vector<T>::insert(it, x);
		}
	}
	void erase(const T& x) {
		auto it = lower_bound(x);
		if (it != end() && it->t == x.t) {
			vector<T>::erase(it);
		}
	}
	bool empty() const { return vector<T>::empty(); }
};
