#include <iostream>
#include <string>

#include "Instance.h"

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
	while (cin >> name) {
      run(name);
	}
}
