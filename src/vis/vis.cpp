#include <iostream>
#include <string>

#include "../Instance.h"
#include "window.h"

using std::cin;
using std::string;

int main(int argc, char* argv[]) {
	bool makespan = true;
	bool custom = false;
	if (argc > 1) {
		if (string(argv[1]) == string("-d"))
			makespan = false;
		else if (string(argv[1]) == string("-c")) {
			custom = true;
		}
	}

	string name;
	cin >> name;
	instance i;
	if (custom)
		i.read_custom(name);
	else
		i.read(name);

	if (!custom) {
		if (makespan)
			cout << "reading makespan output (use -d parameter to read distance, -c "
							"to read custom file location)"
					 << endl;
		else
			cout << "reading distance output" << endl;
		i.read_out(makespan);
	}

	framedata fd(i);
	render(fd);
}
