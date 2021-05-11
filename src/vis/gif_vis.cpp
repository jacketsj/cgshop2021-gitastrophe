#include <iostream>
#include <string>

#include "../Instance.h"
#include "gif.h"

using std::cin;
using std::string;

int main(int argc, char* argv[]) {
	bool makespan = true;
	bool custom = false;
	bool goals = true;
	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			if (string(argv[i]) == string("-d"))
				makespan = false;
			else if (string(argv[i]) == string("-c"))
				custom = true;
			else if (string(argv[i]) == string("-n"))
				goals = false;
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
			cerr << "reading makespan output (use -d parameter to read distance, -c "
							"to read custom file location)"
					 << endl;
		else
			cerr << "reading distance output" << endl;
		i.read_out(makespan);
	}

	framedata fd(i);
	render_gif(fd, 8, goals);
}
