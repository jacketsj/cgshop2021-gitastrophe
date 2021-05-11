#include <iostream>
#include <string>

#include "Instance.h"

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

	if (!custom) {
		if (makespan)
			cout << "reading makespan output (use -d parameter to read distance, -c "
							"to read custom file location)"
					 << endl;
		else
			cout << "reading distance output" << endl;
	}

	string name;
	while (cin >> name) {
		instance i;
		if (custom)
			i.read_custom(name);
		else {
			i.read(name);
			i.read_out(makespan);
		}

		bool ver = verify(i);

		i.write();
		cout << "Instance " << i.name << " is valid? " << (ver ? "true" : "false")
				 << endl;
		if (ver) {
			cout << "Instance " << i.name << " makespan: " << score(i, true) << endl;
			cout << "Instance " << i.name << " distance: " << score(i, false) << endl;
		}
	}
}
