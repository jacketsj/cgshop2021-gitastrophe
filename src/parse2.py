from os import listdir
from os.path import isfile, join

path = "../input/"
files = [f for f in listdir(path) if isfile(join(path, f))]
names = [i.strip() for i in open("todo.txt", "r").readlines()]
print(names)
for n in names:
    done = False
    for f in files:
        if f.startswith(n):
            print(f)
            done = True
            break
    if not done:
        print(n, "is not done")
