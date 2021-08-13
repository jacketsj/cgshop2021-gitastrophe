# CG:SHOP 2021 - Gitastrophe

## Compiling/Building

Dependencies:
```cmake```

Enter the `build` directory and run `build.sh`.
Visualizer build might fail due to dependencies.
If so, you may need to run:
```sudo apt-get install libglfw3-dev libgles2-mesa-dev```

## Instance format description:

filename is name of instance

$n$ is the number of robots

$m$ is the number of obstacles

The next $n$ lines contains the locations of the robots.

The next $n$ lines contains the targets of the robots.

The next $m$ lines contains the obstacle coordinates.

## Output format description:

filename is name of instance

$n$ is the number of robots

$t$ is the time/makespan of the solution

The next $t$ lines each contain $n$ integers, each with the direction that the $i$th robot moves in encoded as follows:

- 0: Stay still
- 1: North
- 2: East
- 3: South
- 4: West

## The Output Directory

Whenever a solver tries to save an output file
without a specified custom location,
it will attempt to save the file to both of `output/distance` and `output/makespan`.
If there is already another output file there, and it is better according to the corresponding metric,
the solver will _not_ overwrite that file.

## Using the visualizer
Like a lot of the other executables, the visualizer can be run in a few different ways.
Make sure to always run everything from inside the build directory.  

### Command-line options
The default:
```
./vis <<< test_sprinkle
```
This will load `../input/test_sprinkle.in` and `../output/makespan/test_sprinkle.out`.  
Alternatively, you can use the path to a `.in` file, but note that this will simply remove the file extension and path internally:
```
./vis <<< ../input/test_sprinkle.in
```
If you want to load `../output/distance/test_sprinkle.out` for the output, you can instead use the `-d` flag with either of the above options:
```
./vis -d <<< test_sprinkle
```
You can also choose the location of a 'custom' `.out` file with the `-c` option:
```
./vis -c <<< ../out/custom_path/debug.out
```

### Controls
The map of controls is as follows:
- SPACE: Play/Pause the video in the current direction of playback
- RIGHT: Play/Pause the video in the forward direction
- LEFT: Play/Pause the video in the backwards direction
- UP: Double the speed of playback (up to 64x)
- DOWN: Halve the speed of playback (down to 1x)
- L: Toggle goal lines

### GIF Output
A related executable to the visualizer will create GIFs of solutions.
`gif_vis` has all the same command-line options as `vis`, but instead of opening a window
and allowing the user to control playback,
it redirects a raw video stream to stdout.  
The script `build/rgif.sh` redirects this raw video stream into ffmpeg and produces an mp4 file.

## The different solvers

### How to run

All solvers can be run similarly to the visualizer,
although most don't have any command-line parameters,
and some of them will only read `.in` files.
Additionally,
they can take more than one file (unlike the visualizer),
and will process the inputs in sequence.
This allows for commands like this:
```
ls ../input/small_* | ./test_run
```

Whenever a solver is run, and successfully finds a solution,
it will attempt to write that solution.
Before writing, a solution is checked for validity,
and also checked that it is an improvement over the current minimum distance and makespan solutions.

### Greedy Optimization with Random

This (non-optimal, non-complete) solver is really bad, but it works ok on some of the smallest `small_free` instances.
It greedily tries to move each of the agents/robots in the direction of their target at each timestep,
and moves in a random direction when it can't. They also occasionally move in a random direction anyway.

### Conflict-Based Search (CBS)

This implements the well-known MAPF algorithm,
with a small amount of additional code to support geometric constraints.
It also uses (mostly) disjoint children, a common slight optimization to CBS.
Unfortunately, it seems to be hopeless on anything but the smallest of the small instances using only the makespan metric.
Using the distance metric it can barely even solve any of the test cases.

### Physics Solver

This solver runs a physics simulation with the following properties:
- All the robots are positive charges.
- All obstacles are stationary positive charges.
- Each robot has a spring attaching it to its target.
  The spring takes a (mostly) valid path to the target, and is also charged.
- On each frame, the path of the spring is re-calculated with a BFS.
  Whenever a robot is encountered in the BFS, there is a small probability that it will pass through the robot.

### Feasibility Solver
This solver runs a two-phase process. For the first phase:
- All the robots are positive charges.
- There is a large positive charge in the centre of the grid.
- For O(sqrt(n)) steps, we run this physics simulation to spread the robots out.
- At the end of this phase, each robot has an unobstructed path to the goal.

In the second phase:
- An A-star is run from each robot to their goal, treating all the other robots' (currently computed) paths as obstacles.
- The order in which the robots go to the goal has two modes: (1) in order of distance from the centre of the grid (not guaranteed to work, but produces smaller makespan and distance). (2) in order of a precomputed BFS tree from the corner of the grid, filling in the leaves of the tree first.

### Greedy Improver

Greedy improver currently goes through the robots in order from 1 to n and finds the optimal path from start to target for that robot. Currently converges quickly to local optimum for small instances.
Should add: 
- find strictly shorter paths
- use randomized order of robots
- use order of robots sorted by distance travelled.



## Sparse Graph Implementation

The sparse graph is currently stored as a set of moves through time for each cell of the grid G.
Let D denote the total distance of an instance, and T the makespan.
Memory usage: O(D + |G|) 
Supports the following operations:
- Check if there is a robot at position p at time t in O(log T)
- Check if moving from p_1 to p_2 is valid at time t in O(log T)
- Delete the tail of the path P of a robot in O(|P| * log T)
- Find the best path P of a robot in O((S + |P|) * log T) where S is the number of nodes searched to find best path

## (Dense) Graph Implementation 

Not sparse implementation of the graph. For grid G and makespan T, takes space O(|G| * T). Supports mostly the same thing as the sparse graph without the log factor except uses arrays of size O(|G| * T) to find best path.
