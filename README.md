# CG:SHOP 2021 - gitastrophe

This repository contains the source code of team gitastrophe in 
[CG:SHOP 2021](https://cgshop.ibr.cs.tu-bs.de/competition/cg-shop-2021/). 

For more information, see our paper in the SoCG proceedings or on arxiv.

<a id="1">[1]</a> 
Paul Liu, Jack Spalding-Jamieson, Brandon Zhang, Da-Wei Zheng. 
Coordinated Motion Planning Through Randomized k-Opt (CG Challenge). SoCG 2021: 64:1-64:8.
<a href="https://drops.dagstuhl.de/opus/volltexte/2021/13863">https://drops.dagstuhl.de/opus/volltexte/2021/13863</a>

<a id="2">[2]</a> 
Paul Liu, Jack Spalding-Jamieson, Brandon Zhang, Da-Wei Zheng. 
Coordinated Motion Planning Through Randomized k-Opt. CoRR abs/2103.15062 (2021).
<a href="https://arxiv.org/abs/2103.15062">https://arxiv.org/abs/2103.15062</a>

## Compiling/building

We rely on `cmake` to build and run our code. If you don't have `cmake` you can get it with:
```
sudo apt-get install cmake
```

Enter the `build` directory and run `build.sh`.
```
cd build; ./build.sh
```

Visualizer build might fail due to dependencies.
If so, you may need to run:
```
sudo apt-get install libglfw3-dev libgles2-mesa-dev
```

If you don't need to use the visualizer, you can just run `make optimize`.

## Using the optimizer 

### How to run

Navigate to the `build` directory if you aren't already there.

To see a full details of how to run the code, you can see the usage statment by running:
```
./optimize --help
```

To run on a json file and to output a json file under the CG:SHOP format called `output.json`, you can run a command like:
```
./optimize ../raw_input/small_019_20x20_90_329.instance.json -o output.json
```

If you wish to run on a file used in the CG:SHOP 2021 competition for distance, you can simply run:
```
./optimize small_019_20x20_90_329
```

If you wish to run optimizer for makespan, and let it run for 10 minutes (600 seconds), you can run:
```
./optimize small_019_20x20_90_329 600 -m
```

If you want to manually set k and R 
(see our paper[[1]](#1) for more information on these parameters)
you can do so with:
```
./optimize small_019_20x20_90_329 -k 5 -R 10
```

Results can be found in the `output/distance` (or `output/makespan`) folder and can be visualized with our visualizer.

### Optimizer details

Whenever `optimize` is run, and successfully finds a solution,
it will attempt to write that solution.
Before writing, a solution is checked for validity,
and also checked that it is an improvement over the current minimum distance and makespan solutions.

Whenever a solver tries to save an output file
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

### GIF output
A related executable to the visualizer will create GIFs of solutions.
`gif_vis` has all the same command-line options as `vis`, but instead of opening a window
and allowing the user to control playback,
it redirects a raw video stream to stdout.  
The script `build/rgif.sh` redirects this raw video stream into ffmpeg and produces an mp4 file.

## Custom Format Descriptions

### Instance format description:

filename is name of instance

**n** is the number of robots

**m** is the number of obstacles

The next **n** lines contains the locations of the robots.

The next **n** lines contains the targets of the robots.

The next **m** lines contains the obstacle coordinates.

### Output format description:

filename is name of instance

**n** is the number of robots

**t** is the time/makespan of the solution

The next **t** lines each contain **n** integers, each with the direction that the **i**th robot moves in encoded as follows:

- 0: Stay still
- 1: North
- 2: East
- 3: South
- 4: West

