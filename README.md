This repository is an attempt to use the approach described in ["A Statistical Measure for Map Consistency in SLAM"](http://www2.informatik.uni-freiburg.de/~mazuran/consist.html) by Mazuran et al. They come up with a measure to quantify the inconsistency between two scans. We want to use this measure to find the level of misalignment between a real scan and a virtaul scan that should ideally be similar to the real scan. This measure, known as the inconsistency distance in the paper, is calculated based on how many points of each scan are inside a polygon drawn by the points of the other scan.

`cmake_modules` contains modules to find G2O and Eigen packages. These are directly taken from the G2O repositroy.

`data` contains scans taken form a ROSbag and simply copy-pasted into a text file. More details are written in the README inside the folder.

[`libs/map-consistency`](https://github.com/shrinivas96/map-consistency) contains the repository of the authors' code as a git submodule. The authors provide [a source code](http://www2.informatik.uni-freiburg.de/~mazuran/consist.html#:~:text=available%20for%20download.-,Download,-Consist%20v0.1%20source) on their webste. Unfortunately though, the code seems to have been written in 2014 and does not compile as is. This repository is an attempt to fix that code in the most bare minimum way possible, enough for me to use just the classes that I need from them (`map-consistency/src/consist/visibility.h`).

`src/main.cpp` is the main file that reads the scans file and converts the ranges into `g2o::RawLaser` data type. Then the scan and the pose are passed on to a `consist::Visibility` object. Ideally `consist::Visibility::polyOcclusions()` should return the array of distances of each point from a given scan that is inside the other scan. Doing this for both scans and adding up the distances should give us the inconsistency distance between the two scans. At the moment `polyOcclusions()` returns empty arrays. I do not know how to fix this though.

Note: `src/main.cpp` has a function `scan_as_vector()` that returns the two lines of scans in `data/` folders as vectors.