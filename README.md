# K Shortest Paths
This is an implementation of a finding shortest paths in a graph using a modified version of Dijkstra'a Algorithm.

This has been built and optimised for UNIX-like environments (built on `Ubuntu 18.04 LTS`).

---

### To run:
* Clone the repository
* Open directory in your preferred terminal
* Run `make build`
* Run `make run` to run the program

The graph in `finalInput.txt` is the current input into the program, feel free to change to `input.txt` or even your own data, by editing `main.cpp`.

---

### Algorithm Steps
1. Find absolute shortest path using dijkstra's algorithm
2. In this path, find smallest edge, mark as `forbidden` - we no longer can use this path
3. Find another path now without the `forbidden` edge
4. Repeat until you have found all k paths

