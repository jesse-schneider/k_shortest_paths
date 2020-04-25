#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include<fstream>
#include <ctime>

using namespace std;

int main(int argc, char ** argv) {

    string filename = "input.txt";
    ifstream input;

    clock_t start_t, end_t;
    double cpu_time_used;

    /*
        6 nodes, 9 edges between nodes, directed graph
        find 3 shortest paths from C -> H: last input line Source, Destination, k

    */

    input.open(filename);

    for(int i = 0; i < 1; i++) {

        start_t = clock();

        end_t = clock();
        cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

        printf("Time taken: %f seconds\n", cpu_time_used);
        cout << endl; 
    }
    input.close();
   return 0;
}
