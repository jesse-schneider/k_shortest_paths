#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include<fstream>
#include <ctime>
#include<unordered_map>

using namespace std;

/* Node class to store each node
    each class stores:
        - list of nodes this node connects to
        - each cost of the connection
*/

class Node {

    public:
        void findConnections();
        void isConnection();
        void addEdge(string dest, double cost);
        void printConnections();

    private:
        vector<string> connections;
        vector<double> connectCosts;
};

void Node::addEdge(string dest, double cost) {
    connections.push_back(dest);
    connectCosts.push_back(cost);
}

void Node::printConnections() {
    for(int i = 0; i < connections.size(); i++) {
        cout << connections[i] << " " << connectCosts[i] << " ";
    }
    cout << endl;

 }
int main(int argc, char ** argv) {

    string filename = "input.txt";
    ifstream input;

    clock_t start_t, end_t;
    double cpu_time_used;
    start_t = clock();

    /*
        6 nodes, 9 edges between nodes, directed graph
        find 3 shortest paths from C -> H: last input line Source, Destination, k

    */

    input.open(filename);
    int n, e;
    input >> n >> e;

    //for each node, initalise a node
    unordered_map<string, Node> nodes;
    for(int i = 0; i < e; i++) {
        //input : G D 6
        string s, d; double cost;
        input >> s >> d >> cost;

        auto iter = nodes.find(s);

        //node does not exist yet in map
        if(iter == nodes.end()) {
            Node n;
            n.addEdge(d, cost);
            nodes[s] = n;
        } else {
            Node n(iter->second);
            n.addEdge(d, cost);
            nodes[s] = n;
        }
    }
    string source, dest; int k;

    input >> source >> dest >> k;

    //print out list of nodes
    auto iterator = nodes.begin();
    while (iterator != nodes.end()) {

        cout << iterator->first << ":: ";
        iterator->second.printConnections();
        iterator++;
    }
    
        
        

        end_t = clock();
        cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

        printf("Time taken: %f seconds\n", cpu_time_used);
        cout << endl; 
    input.close();
   return 0;
}
