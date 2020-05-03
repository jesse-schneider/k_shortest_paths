#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include<fstream>
#include <ctime>
#include<unordered_map>
#include<queue>
#include<limits>
#include<bits/stdc++.h>

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
        void addEdge(const string &dest, const double &cost);
        void printConnections();
        double getPriority();
        void setPriority(const double &c);
        void setName(const string &name);
        string getName();
        vector<string> getConnections();
        vector<double> getConnectCosts();
        void setPrevious(string prev);
        string & getPrevious();
        double getPreviousCost(string destination);

        Node(): priority{numeric_limits<double>::infinity()} {}

    private:
        double priority;
        string name;
        string previous = "";
        vector<string> connections;
        vector<double> connectCosts;
};

void Node::addEdge(const string &dest, const double &cost) {
    connections.push_back(dest);
    connectCosts.push_back(cost);
}

void Node::printConnections() {
    cout << "Node: " << name << " : ";
    for(int i = 0; i < connections.size(); i++) {
        cout << connections[i] << " " << connectCosts[i] << " ";
    }
    cout << endl;
}

double Node::getPriority() {
    return priority;
}

void Node::setPriority(const double &c) {
    priority = c;
}

void Node::setName(const string &n) {
    name = n;
}

string Node::getName() {
    return name;
}

vector<string> Node::getConnections() {
    return connections;
}

vector<double> Node::getConnectCosts() {
    return connectCosts;
}
void Node::setPrevious(string prev) {
    previous = prev;
}

string & Node::getPrevious() {
    return previous;
}
double Node::getPreviousCost(string destination) {

    for(int i = 0; i < connections.size(); i++) {
        if(connections[i] == destination) {
            return connectCosts[i];
        }
    }
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

    //for each input node, create and store
    unordered_map<string, Node> nodes;
    unordered_map<string, double> costToArrive;
    for(int i = 0; i < e; i++) {
        
        string s, d; double cost;
        input >> s >> d >> cost;

        //create iterator to find existing node
        auto iter = nodes.find(s);

        //if node does not exist yet in map, create it, else already exists
        if(iter == nodes.end()) {
            Node n;
            n.addEdge(d, cost);
            n.setName(s);
            costToArrive[s] = numeric_limits<double>::infinity(); 
            nodes[s] = n;
        } else {
            Node n(iter->second);
            n.addEdge(d, cost);
            nodes[s] = n;
        }
    }

    //read in the source, destination and k (how many paths)
    string source, dest; int k;
    input >> source >> dest >> k;
    input.close();

    //if destination node not included in edge list, add it to the map
    if(nodes.find(dest) == nodes.end()) {
        Node d;
        d.setName(dest);
        costToArrive[dest] = numeric_limits<double>::infinity();
        nodes[dest] = d;
    }

    /*
        perform dijkstra's algorithm to find the cheapest path
        using a priority queue and processing nodes sequentially (nodes added to queue as explored)
    */

   //priority queue comparator function for sorting priority upon insert
   struct CostCompare {
       bool operator()(Node & nodeA, Node & nodeB) {
           return nodeA.getPriority() > nodeB.getPriority();
       }
   };

    priority_queue<Node, vector<Node>, CostCompare> queue;
    
    costToArrive[source] = 0;
    costToArrive[dest] = numeric_limits<double>::infinity();
    queue.push(nodes[source]);
    vector<Node> visited;

    //iterate over queue
    while(!queue.empty()) {
        Node n = queue.top();
        visited.push_back(n);
        queue.pop();

        double currentDist;
        currentDist = costToArrive[n.getName()];
       
       if(costToArrive.find(n.getName()) == costToArrive.end())
           break;

        vector<string> connects = n.getConnections();
        vector<double> connectCs = n.getConnectCosts();

        for(int i = 0; i < connects.size(); i++) {
            //update costs for all connects to node
            string neighbour = connects[i];
            double neighCost = connectCs[i];

            //check if we need to change distances
            if(costToArrive[neighbour] > currentDist + neighCost) {
                // cout << "neigbour: " << neighbour << " cost: " << currentDist + neighCost << " currentDist: " << currentDist << " neighCost: " << neighCost << endl;
                // cout << "costToArrive[neighbour]: " << costToArrive[neighbour] << endl;
                costToArrive[neighbour] = currentDist + neighCost;
                if(nodes.find(neighbour) != nodes.end()) {
                    nodes[neighbour].setPriority(costToArrive[neighbour]);
                    nodes[neighbour].setPrevious(n.getName());
                    queue.push(nodes[neighbour]);
                } 
            }
        }
    }

    //get the final path, and store it
    string prev = dest;
    vector<pair<string, double>> path;

    while(prev != "") {
        if(prev == source)
            break;
        auto it = nodes.find(prev);
        string & next = it->second.getPrevious();
        double c = nodes[next].getPreviousCost(prev);
        path.push_back(make_pair(string(next), c));
        prev = next;
    }

    reverse(path.begin(), path.end());
    path.push_back(make_pair(dest, 0));

    for(auto &i: path) {
        cout << i.first << " cost: " << i.second  << endl;
    }    

    //shortest path has been found, now let's find the other k-1 paths
    cout << "cost to final: " << costToArrive[dest] << endl;

    /*
        to find k-1 paths, we are going to find the smallest cost in the current path, make it unusable, and find the next path, then store it
        continue this until all k-1 paths are found
    */
   vector<vector<pair<string, double>>> allPaths;
   allPaths.push_back(path);

    end_t = clock();
    cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

    printf("Time taken: %f seconds\n", cpu_time_used);
    cout << endl; 
    return 0;
}





// //  print out list of nodes
    // auto iterator = nodes.begin();
    // while (iterator != nodes.end()) {
    //     cout << iterator->first << ":: ";
    //     iterator->second.printConnections();
    //     iterator++;
    // }