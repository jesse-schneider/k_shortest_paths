#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include<fstream>
#include <ctime>
#include<unordered_map>
#include<queue>
#include<limits>

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
        void setPrev(Node *);
        Node & getPrev();
        void showPath();

        Node(): priority{numeric_limits<double>::infinity()} {}

    private:
        double priority;
        string name;
        Node *prev = NULL;
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
void Node::setPrev(Node *previous) {
    prev = previous;
}

Node & Node::getPrev() {
    return *prev;
}

void Node::showPath() {
    
    cout << name << endl;

    if(prev == NULL) {
        cout << "NULL" << endl;
    }

    // while(prev != NULL) {
    for(int i = 0; i < 2; i++) {
        cout << prev->getPriority() << endl;
        // string t = prev->getName();
        // cout << t;
        Node *next = &prev->getPrev();
        prev = next;
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
    string path = "";
    vector<Node> visited;

    //iterate over queue
    while(!queue.empty()) {
        Node n = queue.top();
        visited.push_back(n);
        queue.pop();
        double currentDist;

        currentDist = costToArrive[n.getName()];
       
       if(costToArrive.find(n.getName()) == costToArrive.end()) {
           break;
       }

        vector<string> connects = n.getConnections();
        vector<double> connectCs = n.getConnectCosts();

        for(int i = 0; i < connects.size(); i++) {
            //update costs for all connects to node
            string neighbour = connects[i];
            double neighCost = connectCs[i];
            //check if we need to change distances
            if(costToArrive[neighbour] > currentDist + neighCost) {
                
                path += n.getName();
                cout << "path changed: " << n.getName() << endl;
                // cout << "neigbour: " << neighbour << " cost: " << currentDist + neighCost << " currentDist: " << currentDist << " neighCost: " << neighCost << endl;
                // cout << "costToArrive[neighbour]: " << costToArrive[neighbour] << endl;
                costToArrive[neighbour] = currentDist + neighCost;
                if(nodes.find(neighbour) != nodes.end()) {
                    nodes[neighbour].setPrev(&n);
                    nodes[neighbour].setPriority(costToArrive[neighbour]);
                    queue.push(nodes[neighbour]);
                } 
            }
        }
    }
    for(int i = 0; i < visited.size(); i++) {
        cout << visited[i].getName() << endl;
    }

    //  print out list of nodes
    auto iterator = nodes.begin();
    while (iterator != nodes.end()) {
        cout << iterator->first << ":: ";
        iterator->second.printConnections();
        iterator++;
    }

    visited.back().showPath();
    

    //shortest path has been found, now let's find the other k-1 paths
    cout << "cost to final: " << costToArrive[dest] << endl;
    cout << "path: " << path << endl;

    /*
        to find k-1 paths, we are going to find the smallest cost in the current path, make it unusable, and find the next path, then store it
        continue this until all k-1 paths are found
    */

    end_t = clock();
    cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

    printf("Time taken: %f seconds\n", cpu_time_used);
    cout << endl; 
    input.close();
    return 0;
}




