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
        void setPrev(Node*);
        Node * getPrev();

        Node(): priority{numeric_limits<double>::infinity()}, prev{NULL} {}

    private:
        double priority;
        string name;
        Node *prev;
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

Node * Node::getPrev() {
    return prev;
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
    unordered_map<string, double> costToArrive;
    for(int i = 0; i < e; i++) {
        //input : G D 6
        string s, d; double cost;
        input >> s >> d >> cost;

        //iterator to check if node already exists
        auto iter = nodes.find(s);

        //if node does not exist yet in map, else already exists
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

    //read in the source, destination and how many paths
    string source, dest; int k;

    input >> source >> dest >> k;

    /*
        add source node to path queue
        check it's connections, find cheapest, add to queue
    */

   //comparator function for sorting priority upon insert 
   struct CostCompare {
       bool operator()(Node & nodeA, Node & nodeB) {
           return nodeA.getPriority() > nodeB.getPriority();
       }
   };

    priority_queue<Node, vector<Node>, CostCompare> queue;
    
    costToArrive[source] = 0;
    costToArrive[dest] = numeric_limits<double>::infinity();
    queue.push(nodes[source]);

    //iterate over queue
    while(!queue.empty()) {
        Node n = queue.top();
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
                // cout << "neigbour: " << neighbour << " cost: " << currentDist + neighCost << " currentDist: " << currentDist << " neighCost: " << neighCost << endl;
                // cout << "costToArrive[neighbour]: " << costToArrive[neighbour] << endl;
                costToArrive[neighbour] = currentDist + neighCost;
                nodes[neighbour].setPrev(&n);
                nodes[neighbour].setPriority(costToArrive[neighbour]);
                queue.push(nodes[neighbour]);
            }
            
        }
    }

    for(auto &i: costToArrive) {
        cout << " Node: " << i.first << " " << i.second << endl;
    }
    cout << "cost to final: " << costToArrive[dest] << endl;

    end_t = clock();
    cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

    printf("Time taken: %f seconds\n", cpu_time_used);
    cout << endl; 
    input.close();
    return 0;
}




 //print out list of nodes
    // auto iterator = nodes.begin();
    // while (iterator != nodes.end()) {
    //     cout << iterator->first << ":: ";
    //     iterator->second.printConnections();
    //     iterator++;
    // }