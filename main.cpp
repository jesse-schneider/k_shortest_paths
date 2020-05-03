#include<bits/stdc++.h>
#include"node.cpp"

using namespace std;

/* imported Node class stores each node
    each class stores:
        - list of nodes this node connects to
        - each cost of the connection
        - the previous node in the path
        - the list of forbidden nodes
*/

int getMin(vector<pair<string, double>>);
vector<pair<string, double>> getPath(string dest, string & source, unordered_map<string, Node> nodes);
void findPath(unordered_map<string, double> &costToArrive, unordered_map<string, Node> &nodes, string source, string dest);



int main(int argc, char ** argv) {

    string filename = "finalInput.txt";
    ifstream input;

    clock_t start_t, end_t;
    double cpu_time_used;
    start_t = clock();

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
        to find k paths, we are going to perform dijkstra's algorithm to find the cheapest path
        using a priority queue and processing nodes sequentially (nodes added to queue as explored)

        when we have a working path, we find the smallest edge in the current path, we:
             - make it forbidden
             - find a new path without that edge
        When we have a new path completed, we replace that edge, and repeat this proces on the new path

        continue this until all k-1 paths are found
    */

   vector<vector<pair<string, double>>> allPaths;
   pair<string, double> forbiddenEdge;

   for(int i = 0; i < k; i++) {

        //find a path using dijkstra's algorithm
        findPath(costToArrive, nodes, source, dest);
        vector<pair<string, double>> path = getPath(dest, source, nodes);

        //print out the cost to stdout
        cout << costToArrive[dest];
        if(i != (k - 1))
            cout << ", ";
        allPaths.push_back(path);

        // find the minimum cost edge, remove the node previous to it before it from the posible path, move back a node and find path
        int minIndex = getMin(path);
        forbiddenEdge = path.at(minIndex - 1);
        pair<string, double> forbiddenNode = path.at(minIndex);

        nodes[forbiddenEdge.first].clearForbidden();
        nodes[forbiddenEdge.first].addForbiddenPath(forbiddenNode.first);

        for(auto &i: costToArrive) {
            i.second = numeric_limits<double>::infinity();
        }
   }

    end_t = clock();
    cpu_time_used = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;

    printf(" Time taken: %f seconds", cpu_time_used);
    cout << endl; 
    return 0;
}



//comparator function for finding the smallest value in a vector of pair<string, double>
struct MinCompare {
       bool operator()(const pair<string, double> a, const pair<string, double> b) const {
           return a.second < b.second;
       }
   };

//function that implements MinCompare to find the minCost connection 
int getMin(vector<pair<string, double>> v) {
        pair<string, double> min = *min_element(v.begin(), v.end(), MinCompare());
        
        for(int i = 0; i < v.size(); i++) {
            if(v[i].first == min.first)
                return i;
        }
    }

vector<pair<string, double>> getPath(string dest, string & source, unordered_map<string, Node> nodes) {
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
    return path;
}

void findPath(unordered_map<string, double> &costToArrive, unordered_map<string, Node> &nodes, string source, string dest) {

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

    //iterate over queue
    while(!queue.empty()) {
        Node n = queue.top();
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

            //if forbidden, skip node
            if(n.checkIfForbidden(neighbour))
                continue;

            //check if we need to change distances
            if(costToArrive[neighbour] > currentDist + neighCost) {
                costToArrive[neighbour] = currentDist + neighCost;
                if(nodes.find(neighbour) != nodes.end()) {
                    nodes[neighbour].setPriority(costToArrive[neighbour]);
                    nodes[neighbour].setPrevious(n.getName());
                    queue.push(nodes[neighbour]);
                } 
            }
        }
    }
}

