#include<bits/stdc++.h>

using namespace std;

/* Node class to store each node
    each class stores:
        - list of nodes this node connects to
        - each cost of the connection
        - the previous node in the path
        - the list of forbidden nodes
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
        void setPrevious(const string & prev);
        string & getPrevious();
        double getPreviousCost(const string & destination);
        void addForbiddenPath(const string & f);
        void clearForbidden();
        bool checkIfForbidden(const string & val);

        Node(): priority{numeric_limits<double>::infinity()} {}

    private:
        double priority;
        string name;
        string previous = "";
        vector<string> connections;
        vector<double> connectCosts;
        vector<string> forbidden;
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
void Node::setPrevious(const string & prev) {
    previous = prev;
}

string & Node::getPrevious() {
    return previous;
}
double Node::getPreviousCost(const string & destination) {

    for(int i = 0; i < connections.size(); i++) {
        if(connections[i] == destination) {
            return connectCosts[i];
        }
    }
}

void Node::addForbiddenPath(const string & f) {
    forbidden.push_back(f);
}

void Node::clearForbidden() {
    forbidden.clear();
}

bool Node::checkIfForbidden(const string & key) {

    for(auto &i: forbidden) {
        if(i == key)
            return true;
    }
    return false;
}
