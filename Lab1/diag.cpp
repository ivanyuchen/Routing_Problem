#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char* argv[]) {
    if (argc < 2) return 1;
    ifstream fin(argv[1]);
    string line;
    getline(fin, line); // cost
    
    if (getline(fin, line)) {
        stringstream ss(line);
        int p;
        vector<int> pins;
        while (ss >> p) pins.push_back(p);
        cout << "Top pins read: " << pins.size() << endl;
    }
    
    if (getline(fin, line)) {
        stringstream ss(line);
        int p;
        vector<int> pins;
        while (ss >> p) pins.push_back(p);
        cout << "Bottom pins read: " << pins.size() << endl;
    }
    return 0;
}
