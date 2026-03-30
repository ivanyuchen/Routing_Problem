#include <iostream>
#include <fstream>
using namespace std;
int main() {
    ifstream fin("Lab1/testcase/testcase1.txt");
    double a, b, c, d;
    if (fin >> a >> b >> c >> d) cout << "Read: " << a << " " << b << endl;
    else cout << "Failed to read" << endl;
    return 0;
}
