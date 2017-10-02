// Own
#include "util.h"

// Standard C/C++
#include <iostream>
#include <fstream>
using namespace std;

bool ReadFile(const char* pFileName, string& outFile)
{
    ifstream f(pFileName);
    
    bool ret = false;
    
    if (f.is_open()) {
        string line;
        while (getline(f, line)) {
            outFile.append(line);
            outFile.append("\n");
        }
        
        f.close();
        
        ret = true;
    }
    else {
		cout << "Error reading file: " << *pFileName << endl;
    }
    
    return ret;
}
uint Mod(uint start, uint edge, int step)
{
	if (step >= 0) return (start + step) % edge;
	else {
		int n = start + step;
		if (n > 0) return n;
		else return edge + step;
	}
}
