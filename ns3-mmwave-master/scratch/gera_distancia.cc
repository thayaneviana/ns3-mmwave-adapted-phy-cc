#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <cmath>

using namespace std;


int
main (int argc, char *argv[])
{
int n;
int raio = 150;
int numUe=5;  
int dist[numUe];
double x[numUe];
double y[numUe];
srand( (unsigned)time(NULL) );
for (n=0 ; n < numUe; n++) 
 {
	dist[n] = rand() % (raio+1);
	x[n] = rand () % (dist[n]+1);
	y[n] = sqrt (pow(dist[n],2) - pow(x[n],2));
	//std::cout << "Nó=" << n << endl;
	//std::cout << "dist=" << dist[n] << endl;
	//std::cout << "x=" << x[n] << endl;
	//std::cout << "y=" << y[n] << endl;
	printf("uePositionAlloc->Add (Vector (%f, %f, 1.5)); \n", x[n], y[n]);
	//std::cout << "--------------------------------" << endl;
}
	
}
