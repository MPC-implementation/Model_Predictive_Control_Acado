#include <cstdio>
#include <iostream>
int main(int argc, char* argv[])
{
  using namespace std;
  freopen( "output.txt", "w", stdout );
  freopen( "error.txt", "w", stderr );
  int i = 1;
  cout << "Output message" << endl;
  fclose(stdout);
  cout << "Output message" << endl;
  cout << "Output message" << endl;
  cout << "Output message" << endl;
  freopen( "output.txt", "w", stdout );
  cout << i << endl;
  cerr << "Error message" << endl;
}