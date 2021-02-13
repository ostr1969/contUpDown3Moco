//read two columns from fixed width 2 colums file
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
//#include "mypchip.h"
using namespace std;
int readx_y(string filename, double x[],double y[] ,int &sz)
{
   std::ifstream ifile(filename, std::ios::in);

    //check to see that the file was opened correctly:
    vector<vector<double>> scores;
        if (!ifile.is_open()) {
         std::cerr << "There was a problem opening the input file!\n";
         exit(1);//exit or do additional error checking
                   }
       double num1,num2;int i=0;
       vector<double> v(2);
    //keep storing values from the text file so long as data exists:
       while (ifile >> v[0]>>v[1]) {
        scores.push_back(v);
            }
     ifile.close();
     sz=scores.size();
     for (int i=0;i<scores.size();i++)
     {
	     x[i]=scores[i][0]; y[i]=scores[i][1];
     }
    //verify that the scores were stored correctly:
  //   for (vector<double> v:scores) { 
  //          cout << v[0] << " "<<v[1]<<endl; 
 //   } 
return 0;
} 

/*int main ()
{  
 vector<vector<double>> scores;
readxy( "src/delp1.txt",scores);
 cout<<pchiptx(scores,100)<<endl;
for (int i=0;i<scores.size();i++)
cout<< scores[i][0] << ":"<<scores[i][1]<< endl;
return 0;
}*/
