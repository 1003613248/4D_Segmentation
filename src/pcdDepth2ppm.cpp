/***********************
This code convert a PointCloud<XYZRGBA> to a PPM file, which contain Depth channels
Using ppm P5, 16bit for a single depth pixel
***************************/
#include <stdlib.h>
//#include <stdio.h>
//#include <TestVideoSegmentation.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string.h>
#include <sstream>
#include <fstream>
//using namespace pcl;
 

int main(int argc, char** argv)
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PCDReader reader;
std::string data_folder;
data_folder = argv[1];
int n = 6; 
while (n< 50)
{
std::stringstream fileName, ppmName;
fileName<< data_folder;
fileName<<n;
fileName<<".pcd";
//ppmName<<data_folder;//save ppm to /data/n.ppm
ppmName<<n;
ppmName<<".ppm";
reader.read<pcl::PointXYZRGBA> (fileName.str(), *cloud);
 cloud->width = 640;
 cloud->height = 480;
  const int dimx = cloud->width, dimy = cloud->height;
const std::string tmp = ppmName.str();
const char* pName = tmp.c_str();
std::ofstream outfile(pName, std::ofstream::out);
//   fprintf(fp, "P5\n%d %d\n65535\n", dimx, dimy);

//outfile.put("P5\n%d %d\n65535\n", dimx, dimy)
outfile<< "P5"<<"\n"<<dimx<<" "<<dimy<<"\n"<<"65535"<<std::endl;

pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator pI = cloud->begin();
size_t size = dimx*dimy;
  for (int j = 0; j < dimy; ++j)
  {
    for (int i = 0; i < dimx; ++i)
    {
      static unsigned short depth;
      depth = 5000*(pI->z);  /* depth 1m = 5000 */
/***********************
in C++11, use std::to_string as:

std::string s = std::to_string(number);
char const *pchar = s.c_str();  //use char const* as target type

And in C++03, what you're doing is 

stringstream->string->char const* 

char const* pchar = temp_str.c_str(); //dont use cast
***********************************************/

std::stringstream td;
td<< depth;
std::string tem_d = td.str();

char const *w_d = tem_d.c_str();
  //    fwrite(depth, sizeof(short), 1, fp);
outfile.put(depth);
//std::cout<< pI->z<<  " d " <<depth<< " const " << w_d<< "\n";
	pI++;
    }
  }
 // fclose(fp);
outfile.close();
cloud->clear();
++n;
}
  return EXIT_SUCCESS;
}
