/***********************
This code convert a PointCloud<XYZRGBA> to a PPM file, which contain RGB channels
***************************/
#include <stdlib.h>
#include <stdio.h>
//#include <TestVideoSegmentation.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string.h>
#include <sstream>
//using namespace pcl;
 

int main(int argc, char** argv)
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PCDReader reader;
std::string data_folder;
data_folder = argv[1];
int n = 1; 
while (n< 828)
{
std::stringstream fileName, ppmName;
fileName<< data_folder;
fileName<<n;
fileName<<".pcd";
ppmName<<data_folder;//save ppm to /data/n.ppm
ppmName<<n;
ppmName<<".ppm";
reader.read<pcl::PointXYZRGBA> (fileName.str(), *cloud);
 cloud->width = 640;
 cloud->height = 480;
  const int dimx = cloud->width, dimy = cloud->height;
char pName[sizeof(n)+strlen(".ppm")];
snprintf(pName, sizeof(pName), "%d.ppm", n);
 // FILE *fp = fopen(pName, "wb"); /* b - binary mode */
  FILE *fp = fopen(pName, "wb"); /* b - binary mode */
  (void) fprintf(fp, "P6\n%d %d\n255\n", dimx, dimy);
pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator pI = cloud->begin();
  for (int j = 0; j < dimy; ++j)
  {
    for (int i = 0; i < dimx; ++i)
    {
      static unsigned char color[3];
      color[0] = pI->r;  /* red */
      color[1] = pI->g;  /* green */
      color[2] = pI->b;  /* blue */
      (void) fwrite(color, 1, 3, fp);
	pI++;
    }
  }
  (void) fclose(fp);
cloud->clear();
++n;
}
  return EXIT_SUCCESS;
}
