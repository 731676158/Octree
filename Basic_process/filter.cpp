#include <iostream>
#include <string>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
//using namespace pcl::visulization;

#define CloudData PointXYZ

bool read_pcd(string& file_name, PointCloud<CloudData>& cloud){
    if (loadPCDFile<CloudData>(file_name, cloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
        return false;
    }
    else return true;
}

void voxel_sample(PointCloud<CloudData>::Ptr input, PointCloud<CloudData>::Ptr output, float* res){
    ApproximateVoxelGrid<CloudData> voxelgrid;
    voxelgrid.setLeafSize(res[0], res[1], res[2]);
    voxelgrid.setInputCloud(input);
	voxelgrid.filter(*output);
}

bool write_pcd(string& file_name, PointCloud<CloudData>& cloud){
    if (savePCDFileASCII(file_name, cloud) == -1){
        PCL_ERROR("Couldn't save cloud.");
        return false;
    } else return true;
}

int main()
{
    /********降低点云稀疏度********/
    string path = "/home/jeff/codes/lidar1/pcds/local_1";
    string suffix = ".pcd";
    string file_in = path + suffix;
    PointCloud<CloudData>::Ptr cloud(new PointCloud<CloudData>());
    if (!read_pcd(file_in, *cloud))
    {
        cout << "Read failed!" << endl;
    }

    cout << "Before filtered points: " << cloud->width * cloud->height << endl;

    float res[3] = {1.0, 1.0, 1.0};
    voxel_sample(cloud, cloud, res);

    cout << "After filtered points: " << cloud->width * cloud->height << endl;

    string file_out = path + "_filted" + suffix;
    if (!write_pcd(file_out, *cloud))
    {
        cout << "Write failed!" << endl;
    }
}