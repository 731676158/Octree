#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <filesystem>

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char** argv) {
	/////The file to read from.
	//string infile;

	/////The file to output to.
	//string outfile;

	//// Declare the supported options.
	//po::options_description desc("Program options");
	//desc.add_options()
	//	//Options
	//	("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
	//	("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
	//	;
	//// Parse the command line
	//po::variables_map vm;
	//po::store(po::parse_command_line(argc, argv, desc), vm);

	//// Print help
	//if (vm.count("help"))
	//{
	//	cout << desc << "\n";
	//	return false;
	//}

	//// Process options.
	//po::notify(vm);

	//string infile{ "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\data\\0000000002.bin" };
	//string outfile{ "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\data\\0000000002.pcd" };
	//for (int i = 0; i < 154; ++i)
	int i = 1;
	{
		string infile_prefix{ "C:\\files\\codes\\git\\Octree\\data\\2011_09_26_drive_0005_sync\\2011_09_26\\2011_09_26_drive_0005_sync\\velodyne_points\\data\\" };
		string outfile_prefix{ "C:\\files\\codes\\git\\Octree\\data\\2011_09_26_drive_0005_sync\\pcds\\" };

		if (i < 10)
		{
			infile_prefix = infile_prefix + "000000000" + to_string(i) + ".bin";
			outfile_prefix = outfile_prefix + "000000000" + to_string(i) + ".pcd";
		}
		else if (i < 100)
		{
			infile_prefix = infile_prefix + "00000000" + to_string(i) + ".bin";
			outfile_prefix = outfile_prefix + "00000000" + to_string(i) + ".pcd";
		}
		else if (i < 1000)
		{
			infile_prefix = infile_prefix + "0000000" + to_string(i) + ".bin";
			outfile_prefix = outfile_prefix + "0000000" + to_string(i) + ".pcd";
		}

		//if (std::filesystem::exists(outfile_prefix)) continue;

		// load point cloud
		fstream input(infile_prefix.c_str(), ios::in | ios::binary);
		if (!input.good()) {
			cerr << "Could not read file: " << infile_prefix << endl;
			exit(EXIT_FAILURE);
		}
		input.seekg(0, ios::beg);

		pcl::PointCloud<PointXYZI>::Ptr points(new pcl::PointCloud<PointXYZI>);
		//pcl::PointCloud<PointCloud2>::Ptr points (new pcl::PointCloud<PointCloud2>);

		int j;
		for (j = 0; input.good() && !input.eof(); j++) {
			PointXYZI point;
			input.read((char*)&point.x, 3 * sizeof(float));
			input.read((char*)&point.intensity, sizeof(float));
			points->push_back(point);
		}
		input.close();

		cout << "Read KTTI point cloud with " << j << " points, writing to " << outfile_prefix << endl;

		pcl::PCDWriter writer;

		// Save DoN features
				//pcl::io::savePCDFileBinary(outfile, *points);
		writer.write<PointXYZI>(outfile_prefix, *points, false);
		//writer.write<PointCloud2> (outfile, *points, false);

	}


}


