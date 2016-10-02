/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

// --------------
// -----Help-----
// --------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr2 (new pcl::PointCloud<pcl::PointXYZ>);


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);

void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


//nested for loops test each point of a cloud with every point of another cloud
void checkCol(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr tCloud)
{
	int c1;
	int c2;
	unsigned int count = 0;
	for (c1 = 0; c1 < cloud->points.size(); c1++)
	{
		for (c2 = 0; c2 < tCloud->points.size(); c2++) 
		{
			if (pcl::euclideanDistance(cloud->points[c1], tCloud->points[c2]) < 0.01f)
			{
			  cout << "touch at cloud point " << c1 << " and tCloud point" << c2 << endl;
			}
		}
	}
}

//KDTree using radius search
void checkKdTree(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr tCloud)
{
	int c1;
	for (c1 = 0; c1 < cloud->points.size(); c1++)
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(tCloud);
		std::vector<int> radiusSearchResults;
		std::vector<float> squaredDistanceResults;		

		if (kdtree.radiusSearch(cloud->points[c1], 0.01f, radiusSearchResults, squaredDistanceResults) > 0)
		{
			for (int i = 0; i < radiusSearchResults.size(); i++)
			{
				cout << "touch at point number " << c1 << " " << cloud->points[radiusSearchResults[i]].x << " " << cloud->points[radiusSearchResults[i]].y << " " << cloud->points[radiusSearchResults[i]].z << endl;
			}
		}
	}
}




boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud 2");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 2");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	Eigen::Affine3f t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tCloud (new pcl::PointCloud<pcl::PointXYZ>);  
	pcl::getTransformation(0.51f,0.0f,0.0f,0.0f,0.0f,0.0f,t);
	pcl::transformPointCloud(*cloud, *tCloud, t);

	viewer->updatePointCloud(tCloud, "sample cloud");

	//checkCol(cloud, tCloud);
	checkKdTree(cloud, tCloud);

	return (viewer);
}


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	//Loud pcd file into both cloud pointers
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("sphereCloud.pcd", *basic_cloud_ptr) == -1)
	{
		cout << "Something borked" << endl;
		return (-1);
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("sphereCloud.pcd", *basic_cloud_ptr2) == -1)
	{
		cout << "Something borked" << endl;
		return (-1);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = simpleVis(basic_cloud_ptr, basic_cloud_ptr2);

	//Main Loop
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
