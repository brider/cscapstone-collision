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



void trans (pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
  Eigen::Affine3f t;
  pcl::getTransformation(1.0f,1.0f,1.0f,0.0f,0.0f,0.0f,t);
  pcl::transformPointCloud(*in, *out, t);
}



unsigned int text_id = 0;


/*void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}*/



void checkCol()
{

}

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

    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  //viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  Eigen::Affine3f t;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tCloud (new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::getTransformation(0.51f,0.0f,0.0f,0.0f,0.0f,0.0f,t);
  pcl::transformPointCloud(*cloud, *tCloud, t);

  //  trans(cloud, tCloud);
  viewer->updatePointCloud(tCloud, "sample cloud");

  //nested for loops test each point of a cloud with every point of another cloud
  //int c1;
  //int c2;
  /*unsigned int count = 0;
  for (c1 = 0; c1 < cloud->points.size(); c1++)
  {
    for (c2 = 0; c2 < tCloud->points.size(); c2++) 
    {
      if (pcl::euclideanDistance(cloud->points[c1], tCloud->points[c2]) < 0.01f)
      {
          cout << "touche " << c1 << " " << c2 << endl;
      }
    }
  }*/

	checkKdTree(cloud, tCloud);

	

	

  return (viewer);
}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "space" && event.keyDown ())
  {
    std::cout << "space was pressed" << std::endl;
	//trans(basic_cloud_ptr, basic_cloud_ptr2);
	//viewer->updatePointCloud(basic_cloud_ptr2, "sample cloud");

  }
}



/*boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}*/


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------


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


  
  /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);
*/
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    viewer = simpleVis(basic_cloud_ptr, basic_cloud_ptr2);


  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    //cout << "Test" << endl;
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
