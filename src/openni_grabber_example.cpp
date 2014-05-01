 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
#include "../include/pcl_slam.h"


     SLAMProcessor *sp;

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {

      pcl::PointCloud<pcl::PointXYZ> ptr = *cloud ;
      pcl::PointCloud<pcl::PointXYZ> ptr2 = pcl::PointCloud<pcl::PointXYZ>(ptr) ;
      if(!ptr2.empty())
       sp->addFrame(ptr2);
//       if (!viewer.wasStopped())
//         viewer.showCloud (cloud);
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (true)
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main (int argc, char **argv)
 {

    sp = new SLAMProcessor(argc, argv);
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
