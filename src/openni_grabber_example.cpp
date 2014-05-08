 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
#include "../include/pcl_slam.h"

     SLAMProcessor *sp;
pcl::PointCloud<pcl::PointXYZ> pc;
int pcwrite;
boost::mutex mtx;
bool available = false;

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer ()  {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
//pcl::PointCloud<pcl::PointXYZ> cloud2;
//cloud2.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
//cloud2.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
//cloud2.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
//cloud2.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
      if(!cloud->empty())
      {
        mtx.lock();
        available = true;
        pc = *cloud;
        mtx.unlock();
      }
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
         if(available && mtx.try_lock())
         {
           available = false;
pcl::PointCloud<pcl::PointXYZ> pc2;
                        pcl::copyPointCloud(pc,pc2); 
       sp->addFrame(pc2);
       mtx.unlock();
         }
       }

       interface->stop ();
     }

     //pcl::visualization::CloudViewer viewer;
 };

 int main (int argc, char **argv)
 {
pcwrite = 0;
    sp = new SLAMProcessor(argc, argv);
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
