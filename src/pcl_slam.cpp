/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/



#include "../include/pcl_slam.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
// typedef pcl::PointXYZ pcl::PointXYZ;
// typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>;
// typedef pcl::PointNormal pcl::PointNormal;
// typedef pcl::PointCloud<pcl::PointNormal> pcl::PointCloud<pcl::PointNormal>;



// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void SLAMProcessor::showCloudsLeft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  // PCL_INFO ("Press q to begin the registration.\n");
  // p-> spin();
  p->spinOnce();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void SLAMProcessor::showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<pcl::PointNormal> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<pcl::PointNormal> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

// ////////////////////////////////////////////////////////////////////////////////
// /** \brief Load a set of PCD files that we want to register together
//   * \param argc the number of arguments (pass from main ())
//   * \param argv the actual command line arguments (pass from main ())
//   * \param models the resultant vector of point cloud datasets
//   */
// void loadData (int argc, char **argv, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > &models)
// {
//   std::string extension (".pcd");
//   // Suppose the first argument is the actual test model
//   for (int i = 1; i < argc; i++)
//   {
//     std::string fname = std::string (argv[i]);
//     // Needs to be at least 5: .plot
//     if (fname.size () <= extension.size ())
//       continue;

//     std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

//     //check that the argument is a pcd file
//     if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
//     {
//       // Load the cloud and saves it into the global list of models
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr();
//       pcl::io::loadPCDFile (argv[i], *cloud);
//       //remove NAN points from the cloud
//       std::vector<int> indices;
//       pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

//       models.push_back (m);
//     }
//   }
// }


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void SLAMProcessor::pairAlign (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.05);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (100);
  reg.setRANSACIterations(1000);
  reg.setRANSACOutlierRejectionThreshold(0.005);

  //reg.estimateRigidTransformationLM(points_with_normals_src, points_with_normals_tgt, Ti);


  for (int i = 0; i < 2; ++i)
  {
   // PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    if(!reg.hasConverged()){
      PCL_ERROR ("Registration did not converge\n");
    }

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;



		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();



    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  // targetToSource(0,0) = 1.0;
  // targetToSource(1,1) = 1.0;
  // targetToSource(2,2) = 1.0;
  // targetToSource(3,3) = 1.0;
  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

 //  p->removePointCloud ("source");
 //  p->removePointCloud ("target");

 //  PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tgt_h (output, 0, 255, 0);
 //  PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_h (cloud_src, 255, 0, 0);
 //  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
 //  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	// // PCL_INFO ("Press q to continue the registration.\n");
 // //  p->spin ();

 //  p->removePointCloud ("source"); 
 //  p->removePointCloud ("target");

  //add the source to the transformed target
  //*output += *cloud_src;
  
  final_transform = targetToSource;
 }


// /* ---[ */
// int main (int argc, char** argv)
// {
//   // Load data
//   std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
//   loadData (argc, argv, data);

//   // Check user input
//   if (data.empty ())
//   {
//     PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
//     PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
//     return (-1);
//   }
//   PCL_INFO ("Loaded %d datasets.", (int)data.size ());
  
//   // Create a PCLVisualizer object
//   p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
//   p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//   p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

// 	pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>), source, target;
//   Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
//   for (size_t i = 1; i < data.size (); ++i)
//   {
//     source = data[i-1].cloud;
//     target = data[i].cloud;

//     // Add visualization data
//     showCloudsLeft(source, target);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
//     PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
//     pairAlign (source, target, temp, pairTransform, true);

//     //transform current pair into the global transform
//     pcl::transformPointCloud (*temp, *result, GlobalTransform);

//     //update the global transform
//     GlobalTransform = pairTransform * GlobalTransform;

// 		//save aligned pair, transformed into the first cloud's frame
//     std::stringstream ss;
//     ss << i << ".pcd";
//     pcl::io::savePCDFile (ss.str (), *result, true);

//   }
// }
/* ]--- */

 SLAMProcessor::SLAMProcessor(int argc, char** argv)
  :m_sensorTransform(Eigen::Matrix4f::Identity())
  ,m_globalCloud(new pcl::PointCloud<pcl::PointXYZ>)
 {
   p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration");
   p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);
   //p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
 }

 void SLAMProcessor::addFrame(pcl::PointCloud<pcl::PointXYZ> &frame){
    //pcl::PointCloud<pcl::PointXYZ>::Ptr target = m_globalCloud; //frame.makeShared();

    //PCL_INFO ("adding frame\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawFrame = frame.makeShared(); //m_frames.back();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredFrame (new pcl::PointCloud<pcl::PointXYZ>);



    pcl::VoxelGrid<pcl::PointXYZ> grid;
    float leafSize =0.005;
    grid.setLeafSize (leafSize, leafSize, leafSize);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; 
    sor.setMeanK (10);
    sor.setStddevMulThresh (1.0);


    //PCL_INFO ("filtering input\n");
    std::vector<int> indices; 
    pcl::removeNaNFromPointCloud(*rawFrame,*filteredFrame, indices); 

    sor.setInputCloud (filteredFrame);
    sor.filter (*filteredFrame);


    if(! m_frames.empty()){
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr partiallyAlignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr alignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledFrame (new pcl::PointCloud<pcl::PointXYZ>);
      Eigen::Matrix4f alignmentCorrectionTransform;

      //PCL_INFO ("applying current alignment\n");
      pcl::transformPointCloud (*filteredFrame, *partiallyAlignedFrame, m_sensorTransform);


      // Add visualization data
      // PCL_INFO ("visualizing\n");
      // showCloudsLeft(partiallyAlignedFrame, m_globalCloud);

      //pcl::transformPointCloud (*source, *result, m_sensorTransform);
     
      //PCL_INFO ("Aligning frame of size %d with global map of size %d.\n", m_globalCloud->points.size (), partiallyAlignedFrame->points.size ());
      pairAlign (m_globalCloud, partiallyAlignedFrame, alignedFrame, alignmentCorrectionTransform, true);

      //transform current pair into the global transform
      //pcl::transformPointCloud (*temp, *result, m_sensorTransform);

      //update the global transform
      pcl::PointXYZ pt1(m_sensorTransform(0,3), m_sensorTransform(1,3) ,m_sensorTransform(2,3));

      m_sensorTransform = alignmentCorrectionTransform * m_sensorTransform;
      // m_sensorTransform(0,0) = 1.0;
      // m_sensorTransform(1,1) = 1.0;
      // m_sensorTransform(2,2) = 1.0;
      // m_sensorTransform(3,3) = 1.0;

       pcl::PointXYZ pt2(m_sensorTransform(0,3), m_sensorTransform(1,3) ,m_sensorTransform(2,3));

      std::string id = "line";
      id += boost::lexical_cast<std::string>(m_frames.size());
      p->addLine (pt1,pt2,0,0,1, id.c_str()); 

      showCloudsLeft(alignedFrame, m_globalCloud);
      
      //PCL_INFO ("downsampling aligned frame\n");
     //  pcl::transformPointCloud (*source, *result, m_sensorTransform);
      grid.setInputCloud (alignedFrame);
      grid.filter (*downsampledFrame);

      //PCL_INFO ("adding aligned frame to map\n");
     *m_globalCloud += *downsampledFrame;

     // if(m_frames.size() % 20 == 0){
     //   sor.setMeanK (10);
     //  sor.setStddevMulThresh (2.0);
     //  sor.setInputCloud (m_globalCloud);
     //  sor.filter (*m_globalCloud);
     // }


      // std::cout << "Alignment Correction: " << std::endl << alignmentCorrectionTransform <<std::endl;
      //std::cout << "New Global Transform: " << std::endl << m_sensorTransform <<std::endl;
     std::cout << "New Global Position: "  << m_sensorTransform(0,3) << "," << m_sensorTransform(1,3) << "," <<m_sensorTransform(2,3) << "," <<std::endl;

    }else{
      *m_globalCloud += *filteredFrame;

    }

    m_frames.push_back(filteredFrame);
    
 }