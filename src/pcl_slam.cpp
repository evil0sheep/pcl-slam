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



/*For RANSAC initial alignment, needs keypoint estimation, please save */
/* 
void
SLAMProcessor::computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  normals =  pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);

  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setInputCloud (points);
  norm_est.setSearchMethod (search_method_xyz);
  norm_est.setRadiusSearch (0.02);
  norm_est.compute (*normals);
}

// Compute the local feature descriptors
void
SLAMProcessor::computeLocalFeatures (const pcl::PointCloud<pcl::PointXYZ>::Ptr &points, const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &features)
{
  features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (points);
 // std::cout << fpfh_est.getKSearch () <<std::endl;
  fpfh_est.setInputNormals (normals);
  fpfh_est.setSearchMethod (search_method_xyz);
  fpfh_est.setRadiusSearch (0.002);
  fpfh_est.compute (*features);
}*/


 void SLAMProcessor::pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{

  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), targetToSource, initialTransform;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  float gridSize = 0.05;
  if (downsample)
  {
    grid.setLeafSize (gridSize, gridSize, gridSize);
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

  /* For RANSAC initial alignment, needs keypoint estimation, please save */
/*  pcl::PointCloud<pcl::PointXYZ>::Ptr src_aligned (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
  sac.setMinSampleDistance (gridSize);
  sac.setMaxCorrespondenceDistance (2 * gridSize);
  sac.setEuclideanFitnessEpsilon(1e-8);
  sac.setMaximumIterations (500);


  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features, tgt_features;
  pcl::PointCloud<pcl::Normal>::Ptr src_normals, tgt_normals;

  computeSurfaceNormals(src, src_normals);
  computeSurfaceNormals(tgt, tgt_normals);

  computeLocalFeatures(src, src_normals, src_features);
  computeLocalFeatures(tgt, tgt_normals, tgt_features);

  std::cout << src->points.size() << ", " << src_normals->points.size() << ", " << src_features->points.size() << std::endl;

  sac.setInputSource (src);
  sac.setSourceFeatures (src_features);

  sac.setInputTarget (tgt);
  sac.setTargetFeatures (tgt_features);

  sac.align (*src_aligned);
  initialTransform = sac.getFinalTransformation();

  if(!sac.hasConverged()){
    PCL_ERROR ("SAC Alignment did not converge\n");
  }
*/


  pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.setTransformationEpsilon (1e-8);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (gridSize); 
  reg.setMaximumIterations (10); 
  reg.setEuclideanFitnessEpsilon(1e-8);
  // Set the point representation
  //reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (src);
  reg.setInputTarget (tgt);


  
  //pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_tgt;
   
  reg.setRANSACIterations(1000);
  reg.setRANSACOutlierRejectionThreshold(gridSize);

  //std::cout << reg.getEuclideanFitnessEpsilon() << std::endl;
  //std::cout << src->points.size() << ", " << cloud_src->points.size() << ", " << tgt->points.size() << ", " << cloud_tgt->points.size()  << std::endl;

  reg.align (*reg_result);

  if(!reg.hasConverged()){
    PCL_ERROR ("Registration did not converge\n");
  }

  //accumulate transformation between each Iteration
  Ti = reg.getFinalTransformation () ;

  


  targetToSource =  Ti.inverse() ;

  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


  
  final_transform = targetToSource;
 }



 SLAMProcessor::SLAMProcessor(int argc, char** argv)
  :m_sensorTransform(Eigen::Matrix4f::Identity())
  ,m_globalCloud(new pcl::PointCloud<pcl::PointXYZ>)
  //,search_method_xyz(new pcl::search::KdTree<pcl::PointXYZ>)
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

    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter (true); // Initializing with true will allow us to extract the removed indices
    // rorfilter.setInputCloud (filteredFrame);
    // rorfilter.setRadiusSearch (0.005);
    // rorfilter.setMinNeighborsInRadius (4);
    // rorfilter.setNegative (false);
    // rorfilter.filter (*filteredFrame);


    if(! m_frames.empty()){
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr partiallyAlignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr alignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledFrame (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledMap (new pcl::PointCloud<pcl::PointXYZ>);
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

      grid.setInputCloud (alignedFrame);
      grid.filter (*downsampledFrame);

      grid.setInputCloud (m_globalCloud);
      grid.filter (*downsampledMap);

      showCloudsLeft(alignedFrame, downsampledMap);
      
      //PCL_INFO ("downsampling aligned frame\n");
     //  pcl::transformPointCloud (*source, *result, m_sensorTransform);
      // grid.setInputCloud (alignedFrame);
      // grid.filter (*downsampledFrame);

      //PCL_INFO ("adding aligned frame to map\n");
     *m_globalCloud += *downsampledFrame;
      // grid.setInputCloud (m_globalCloud);
      // grid.filter (*m_globalCloud);

     // if(m_frames.size() % 20 == 0){
     //   sor.setMeanK (10);
     //  sor.setStddevMulThresh (2.0);
     //  sor.setInputCloud (m_globalCloud);
     //  sor.filter (*m_globalCloud);
     // }


      // std::cout << "Alignment Correction: " << std::endl << alignmentCorrectionTransform <<std::endl;
      //std::cout << "New Global Transform: " << std::endl << m_sensorTransform <<std::endl;
      //std::cout << "New Global Scale: "  << m_sensorTransform(0,0) << "," << m_sensorTransform(1,1) << "," <<m_sensorTransform(2,2) << "," <<std::endl;
     //std::cout << "New Global Position: "  << m_sensorTransform(0,3) << "," << m_sensorTransform(1,3) << "," <<m_sensorTransform(2,3) << "," <<std::endl;

    }else{
      // grid.setInputCloud (filteredFrame);
      // grid.filter (*filteredFrame);
      *m_globalCloud += *filteredFrame;

    }

    m_frames.push_back(filteredFrame);
    
 }