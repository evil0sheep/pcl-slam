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
 #include <sys/time.h> 

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


double get_wall_time(){ 
  struct timeval time; 
  if (gettimeofday(&time,NULL)){ 
    // Handle error 
    return 0; 
  } return (double)time.tv_sec + (double)time.tv_usec * .000001; 
}


void SLAMProcessor::setGridSize(float gridSize)
{
  this->gridSize = gridSize;
}

void SLAMProcessor::setLeafSize(float leafSize)
{
  this->leafSize = leafSize;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void SLAMProcessor::showCloudsLeft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");
  PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 255, 0, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 0, 255, 0);


  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

   p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "vp1_target");
   p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "vp1_source");
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
  {
    PCL_WARN ("Cannot create curvature color handler!");
  }

  PointCloudColorHandlerGenericField<pcl::PointNormal> src_color_handler (cloud_source, "curvature");

  if (!src_color_handler.isCapable ())
  {
    PCL_WARN ("Cannot create curvature color handler!");
  }

  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
  p->spinOnce();
}



/*For RANSAC initial alignment, needs keypoint estimation, please save */

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
}


void SLAMProcessor::computeNarfKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr){

  pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  bool setUnseenToMaxRange = false;
  //std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  // if (!pcd_filename_indices.empty ())
  // {
  //   std::string filename = argv[pcd_filename_indices[0]];
  //   if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
  //   {
  //     cerr << "Was not able to open file \""<<filename<<"\".\n";
  //     printUsage (argv[0]);
  //     return 0;
  //   }
  //   scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
  //                                                              point_cloud.sensor_origin_[1],
  //                                                              point_cloud.sensor_origin_[2])) *
  //                       Eigen::Affine3f (point_cloud.sensor_orientation_);
  //   std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
  //   if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
  //     std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  // }
  // else
  // {
  //   setUnseenToMaxRange = true;
  //   cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
  //   for (float x=-0.5f; x<=0.5f; x+=0.01f)
  //   {
  //     for (float y=-0.5f; y<=0.5f; y+=0.01f)
  //     {
  //       pcl::PointXYZ point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
  //       point_cloud.points.push_back (point);
  //     }
  //   }
  //   point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  // }
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
    float angular_resolution = 0.02f;

  float noise_level = 0.0;
  float min_range = 0.0f;
  float angular_w =pcl::deg2rad (360.0f), angular_h = pcl::deg2rad (180.0f);

  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, angular_w, angular_h,
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  // boost::shared_ptr<pcl::RangeImage> range_image_vis_ptr (new pcl::RangeImage);
  // pcl::RangeImage& range_image_vis = *range_image_vis_ptr;   
  // range_image_vis.createFromPointCloud (point_cloud, 0.001, angular_w, angular_h,
  //                                  scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  // range_image_vis.integrateFarRanges (far_ranges);
  // if (setUnseenToMaxRange)
  //   range_image_vis.setUnseenToMaxRange ();
  

  //printf("%f\n", pcl::deg2rad (72.0f / 320));
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  // pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  // viewer.setBackgroundColor (1, 1, 1);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  // viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  // //viewer.addCoordinateSystem (1.0f, "global");
  // //PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  // //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  // viewer.initCameraParameters ();
  // //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  
  // // --------------------------
  // // -----Show range image-----
  // // --------------------------
  
  // range_image_widget.showRangeImage (range_image_vis);
  // range_image_widget.spinOnce();


  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  //float support_size = 0.01f;
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size =  0.2f;
  narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
   narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.01;
  //narf_keypoint_detector.getParameters ().use_recursive_scale_reduction = true;
  narf_keypoint_detector.getParameters ().min_interest_value = .45;


  // printf("\n\nmin_surface_change_score %f\n", narf_keypoint_detector.getParameters ().min_surface_change_score);
   //printf("min_interest_value %f\n", narf_keypoint_detector.getParameters ().min_interest_value);
  // printf("no_of_polynomial_approximations_per_point %d\n", narf_keypoint_detector.getParameters ().no_of_polynomial_approximations_per_point);
  // printf("min_distance_between_interest_points %f\n", narf_keypoint_detector.getParameters ().min_distance_between_interest_points);
  // printf("distance_for_additional_points %f\n", narf_keypoint_detector.getParameters ().distance_for_additional_points);
  // printf("optimal_distance_to_high_surface_change %f\n", narf_keypoint_detector.getParameters ().optimal_distance_to_high_surface_change);
  // printf("optimal_range_image_patch_size %d\n", narf_keypoint_detector.getParameters ().optimal_range_image_patch_size);
  // printf("use_recursive_scale_reduction %d\n", narf_keypoint_detector.getParameters ().use_recursive_scale_reduction);
   //printf("calculate_sparse_interest_image %d\n\n\n", narf_keypoint_detector.getParameters ().calculate_sparse_interest_image);

  
  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  //std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";



  // ----------------------------------------------
  // -----Show keypoints in range image widget-----
  // ----------------------------------------------
  // float w_ratio = range_image_vis.width/range_image.width;
  // float h_ratio = range_image_vis.height/range_image.height;
  // for (size_t i=0; i<keypoint_indices.points.size (); ++i)
  //   range_image_widget.markPoint (w_ratio * (keypoint_indices.points[i]%range_image.width),
  //                                 h_ratio * (keypoint_indices.points[i]/range_image.width),
  //                                 pcl::visualization::blue_color);

  //markPoint (size_t u, size_t v, Vector3ub fg_color, Vector3ub bg_color=red_color, float radius=2)
  
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  //pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();




  // *m_keypointsCloud += *keypoints_ptr;
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (m_keypointsCloud, 0, 0, 0);

  // p->removePointCloud ("keypoints");
  // p->addPointCloud<pcl::PointXYZ> (m_keypointsCloud, keypoints_color_handler, "keypoints");
  // p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
  
}



void SLAMProcessor::pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f &transform_guess, Eigen::Matrix4f &final_transform, bool downsample)
{
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), targetToSource, initialTransform;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;

;
  //float gridSize = 0.10;
  // if (downsample)
  // {
  //   grid.setLeafSize (gridSize, gridSize, gridSize);
  //   grid.setInputCloud (cloud_src);
  //   grid.filter (*src);
  //   grid.setInputCloud (cloud_tgt);
  //   grid.filter (*tgt);
  // }
  // else
  // {
    src = cloud_src;
    tgt = cloud_tgt;
  // }

  double t0 = get_wall_time();

  // printf("computing keypoints for src cloud of size %lu\n", src->size());
  // computeNarfKeypoint(cloud_src);

  computeNarfKeypoint(cloud_tgt, tgt_keypoints);

  pcl::transformPointCloud (*tgt_keypoints, *tgt, transform_guess);


  double t1 = get_wall_time();

  //printf("computing keypoints took %f second\n", t1-t0);

  /* For RANSAC initial alignment, needs keypoint estimation, please save */
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr src_aligned (new pcl::PointCloud<pcl::PointXYZ>);

  //   pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
  //   sac.setMinSampleDistance (gridSize);
  //   sac.setMaxCorrespondenceDistance (2 * gridSize);
  //   sac.setEuclideanFitnessEpsilon(1e-8);
  //   sac.setMaximumIterations (5000);


  //   pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features, tgt_features;
  //   pcl::PointCloud<pcl::Normal>::Ptr src_normals, tgt_normals;

  //   computeSurfaceNormals(src, src_normals);
  //   computeSurfaceNormals(tgt, tgt_normals);

  //   computeLocalFeatures(src, src_normals, src_features);
  //   computeLocalFeatures(tgt, tgt_normals, tgt_features);

  //   std::cout << src->points.size() << ", " << src_normals->points.size() << ", " << src_features->points.size() << std::endl;
  //   std::cout << tgt->points.size() << ", " << tgt_normals->points.size() << ", " << tgt_features->points.size() << std::endl;

  //   sac.setInputSource (src);
  //   sac.setSourceFeatures (src_features);

  //   sac.setInputTarget (tgt);
  //   sac.setTargetFeatures (tgt_features);

  //   sac.align (*src_aligned);
  //   initialTransform = sac.getFinalTransformation();

  //   if(!sac.hasConverged()){
  //     PCL_ERROR ("SAC Alignment did not converge\n");
  //   }

  // pcl::transformPointCloud (*tgt, *tgt, initialTransform);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.setTransformationEpsilon (1e-8);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.05);
  reg.setMaximumIterations (50);
  reg.setEuclideanFitnessEpsilon(1);
  // Set the point representation
  //reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputSource (src);
  reg.setInputTarget (tgt);
  //pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_tgt;
  reg.setRANSACIterations(1000);
  reg.setRANSACOutlierRejectionThreshold(0.05);
  //std::cout << reg.getEuclideanFitnessEpsilon() << std::endl;
  //std::cout << src->points.size() << ", " << cloud_src->points.size() << ", " << tgt->points.size() << ", " << cloud_tgt->points.size()  << std::endl;
  reg.align (*reg_result);

  if(!reg.hasConverged())
  {
    PCL_ERROR ("Registration did not converge\n");
  }

  //accumulate transformation between each Iteration
  Ti = reg.getFinalTransformation () ;
  targetToSource =  Ti.inverse() ;
  pcl::transformPointCloud (*tgt, *output, targetToSource);
  final_transform = targetToSource;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void * vd)
{
  SLAMProcessor * sp = (SLAMProcessor *) vd;

  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;
    sp->m_frames.clear();
    sp->m_globalCloud->clear();
    sp->m_filteredCloud->clear();
    sp->m_unfilteredCloud->clear();
    sp->m_sensorTransform = Eigen::Matrix4f::Identity();
    sp->p->removeAllShapes();
    sp->totalPoints=0;
  }
}

SLAMProcessor::SLAMProcessor(int argc, char** argv)
  : m_sensorTransform(Eigen::Matrix4f::Identity())
  , m_globalCloud(new pcl::PointCloud<pcl::PointXYZ>)
  , m_keypointsCloud(new pcl::PointCloud<pcl::PointXYZ>)
  , m_filteredCloud(new pcl::PointCloud<pcl::PointXYZ>)
  , m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>)
  , m_frameCount(0)
  , gridSize(.05)
  , leafSize(.005)
  ,totalPoints(0)
  ,range_image_widget ("Range image")
  ,search_method_xyz(new pcl::search::KdTree<pcl::PointXYZ>)
{
  // gridSize = atof(argv[1]); 
  // leafSize = atof(argv[2]); 
  if(argc > 2){
    param1 = atof(argv[1]); 
    param2 = atof(argv[2]); 
    printf("params: %f, %f\n", param1, param2);
  }
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration");
  p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);
  p->setBackgroundColor(1, 1, 1);
  p->registerKeyboardCallback (keyboardEventOccurred, (void *) this);
  //p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  m_filter = new DoubleExpFilter(glm::dvec3(0), 0.2, 0.7);
}

void SLAMProcessor::addFrame(pcl::PointCloud<pcl::PointXYZ> &frame, bool filter)
{
  double t0 = get_wall_time();




  //pcl::PointCloud<pcl::PointXYZ>::Ptr target = m_globalCloud; //frame.makeShared();
  //PCL_INFO ("adding frame\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr rawFrame = frame.makeShared(); //m_frames.back();
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredFrame (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::VoxelGrid<pcl::PointXYZ> grid;
  // leafSize =0.005;
  // grid.setLeafSize (leafSize, leafSize, leafSize);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setMeanK (10);
  // sor.setStddevMulThresh (1.0);
  // //PCL_INFO ("filtering input\n");
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*rawFrame, *filteredFrame, indices);

  // if(filter)
  // {
  // sor.setInputCloud (filteredFrame);
  // sor.filter (*filteredFrame);
  // }

  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter (true); // Initializing with true will allow us to extract the removed indices
  // rorfilter.setInputCloud (filteredFrame);
  // rorfilter.setRadiusSearch (0.005);
  // rorfilter.setMinNeighborsInRadius (4);
  // rorfilter.setNegative (false);
  // rorfilter.filter (*filteredFrame);

  bool  downsample =false;

  if(! m_frames.empty())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr partiallyAlignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedFrame (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledFrame (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledMap (new pcl::PointCloud<pcl::PointXYZ>);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceFrame (new pcl::PointCloud<pcl::PointXYZ>);
    int target_num_points = 3000;
    int n = ((float) target_num_points / totalPoints) * m_frames.size();
    //int n = 25;
    int max_frame_gap = 100;
    if(m_frames.size() > n){
      for(int i = max_frame_gap; i > 0; i--){
        if(m_frames.size() > n * i){
          int firstIndex = m_frames.size() - n * i;
          firstIndex = (firstIndex / i) * i;
          int j;
          for(j = firstIndex; j < m_frames.size(); j+=i){
            *sourceFrame += *(m_frames[j]);
          }

          for(j; j < m_frames.size(); j++){
            *sourceFrame += *(m_frames[j]);
          }

          break;
        }
        
      }

        // pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter (true); // Initializing with true will allow us to extract the removed indices
        // rorfilter.setInputCloud (sourceFrame);
        // rorfilter.setRadiusSearch (0.05);
        // rorfilter.setMinNeighborsInRadius (6);
        // rorfilter.setNegative (false);
        // rorfilter.filter (*sourceFrame);

        // leafSize =0.01;
        // grid.setLeafSize (leafSize, leafSize, leafSize);
        // grid.setInputCloud (sourceFrame);
        // grid.filter (*sourceFrame);

    }else{
      for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> >::iterator
           it = m_frames.begin(); it != m_frames.end(); ++it){
        *sourceFrame += **it;
      }
    }




      //         rorfilter.setInputCloud (m_globalCloud);
      // rorfilter.filter (*downsampledMap);
      

    Eigen::Matrix4f alignmentCorrectionTransform;
    //PCL_INFO ("applying current alignment\n");
    //pcl::transformPointCloud (*filteredFrame, *partiallyAlignedFrame, m_sensorTransform);
    // Add visualization data
    // PCL_INFO ("visualizing\n");
    // showCloudsLeft(partiallyAlignedFrame, m_globalCloud);
    //pcl::transformPointCloud (*source, *result, m_sensorTransform);
    //PCL_INFO ("Aligning to target frame of size %d.\n", sourceFrame->points.size ());

    pairAlign (sourceFrame, filteredFrame, alignedFrame, m_sensorTransform, alignmentCorrectionTransform, downsample);
    


    //transform current pair into the global transform
    //pcl::transformPointCloud (*temp, *result, m_sensorTransform);
    //update the global transform
    pcl::PointXYZ pt1(m_sensorTransform(0, 3), m_sensorTransform(1, 3) , m_sensorTransform(2, 3));
    m_sensorTransform = alignmentCorrectionTransform * m_sensorTransform;
    // m_sensorTransform(0,0) = 1.0;
    // m_sensorTransform(1,1) = 1.0;
    // m_sensorTransform(2,2) = 1.0;
    // m_sensorTransform(3,3) = 1.0;
    pcl::PointXYZ pt2(m_sensorTransform(0, 3), m_sensorTransform(1, 3) , m_sensorTransform(2, 3));

    // float c1 =.8;
    // float c2 = 1-c1;
    // pcl::PointXYZ pt3(c1 * pt1.x + c2 * pt2.x, c1 * pt1.y + c2 * pt2.y, c1 * pt1.z + c2 * pt2.z);
    // m_sensorTransform(0, 3) = pt3.x;
    // m_sensorTransform(1, 3) = pt3.y;
    // m_sensorTransform(2, 3) = pt3.z;

    glm::dvec3 filterStart = m_filter->getCurrentPosition();
    glm::dvec3 filterEnd = m_filter->updatePosition(glm::dvec3(pt2.x, pt2.y, pt2.z));
    glm::dvec3 filterVec = filterEnd - filterStart;
    glm::dvec3 unfStart = glm::dvec3(pt1.x, pt1.y, pt1.z);
    glm::dvec3 unfEnd = glm::dvec3(pt2.x, pt2.y, pt2.z);
    glm::dvec3 unfVec = unfEnd - unfStart;



    int numSteps = glm::length(unfVec) / .0005;
    for(int i = 1; i <= numSteps; i++){
      glm::dvec3 interpUnf = unfStart + (unfVec * (1.0 / numSteps * i));
      m_unfilteredCloud->push_back(pcl::PointXYZ(interpUnf.x, interpUnf.y, interpUnf.z));
    }

    numSteps = glm::length(filterVec) / .001;
    for (int i = 1; i <= numSteps; i++) {
      glm::dvec3 interpFilt = filterStart + (filterVec * (1.0 / numSteps * i));
      m_filteredCloud->push_back(pcl::PointXYZ(interpFilt.x, interpFilt.y, interpFilt.z));
    }

    p->removePointCloud ("unfiltered_cloud");
    p->removePointCloud ("filtered_cloud");
    PointCloudColorHandlerCustom<pcl::PointXYZ> unf_h (m_unfilteredCloud, 255, 0, 255);
    PointCloudColorHandlerCustom<pcl::PointXYZ> fil_h (m_filteredCloud, 0, 0, 255);


    p->addPointCloud (m_unfilteredCloud, unf_h, "unfiltered_cloud", vp_1);
    p->addPointCloud (m_filteredCloud, fil_h, "filtered_cloud", vp_1);

    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "unfiltered_cloud");
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");
    p->spinOnce();


/*
    std::string id = "line";
    id += boost::lexical_cast<std::string>(m_frames.size());
    p->addLine (pt1, pt2, 1, 0, 1, id.c_str());

    

    std::string filterLineId = "Filtered Transform";
    filterLineId += boost::lexical_cast<std::string>(m_frames.size());
    glm::dvec3 originalPoint = m_filter->getCurrentPosition();
    glm::dvec3 newPoint = m_filter->updatePosition(glm::dvec3(pt2.x, pt2.y, pt2.z));
    pcl::PointXYZ fpt1(originalPoint.x, originalPoint.y, originalPoint.z);
    pcl::PointXYZ fpt2(newPoint.x, newPoint.y, newPoint.z);
    p->addLine(fpt1, fpt2, 1, 0, 1, filterLineId);
*/
  


    double test = get_wall_time();
    // if(downsample){
    //   grid.setInputCloud (alignedFrame);
    //   grid.filter (*downsampledFrame);
    //   grid.setInputCloud (m_globalCloud);
    //   grid.filter (*downsampledMap);
    //   showCloudsLeft(alignedFrame, downsampledMap);
    //   *m_globalCloud += *downsampledFrame;
    // }else{
      showCloudsLeft(alignedFrame, sourceFrame);
      //*m_globalCloud += *alignedFrame;
    // }
      printf("test time: %f seconds\n", get_wall_time() - test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr storeFrame (new pcl::PointCloud<pcl::PointXYZ>);
    *storeFrame += *alignedFrame;
    m_frames.push_back(storeFrame);
    totalPoints += storeFrame->points.size();

    // grid.setInputCloud (m_globalCloud);
    // grid.filter (*m_globalCloud);

    // sor.setMeanK (50);
    // sor.setStddevMulThresh (4.0);
    // sor.setInputCloud (m_globalCloud);
    // sor.filter (*m_globalCloud);
    
    //PCL_INFO ("downsampling aligned frame\n");
    //  pcl::transformPointCloud (*source, *result, m_sensorTransform);
    // grid.setInputCloud (alignedFrame);
    // grid.filter (*downsampledFrame);
    //PCL_INFO ("adding aligned frame to map\n");
    
    // grid.setInputCloud (m_globalCloud);
    // grid.filter (*m_globalCloud);



    // if(m_globalCloud->size() > 2000){

    //             pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter (true); // Initializing with true will allow us to extract the removed indices
    //     rorfilter.setInputCloud (m_globalCloud);
    //     rorfilter.setRadiusSearch (0.05);
    //     rorfilter.setMinNeighborsInRadius (4);
    //     rorfilter.setNegative (false);
    //     rorfilter.filter (*m_globalCloud);


    //       leafSize =0.01;
    // grid.setLeafSize (leafSize, leafSize, leafSize);
    //   grid.setInputCloud (m_globalCloud);
    //    grid.filter (*m_globalCloud);

    // }
    // std::cout << "Alignment Correction: " << std::endl << alignmentCorrectionTransform <<std::endl;
    //std::cout << "New Global Transform: " << std::endl << m_sensorTransform <<std::endl;
    //std::cout << "New Global Scale: "  << m_sensorTransform(0,0) << "," << m_sensorTransform(1,1) << "," <<m_sensorTransform(2,2) << "," <<std::endl;
    //std::cout << "New Global Position: "  << m_sensorTransform(0,3) << "," << m_sensorTransform(1,3) << "," <<m_sensorTransform(2,3) << "," <<std::endl;
  }
  else
  {
    // grid.setInputCloud (filteredFrame);
    // grid.filter (*filteredFrame);
    computeNarfKeypoint(filteredFrame, filteredFrame);
    //*m_globalCloud += *filteredFrame;

    pcl::PointCloud<pcl::PointXYZ>::Ptr storeFrame (new pcl::PointCloud<pcl::PointXYZ>);
    *storeFrame += *filteredFrame;
    m_frames.push_back(storeFrame);
    totalPoints += storeFrame->points.size();
    double lastFrameEndTime = get_wall_time();
  }

  
  double t1 = get_wall_time();
  printf("frame processing took %f seconds\n", t1-t0);


  //printf("full frame took %f seconds\n", t1-lastFrameEndTime);
  lastFrameEndTime = t1;
}

