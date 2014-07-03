/**
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
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
 * $Id: $
 * @brief This file is the execution node of the Human Tracking
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/image_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <boost/filesystem.hpp>
#include <pcl/gpu/people/label_common.h>
#include <iostream>
#include <fstream>
#include<ctime>
#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

#include <pcl/io/lzf_image_io.h>
namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl::gpu;
using namespace pcl::gpu::people;
using namespace pcl;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//IO variables
 io::LZFDepth16ImageWriter ld;
 io::LZFRGB24ImageWriter lrgb;
string filename_depth="depth_data";
string filename_rgb="rgb_data";
int lzf_fps=8;
//declarations for people detector on ground plane
PointCloudT::Ptr cloud_d (new PointCloudT);
pcl::PointIndices inds;
 bool new_cloud_available_flag_d = false;
// PCL viewer //
pcl::visualization::PCLVisualizer viewer_d("PCL Viewer");
pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
// Mutex: //
boost::mutex cloud_mutex_d;
bool segment_body=false;
bool fill_holes=false;
bool no_people_tracked=false;
bool correct_calibration=true;
bool detected=false;
string lzf_dir_global="";
bool islzf=false;
enum { COLS = 640, ROWS = 480 };
Eigen::Vector3f center_person;

 bool valid_inds[COLS * ROWS];

vector<string> getPcdFilesInDir(const string& directory)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);

  if (!fs::exists(dir) || !fs::is_directory(dir))
    PCL_THROW_EXCEPTION(pcl::IOException, "Wrong PCD directory");

  vector<string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;

  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()) )
      if (fs::extension(*pos) == ".pcd")
        result.push_back(pos->path().string());

  return result;
}

void cloud_cb_2 (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex_d.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
 cloud_mutex_d.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
      cout << "[~SampledScopeTime] : Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
      time_ms_ = 0;
    }
    ++i_;
  }
  private:
  int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string
make_name(int counter, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people%04d_%s.png", counter, suffix);
  return buf;
}

template<typename T> void
savePNGFile(const std::string& filename, const pcl::gpu::DeviceArray2D<T>& arr)
{
  int c;
  pcl::PointCloud<T> cloud(arr.cols(), arr.rows());
  arr.download(cloud.points, c);
  pcl::io::savePNGFile(filename, cloud);
}

template <typename T> void
savePNGFile (const std::string& filename, const pcl::PointCloud<T>& cloud)
{
  pcl::io::savePNGFile(filename, cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PeoplePCDApp
{
  public:
    typedef pcl::gpu::people::PeopleDetector PeopleDetector;
    ofstream skeleton_file;
    enum { COLS = 640, ROWS = 480 };

    PeoplePCDApp (pcl::Grabber& capture, bool write)
      : capture_(capture),
        write_ (write),
        exit_(false),
        time_ms_(0),
        cloud_cb_(true),
        counter_(0),
        final_view_("Final labeling"),
        depth_view_("Depth")

    {
      final_view_.setSize (COLS, ROWS);
      depth_view_.setSize (COLS, ROWS);

      final_view_.setPosition (0, 0);
      depth_view_.setPosition (650, 0);

      cmap_device_.create(ROWS, COLS);
      cmap_host_.points.resize(COLS * ROWS);
      depth_device_.create(ROWS, COLS);
      image_device_.create(ROWS, COLS);

      depth_host_.points.resize(COLS * ROWS);

      rgba_host_.points.resize(COLS * ROWS);
      rgb_host_.resize(COLS * ROWS * 3);

      pcl::gpu::people::uploadColorMap(color_map_);



    }

    int drawLimb(int parent, int child){



   	  Eigen::Vector4f j1=people_detector_.skeleton_joints[parent];
   	 Eigen::Vector4f j2=people_detector_.skeleton_joints[child];
   	  if (j1[0]!=-1 && j2[0]!=-1 &&abs(double(j1[2]))<100.0 && abs(double(j2[2]))<100.0&&abs(double(j1[1]))<100.0 && abs(double(j2[1]))<100.0){
   	  Eigen::Vector3f j_projected_parent=people_detector_.project3dTo2d(j1);
   	Eigen::Vector3f j_projected_child=people_detector_.project3dTo2d(j2);

   	depth_view_.addLine((int)j_projected_parent[0],(int)j_projected_parent[1],(int)j_projected_child[0],(int)j_projected_child[1],"limbs",5000);
   	final_view_.addLine((int)j_projected_parent[0],(int)j_projected_parent[1],(int)j_projected_child[0],(int)j_projected_child[1],"limbs",5000);

depth_view_.addCircle((int)j_projected_parent[0],(int)j_projected_parent[1],10.0,"circle",5000);
 final_view_.addCircle((int)j_projected_parent[0],(int)j_projected_parent[1],10.0,"circle",5000);
depth_view_.addCircle((int)j_projected_child[0],(int)j_projected_child[1],10.0,"circle",5000);
 final_view_.addCircle((int)j_projected_child[0],(int)j_projected_child[1],10.0,"circle",5000);


   	 return 1;
   	  }
   	  return -1;


    }

void drawAllLimbs(){
			int i=0;

			final_view_.removeLayer("limbs");
		    depth_view_.removeLayer("limbs");
	        // Iterate over all parts

		    drawLimb( Neck, FaceRB);
		    drawLimb( Neck, FaceLB);
		    drawLimb( Neck, Rshoulder);
		    drawLimb( Neck, Lshoulder);
		   // drawLimb( Rshoulder, Rarm);
		   // drawLimb( Lshoulder, Larm);
		    drawLimb( Rshoulder, Rchest);
		    drawLimb( Lshoulder, Lchest);
		    drawLimb( Rchest, Rhips);
		   	drawLimb( Lchest, Lhips);
		    drawLimb(Lhips,  Lknee);
		    drawLimb(Rhips,  Rknee);

		    drawLimb(Lknee, Lfoot);
		    drawLimb(Rknee, Rfoot);

		    drawLimb(Rshoulder, Relbow);
	       drawLimb(Relbow, Rhand);
	       drawLimb(Lshoulder, Lelbow);
	       drawLimb(Lelbow, Lhand);
	       drawLimb(FaceLB, FaceLT);

	      drawLimb(FaceRB, FaceRT);



	        }




    void
    visualizeAndWrite()
    {
      const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
      pcl::gpu::people::colorizeLabels(color_map_, labels, cmap_device_);
      //people::colorizeMixedLabels(

      int c;
      cmap_host_.width = cmap_device_.cols();
      cmap_host_.height = cmap_device_.rows();
      cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
      cmap_device_.download(cmap_host_.points, c);
      final_view_.removeLayer("circle");
      depth_view_.removeLayer("circle");
      final_view_.addRGBImage<pcl::RGB>(cmap_host_);
      part_t wanted_joints[]={Rhand,Lhand,Relbow  ,Lelbow };
           int num_joints=sizeof(wanted_joints)/sizeof(wanted_joints)[0];


           for (int i=0; i<27;i++){

         	  Eigen::Vector4f j=people_detector_.skeleton_joints[i];
         	  if (j[0]!=-1){
         	  Eigen::Vector3f j_projected=people_detector_.project3dTo2d(j);
         	// depth_view_.addCircle((int)j_projected[0],(int)j_projected[1],10.0,"circle",5000);
         	// final_view_.addCircle((int)j_projected[0],(int)j_projected[1],10.0,"circle",5000);

         	  }
           }


           drawAllLimbs();
      final_view_.spinOnce(1, true);

      if (cloud_cb_)
      {
        depth_host_.width = people_detector_.depth_device1_.cols();
        depth_host_.height = people_detector_.depth_device1_.rows();
        depth_host_.points.resize(depth_host_.width * depth_host_.height);
        people_detector_.depth_device1_.download(depth_host_.points, c);
      }

      depth_view_.addShortImage(&depth_host_.points[0], depth_host_.width, depth_host_.height, 0, 4000, true);




      depth_view_.spinOnce(1, true);

      if (write_)
      {
        PCL_VERBOSE("PeoplePCDApp::visualizeAndWrite : (I) : Writing to disk");
	//We are not saving the PNG files right now
	/*
        if (cloud_cb_)
          savePNGFile(make_name(counter_, "ii"), cloud_host_);
        else
          savePNGFile(make_name(counter_, "ii"), rgba_host_);
        savePNGFile(make_name(counter_, "c2"), cmap_host_);
        savePNGFile(make_name(counter_, "s2"), labels);
        savePNGFile(make_name(counter_, "d1"), people_detector_.depth_device1_);
        savePNGFile(make_name(counter_, "d2"), people_detector_.depth_device2_);*/
        skeleton_file.open("./skeleton.txt", std::ios::app);
        skeleton_file << "\n"<<counter_<<";"<<time_ms_<<";";
        for (int i=0; i<27;i++){

                	  Eigen::Vector4f j=people_detector_.skeleton_joints[i];
                	  skeleton_file<<"{"<<j[0]<<","<<j[1]<<","<<j[2]<<"};";


                  }
        skeleton_file<<" ";
        skeleton_file.close();
      }
    }

    void source_cb1(const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {

      {
        boost::mutex::scoped_lock lock(data_ready_mutex_);
        if (exit_)
          return;
	
        pcl::copyPointCloud(*cloud, cloud_host_);
	


      }
      data_ready_cond_.notify_one();
    }













    void source_cb2(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
    {
      {
        boost::mutex::scoped_try_lock lock(data_ready_mutex_);

        if (exit_ || !lock)
          return;

        //getting depth

        int w = depth_wrapper->getWidth();
        int h = depth_wrapper->getHeight();

        int s = w * PeopleDetector::Depth::elem_size;

        depth_host_.points.resize(w *h);
        depth_host_.width = w;
        depth_host_.height = h;

        const unsigned short *data = depth_wrapper->getDepthMetaData().Data();
         unsigned short data2[w * h];
	unsigned short depth_invalid_value=10000;
         std::copy(data, data + w * h, &data2[0]);
       pcl::PointCloud<unsigned short> *depth_host_backup(new pcl::PointCloud<unsigned short>);
        depth_host_backup->points.resize(w *h);
        depth_host_backup->width = w;
        depth_host_backup->height = h;
	if (no_people_tracked==true){
	//we assume that depth_host_ contains the old values


		for (int i=0; i<depth_host_.points.size();i++){
			depth_host_backup->points[i]=depth_host_.points[i];
			if (segment_body==true){
				if(depth_host_.points[i]==depth_invalid_value)
				data2[i]=depth_invalid_value;

			}


		}

	}

        std::copy(data2, data2 + w * h, &depth_host_.points[0]);
       // std::copy(data2, data2 + w * h, &depth_host_backup->points[0]);

        depth_host_.points.resize(w *h);
               depth_host_.width = w;
               depth_host_.height = h;
       

       //if we want to use the body cluster

	if (no_people_tracked==false){
        for (int i=0; i<depth_host_.points.size();i++){
        	depth_host_backup->points[i]=depth_host_.points[i];
		if (segment_body==true){
			depth_host_.points[i]=depth_invalid_value;
			data2[i]=depth_invalid_value;
			if (detected==true)
				valid_inds[i]=false;
			}


        	}

 	Eigen::Vector4f min;
       Eigen::Vector4f max;
       pcl::getMinMax3D(*people_detector.getNoGroundCloud(),inds.indices, min, max);
       Eigen::Vector3f min_2d=people_detector_.project3dTo2d(min);
       Eigen::Vector3f max_2d=people_detector_.project3dTo2d(max);
	float mean_x=(min_2d[0]+max_2d[0])/2.0;
	float mean_y=(min_2d[1]+max_2d[1])/2.0;
	float mean_depth=0.0;
	int sum_depth=0;

	int max_y=0;
/*
	if (segment_body==true){
	if (detected==true){

        for (int i=0; i<inds.indices.size();i++){


        	pcl::PointXYZRGBA point = people_detector.getNoGroundCloud()->points[inds.indices[i]];
        	float x=point.x;
        	float y=point.y;
        	float z=point.z;

        	Eigen::Vector4f temp;
        	temp[0]=x;
        	temp[1]=y;
        	temp[2]=z;
        	temp[3]=1;

        	Eigen::Vector3f temp2=people_detector_.project3dTo2d(temp);

		if (temp2[1]>max_y)
			max_y=(int)temp2[1];
		

        	//setting the corresponding values
        	if (((int)temp2[1])<ROWS && ((int)temp2[0])<COLS && ((int)temp2[1])>0 && ((int)temp2[0])>0){
		valid_inds[((int)temp2[1])*w+((int)temp2[0])]=true;
        	// depth_host_.points[((int)temp2[1])*w+((int)temp2[0])]=depth_host_backup->points[((int)temp2[1])*w+((int)temp2[0])];
             //data2[((int)temp2[1])*w+((int)temp2[0])]=depth_host_backup->points[((int)temp2[1])*w+((int)temp2[0])];
        	}

        	//filling i all +-10 values

float mean_x=(min_2d[0]+max_2d[0])/2.0;



		int min_j=-20;
		int max_j=20;
		if ((int)temp2[0]<mean_x)
			min_j=5;
		if ((int)temp2[0]>mean_x)
			max_j=5;

	mean_depth+=depth_host_backup->points[((int)temp2[1])*w+((int)temp2[0])];
	sum_depth++;
        	for (int j=-10; j<10; j++){
        		for (int k=-10; k<10;k++){//cols

        			if (((int)temp2[1]+k)<ROWS && ((int)temp2[0])+j<COLS && ((int)temp2[1]+k)>0 && ((int)temp2[0])+j>0){

        			//depth_host_.points[((int)temp2[1]+k)*w+((int)temp2[0])+j]=10000;
                	//data2[((int)temp2[1]+k)*w+((int)temp2[0])+j]=10000;
			valid_inds[((int)temp2[1]+k)*w+((int)temp2[0])+j]=true;
                	//depth_host_.points[((int)temp2[1]+k)*w+((int)temp2[0])+j]=depth_host_backup->points[((int)temp2[1]+k)*w+((int)temp2[0])+j];
                	//data2[((int)temp2[1]+k)*w+((int)temp2[0])+j]=depth_host_backup->points[((int)temp2[1]+k)*w+((int)temp2[0])+j];
        			}


        		}


        	}//end filling +-10
}


                	//depth_host_.points[((int)temp2[1])*w+((int)temp2[0])]=depth_host_backup->points[((int)temp2[1])*w+((int)temp2[0])];
                	//data2[((int)temp2[1])*w+((int)temp2[0])]=depth_host_backup->points[((int)temp2[1])*w+((int)temp2[0])];


                }
}




*/


 //if we want to use the bounding box

      // Eigen::Vector4f min;
      // Eigen::Vector4f max;
       pcl::getMinMax3D(*people_detector.getNoGroundCloud(),inds.indices, min, max);
     //  Eigen::Vector3f min_2d=people_detector_.project3dTo2d(min);
     //  Eigen::Vector3f max_2d=people_detector_.project3dTo2d(max);

       std::cout << "MIN " << min_2d<<std::endl;
       std::cout << "MAX " << max_2d<<std::endl;
max_y=max_2d[1];
       for (long i=0; i<depth_host_.points.size();i++){



		   long y=i/w;
		   long x=i-y*w;
		   if(x>min_2d[0] && y>min_2d[1] && x<max_2d[0] && y<max_2d[1]){
			   valid_inds[i]=true;
			mean_depth+=depth_host_backup->points[i];
			sum_depth++;
		   }

       }




mean_depth=mean_depth/sum_depth;
if(detected){
float depth_thresh=10;
for (int i=0; i<ROWS*COLS;i++){

	if (valid_inds[i]==true && !(depth_host_backup->points[i]-mean_depth<depth_thresh)){
		valid_inds[i]=false;
		
		
	}

}
}

for (int i=0; i<ROWS*COLS;i++){

	if (valid_inds[i]==true){
		depth_host_.points[i]=depth_host_backup->points[i];
		data2[i]=depth_host_backup->points[i];
		
	}

}


if(fill_holes==true  ){
        //closing the gaps: rows
/*
        for (int i=0; i<h;i++){
        	int first=-1;
        	int last=0;
        	for (int j=0; j<w-1;j++){
        		if ((data2[i*w+j])!=depth_invalid_value){
        			if (first==-1){
        				first=j;
        			}
        			last=j;
        		}

        	}
        	//filling all the indicies between first and last in the current row
        	for (int j=first; j<last&&first!=-1;j++){

        		 depth_host_.points[i*w+j]=depth_host_backup->points[i*w+j];
        		 data2[i*w+j]=depth_host_backup->points[i*w+j];



        	}
        }
*/
        //closing the gaps: cols
        for (int i=0; i<w;i++){
        	int first=0;
        	int last=0;
        	for (int j=0; j<h;j++){
        		if ((data2[j*w+i])!=depth_invalid_value){
        			if (first==0){
        				first=j;
        			}
        			last=j;
        		}

        	}

 //std::cout <<"max_y"<< max_y<< std::endl;
		first=last;
		if (max_y!=0 && max_y-last<10)
			if  (last+25<h-1)
				last=last+25;
			else
				last=h-2;
			
		
        	//filling all the indicies between first and last in the current column
        	for (int j=first; j<last;j++){

        		 depth_host_.points[j*w+i]=depth_host_backup->points[j*w+i];
        		 data2[j*w+i]=depth_host_backup->points[j*w+i];



        	}
        }
}

}

        //END of "if we want to use the body cluster"

        //std::copy(&depth_host_.points[0], &depth_host_.points[0] + w * h, data2);
        //unsigned short data3[w*h];
        //std::copy(&data2[0], &data2[0] + w * h, &data3[0]);
        depth_device_.upload(data2, s, h, w);



        //std::copy(data, data + w * h, &depth_host_.points[0]);

        //getting image
        w = image_wrapper->getWidth();
        h = image_wrapper->getHeight();
        s = w * PeopleDetector::Image::elem_size;

        //fill rgb array
        rgb_host_.resize(w * h * 3);
        image_wrapper->fillRGB(w, h, (unsigned char*)&rgb_host_[0]);

        // convert to rgba, TODO image_wrapper should be updated to support rgba directly
        rgba_host_.points.resize(w * h);
        rgba_host_.width = w;
        rgba_host_.height = h;
        for(int i = 0; i < rgba_host_.size(); ++i)
        {
          const unsigned char *pixel = &rgb_host_[i * 3];
	RGB& rgba = rgba_host_.points[i];
	if (correct_calibration && (i%w-8)>0){
          rgba = rgba_host_.points[i-8];
	}
	
	 
          rgba.r = pixel[0];
          rgba.g = pixel[1];
          rgba.b = pixel[2];
        }
        image_device_.upload(&rgba_host_.points[0], s, h, w);
      }
      data_ready_cond_.notify_one();
    }



  void source_cb3(const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {

      {
        boost::mutex::scoped_lock lock(data_ready_mutex_);
        if (exit_)
          return;
	
        pcl::copyPointCloud(*cloud_d, cloud_host_);
	pcl::copyPointCloud(*cloud_d,rgba_host_);
      
 for(size_t i = 0; i < cloud->points.size(); ++i)
  {
   
    depth_host_.points[i] =static_cast<unsigned short>(cloud_host_.points[i].z * 1000); //m -> mm
  }


	int h=ROWS;
	int w=COLS;
	unsigned short depth_invalid_value=10000;

	//if a person was tracked 
	if (detected){
		for (int i=0; i<depth_host_.points.size();i++){
				//depth_host_.points[i]=depth_invalid_value;
				valid_inds[i]=false;

        	}
	}


	Eigen::Vector4f min;
       Eigen::Vector4f max;
       pcl::getMinMax3D(*people_detector.getNoGroundCloud(),inds.indices, min, max);
       Eigen::Vector3f min_2d=people_detector_.project3dTo2d(min);
       Eigen::Vector3f max_2d=people_detector_.project3dTo2d(max);
	float mean_x=(min_2d[0]+max_2d[0])/2.0;
	float mean_y=(min_2d[1]+max_2d[1])/2.0;
	float mean_depth=0.0;
	int sum_depth=0;
	int max_y=0;


        for (int i=0; i<inds.indices.size() && detected;i++){//going through all valid indicies and filling vlid_inds if a person was detected


        	pcl::PointXYZRGBA point = people_detector.getNoGroundCloud()->points[inds.indices[i]];
        	float x=point.x;
        	float y=point.y;
        	float z=point.z;

        	Eigen::Vector4f temp;
        	temp[0]=x;
        	temp[1]=y;
        	temp[2]=z;
        	temp[3]=1;

        	Eigen::Vector3f projected=people_detector_.project3dTo2d(temp);


		//estimating the highest Y-value to add points to the legs
		if (projected[1]>max_y)
			max_y=(int)projected[1];
		

        	//setting the corresponding values
        	if (((int)projected[1])<ROWS && ((int)projected[0])<COLS && ((int)projected[1])>0 && ((int)projected[0])>0){
		valid_inds[((int)projected[1])*w+((int)projected[0])]=true;
        	}

        	//filling all +-10 values

		float mean_x=(min_2d[0]+max_2d[0])/2.0;
		int min_j=-20;
		int max_j=20;
		if ((int)projected[0]<mean_x)
			min_j=5;
		if ((int)projected[0]>mean_x)
			max_j=5;

		//for  depth filtering
		mean_depth+=depth_host_.points[((int)projected[1])*w+((int)projected[0])];
		sum_depth++;

        	for (int j=-10; j<10; j++){
        		for (int k=-10; k<10;k++){

        			if (((int)projected[1]+k)<ROWS && ((int)projected[0])+j<COLS && ((int)projected[1]+k)>0 && ((int)projected[0])+j>0){
				valid_inds[((int)projected[1]+k)*w+((int)projected[0])+j]=true;

        			}
			}


         
		}//end filling +-10
	}//end of filling valid_inds





	mean_depth=mean_depth/sum_depth;

	//Extending the legs
        for (int i=0; i<w;i++){
        	int first=0;
        	int last=0;
        	for (int j=0; j<h;j++){
        		if ((valid_inds[j*w+i])==true){
        			if (first==0){
        				first=j;
        			}
        			last=j;
        		}

        	}

 
		first=last;
		if (max_y!=0 && max_y-last<10)
			if  (last+25<h-1)
				last=last+25;
			else
				last=h-2;
			
		
        	//filling all the indicies between first and last in the current column
        	for (int j=first; j<last;j++)
        		 valid_inds[j*w+i]=true;

}




	if(detected){
	float depth_thresh=200;
	for (int i=0; i<ROWS*COLS;i++){

		if (valid_inds[i]==false || !(depth_host_.points[i]-mean_depth<depth_thresh))
			depth_host_.points[i]=depth_invalid_value;

			
		}
	}


	//uploading the data
	int s = w * PeopleDetector::Depth::elem_size;
		depth_device_.upload(&depth_host_.points[0], s, h, w);
	s = w * PeopleDetector::Image::elem_size;
		image_device_.upload(&rgba_host_.points[0], s, h, w);


      }
      data_ready_cond_.notify_one();
    }
















    void
    startMainLoop ()
    {


    	//--------People detection------



    	  // Algorithm parameters:
    	 // std::string svm_filename = "/home/roitberg/workspace/pcl/gpu/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
    	  std::string svm_filename = "/home/alina/workspace/pcl/gpu/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";

    	  float min_confidence = -2.0;
    	  float min_height = 0.9;
    	  float max_height = 2.3;
    	  //float min_height = 0.3;
    	  //float max_height = 2.3;
    	  float voxel_size = 0.06;

    	  float sampling_factor = 1;
    	  Eigen::Matrix3f rgb_intrinsics_matrix;
    	  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics



    	  // Read Kinect live stream:


    	  pcl::Grabber* interface = new pcl::OpenNIGrabber();
	if( islzf)
	  interface = new pcl::ImageGrabber<PointXYZRGBA>(lzf_dir_global,lzf_fps, false, true) ;
 
    	  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    	  boost::bind (&cloud_cb_2, _1, cloud_d, &new_cloud_available_flag_d);
    	  interface->registerCallback (f);
    	  interface->start ();

    	  // Wait for the first frame:
    	  while(!new_cloud_available_flag_d)
    	    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    	  new_cloud_available_flag_d = false;

    	  cloud_mutex_d.lock ();    // for not overwriting the point cloud

    	  // Display pointcloud:
    	  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_d);
    	  viewer_d.addPointCloud<PointT> (cloud_d, rgb, "input_cloud");
    	  viewer_d.setCameraPosition(0,0,-2,0,-1,0,0);

    	  // Add point picking callback to viewer:
    	  struct callback_args cb_args;
    	  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    	  cb_args.clicked_points_3d = clicked_points_3d;
    	  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_d);
    	  viewer_d.registerPointPickingCallback (pp_callback, (void*)&cb_args);
    	  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    	  // Spin until 'Q' is pressed:
    	  viewer_d.spin();
    	  std::cout << "done." << std::endl;

    	  cloud_mutex_d.unlock ();

    	  // Ground plane estimation:
    	  Eigen::VectorXf ground_coeffs;
    	  ground_coeffs.resize(4);
    	  std::vector<int> clicked_points_indices;
    	  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    	    clicked_points_indices.push_back(i);
    	  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    	  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
    	  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

    	  viewer_d.close();

    	  // Initialize new viewer:
    	  pcl::visualization::PCLVisualizer viewer_d("PCL Viewer");          // viewer initialization
    	  viewer_d.setCameraPosition(0,0,-2,0,-1,0,0);


    	  // Create classifier for people detection:
    	  pcl::people::PersonClassifier<pcl::RGB> person_classifier;

    	  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

    	  // People detection app initialization:

    	  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
    	  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    	  people_detector.setClassifier(person_classifier);                // set person classifier
    	  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
    	  people_detector.setSamplingFactor(sampling_factor);              // set a downsampling factor to the point cloud (for increasing speed)

    	  people_detector.setMinimumDistanceBetweenHeads(1.0);
    	  //-----------end of people detection-----------


      cloud_cb_ = false;
	
      PCDGrabberBase* ispcd = dynamic_cast<pcl::PCDGrabberBase*>(&capture_);

      if (ispcd || islzf)
        cloud_cb_= true;
	
      typedef boost::shared_ptr<openni_wrapper::DepthImage> DepthImagePtr;
      typedef boost::shared_ptr<openni_wrapper::Image> ImagePtr;

      boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func1 = boost::bind (&PeoplePCDApp::source_cb1, this, _1);
      boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func2 = boost::bind (&PeoplePCDApp::source_cb2, this, _1, _2, _3);

	boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func3 = boost::bind (&PeoplePCDApp::source_cb3, this, _1);
      

      boost::signals2::connection c = cloud_cb_ ? capture_.registerCallback (func3) : capture_.registerCallback (func2);

      {
        boost::unique_lock<boost::mutex> lock(data_ready_mutex_);

        try
        {
          capture_.start ();
          while (!exit_ && !final_view_.wasStopped())
          {

        	  //PEOPLE DETECTION


      	    if (new_cloud_available_flag_d && cloud_mutex_d.try_lock ())    // if a new cloud is available
      	    {
      	      new_cloud_available_flag_d = false;

      	      // Perform people detection on the new cloud:
      	      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      	      people_detector.setInputCloud(cloud_d);
      	      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      	      people_detector.compute(clusters);                           // perform people detection

      	      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients
      	      viewer_d.removeAllPointClouds();
      	      viewer_d.removeAllShapes();
      	      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_d);
      	      viewer_d.addPointCloud<PointT> (cloud_d, rgb, "input_cloud");


      	  
      	      unsigned int k = 0;


      	  float conf=-1000.0f;

      	std::vector<pcl::people::PersonCluster<PointT> >::iterator it_maxconf=clusters.begin();
      	int k_maxconf=-1;
inds.indices.clear();
      	      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      	      {


      	    	  //selecting the cluster with maximum confidence

      	        	//
      	        	if(it->getPersonConfidence() >=conf){
      	        		conf=it->getPersonConfidence();
      	        		//inds=it->getIndices();
      	        		it_maxconf=it;
      	        		k_maxconf=k;
      	        	}

      	        	if(it->getPersonConfidence() >min_confidence)
      	        	 k++;

      	      }
      	    if(conf >min_confidence ){

      	    	it_maxconf->drawTBoundingBox(viewer_d, k_maxconf);
      	    	inds=it_maxconf->getIndices();
}
	detected=(k>0);

      	    viewer_d.spinOnce();

      	   // pcl::PointIndices *indss ;
      	  //people_detector.getGrountPlaneInds(indss);

		//no_people_tracked=(k<1);


      	      std::cout << k << " people found" << std::endl;
      	      //viewer.spinOnce();


      	      cloud_mutex_d.unlock ();

      	    std::cout << k << people_detector.getNoGroundCloud()->height<< people_detector.getNoGroundCloud()->width << std::endl;
      	 // const pcl::PointCloud::ConstPtr &cloud3=people_detector.getNoGroundCloud()->makeShared();
      	  pcl::people::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr cl3=people_detector.getNoGroundCloud();
      	  cl3->points.resize(COLS*ROWS);
      	std::cout  << " cl3->size(): " << cl3->size()<<std::endl;
      	std::cout  << " cloud_d->size()" <<cloud_d->size()<< std::endl;

   	    }



      	    //NORMAl

            bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
            if(has_data)
            {
              SampledScopeTime fps(time_ms_);

              if (cloud_cb_ and false)
                process_return_ = people_detector_.process(cloud_host_.makeShared());
              else
              {

                process_return_ = people_detector_.process(depth_device_, image_device_);

              }
              ++counter_;
            }

            if(has_data ){
             visualizeAndWrite();
            } //visualizeAndWrite();

           }


          final_view_.spinOnce (3);
        }
        catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
        catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

        capture_.stop ();
      }
      c.disconnect();
    }

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    pcl::Grabber& capture_;

    bool cloud_cb_;
    bool write_;
    bool exit_;
    int time_ms_;
    int counter_;
    int process_return_;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;
    pcl::PointCloud<pcl::RGB> cmap_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;

    pcl::PointCloud<unsigned short> depth_host_;
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGBA> cloud_host_;

    ImageViewer final_view_;
    ImageViewer depth_view_;

    DeviceArray<pcl::RGB> color_map_;
};

void print_help()
{
  cout << "\nPeople tracking app options (help):" << endl;
  cout << "\t -numTrees    \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0       \t<path_to_tree_file>" << endl;
  cout << "\t -tree1       \t<path_to_tree_file>" << endl;
  cout << "\t -tree2       \t<path_to_tree_file>" << endl;
  cout << "\t -tree3       \t<path_to_tree_file>" << endl;
  cout << "\t -gpu         \t<GPU_device_id>" << endl;
  cout << "\t -w           \t<bool> \tWrite results to disk" << endl;
  cout << "\t -h           \tPrint this help" << endl;
  cout << "\t -dev         \t<Kinect_device_id>" << endl;
  cout << "\t -pcd         \t<path_to_pcd_file>" << endl;
  cout << "\t -oni         \t<path_to_oni_file>" << endl;
  cout << "\t -pcd_folder  \t<path_to_folder_with_pcd_files>" << endl;
}

int main(int argc, char** argv)
{
  // answering for help
  PCL_INFO("People tracking App version 0.2\n");
  if(pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_help(), 0;

  // selecting GPU and prining info
  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  bool write = 0;
  pc::parse_argument (argc, argv, "-w", write);


  // selecting data source
  boost::shared_ptr<pcl::Grabber> capture;
  string openni_device, oni_file, pcd_file, pcd_folder,lzf_dir;

  try
  {
    if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
    {
      capture.reset( new pcl::OpenNIGrabber(openni_device) );
    }
    else
    if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
    {
      capture.reset( new pcl::ONIGrabber(oni_file, true, true) );
    }
    else
    if (pc::parse_argument (argc, argv, "-lzf", lzf_dir) > 0)
    {
      capture.reset( new pcl::ImageGrabber<PointXYZRGBA>(lzf_dir,lzf_fps, false, true) );
	lzf_dir_global=lzf_dir;
	islzf=true;
    }
    else
    if (pc::parse_argument (argc, argv, "-pcd", pcd_file) > 0)
    {
      capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(vector<string>(31, pcd_file), 30, true) );
    }
    else
    if (pc::parse_argument (argc, argv, "-pcd_folder", pcd_folder) > 0)
    {
      vector<string> pcd_files = getPcdFilesInDir(pcd_folder);
      capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(pcd_files, 30, true) );
    }
    else
    {
      capture.reset( new pcl::OpenNIGrabber() );
      //capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224932.oni", true, true) );

      //vector<string> pcd_files(31, "d:/3/0008.pcd");
      //vector<string> pcd_files(31, "d:/git/pcl/gpu/people/tools/test.pcd");
      //vector<string> pcd_files = getPcdFilesInDir("d:/3/");
      //capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(pcd_files, 30, true) );
    }
  }
  catch (const pcl::PCLException& /*e*/) { return cout << "Can't open depth source" << endl, -1; }

  //selecting tree files
  vector<string> tree_files;
  tree_files.push_back("Data/forest1/tree_20.txt");
  tree_files.push_back("Data/forest2/tree_20.txt");
  tree_files.push_back("Data/forest3/tree_20.txt");
  tree_files.push_back("Data/forest4/tree_20.txt");

  pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
  pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
  pc::parse_argument (argc, argv, "-tree2", tree_files[2]);
  pc::parse_argument (argc, argv, "-tree3", tree_files[3]);

  int num_trees = (int)tree_files.size();
  pc::parse_argument (argc, argv, "-numTrees", num_trees);

  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 4)
  {
    PCL_ERROR("[Main] : Invalid number of trees");
    print_help();
    return -1;
  }

  try
  {
    // loading trees
    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
    PCL_VERBOSE("[Main] : Loaded files into rdf");

    // Create the app
    PeoplePCDApp app(*capture, write);
    app.people_detector_.rdf_detector_ = rdf;

    // executing
    app.startMainLoop ();
  }
  catch (const pcl::PCLException& e) { cout << "PCLException: " << e.detailedMessage() << endl; print_help(); }
  catch (const std::runtime_error& e) { cout << e.what() << endl; print_help(); }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; print_help(); }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; print_help(); }

  return 0;
}


