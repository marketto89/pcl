#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/io/grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/people/person_classifier.h>
#include <pcl/people/ground_based_people_detection_app.h>

#include <thread>
#include <chrono>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::gpu::people::PeopleDetector PeopleDetector;

namespace pc = pcl::console;

bool exittt = false;

float min_confidence = -2.0;
float min_height = 0.6;
float max_height = 1.9;
float voxel_size = 0.06;

float sampling_factor = 1;
Eigen::Matrix3f rgb_intrinsics_matrix;

pcl::visualization::PCLVisualizer viewer;
boost::mutex cloud_mutex;

void callback(const PointCloudT::ConstPtr &cloud,
              PointCloudT::Ptr& cloud_out,
              bool* cloud_flag)
{
    std::cout << "Callback" << std::endl;
    cloud_mutex.lock();
    pcl::copyPointCloud(*cloud, *cloud_out);
    *cloud_flag = true;
    cloud_mutex.unlock();
    // Display pointcloud:
    //    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb (cloud);
    //    viewer.addPointCloud<PointT>(cloud, rgb, "cloud");
    //    viewer.setCameraPosition(0,0,-2,0,-1,0,0);
    //    viewer.spin();
}

void
getPcdFilesInDir (const std::string& directory, std::vector<std::string>& pcd_files)
{
    namespace fs = boost::filesystem;
    fs::path dir (directory);

    if (!fs::exists (dir) || !fs::is_directory (dir))
        PCL_THROW_EXCEPTION(pcl::IOException, "Wrong PCD directory");

    fs::directory_iterator pos (dir);
    fs::directory_iterator end;

    for (; pos != end; ++pos)
        if (fs::is_regular_file (pos->status ()))
            if (fs::extension (*pos) == ".pcd")
                pcd_files.push_back (pos->path ().string ());
}


void
print_help ()
{
    std::cout << "\nPeople tracking app options (help):" << std::endl;
    std::cout << "\t -numTrees    \t<int> \tnumber of trees to load" << std::endl;
    std::cout << "\t -tree0       \t<path_to_tree_file>" << std::endl;
    std::cout << "\t -tree1       \t<path_to_tree_file>" << std::endl;
    std::cout << "\t -tree2       \t<path_to_tree_file>" << std::endl;
    std::cout << "\t -tree3       \t<path_to_tree_file>" << std::endl;
    std::cout << "\t -gpu         \t<GPU_device_id>" << std::endl;
    std::cout << "\t -w           \t<bool> \tWrite the results of skeleton tracking in skeleton.txt" << std::endl;
    std::cout << "\t -h           \tPrint this help" << std::endl;
    std::cout << "\t -dev         \t<Kinect_device_id>" << std::endl;
    std::cout << "\t -pcd         \t<path_to_pcd_file>" << std::endl;
    std::cout << "\t -oni         \t<path_to_oni_file>" << std::endl;
    std::cout << "\t -pcd_folder  \t<path_to_folder_with_pcd_files>" << std::endl;
    std::cout << "\t -tracking           \t<bool> activate the skeleton tracking" << std::endl;
    std::cout << "\t -alpha    \t<float> \tset tracking parameter" << std::endl;
    std::cout << "\t -beta    \t<float> \tset tracking parameter" << std::endl;
    std::cout << "\t -lzf        \t<path_to_folder_with_pclzf_files>" << std::endl;
    std::cout << "\t -lzf_fps    \t<int> \tfps for replaying the lzf-files (default: 10)" << std::endl;
    std::cout << "\t -segment_people       \t<path_to_svm_file> \t activates body segmentation" << std::endl;

}

int main(int argc, char** argv)
{
    // answering for help
    PCL_INFO("People tracking App version 0.2\n");
    if (pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
        return print_help (), 0;

    // selecting GPU and prining info
    int device = 0;
    pc::parse_argument (argc, argv, "-gpu", device);
    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);

    // selecting data source
    std::shared_ptr<pcl::Grabber> capture;
    std::string pcd_folder;
    std::vector<std::string> pcd_filenames;

    try
    {
        if (pc::parse_argument (argc, argv, "-pcd_folder", pcd_folder) > 0)
        {
            getPcdFilesInDir (pcd_folder, pcd_filenames);
            std::sort(pcd_filenames.begin(), pcd_filenames.end());
            capture.reset (new pcl::PCDGrabber<PointT> (pcd_filenames,
                                                        1,
                                                        true));
        }
        else
        {
            return std::cout << "WRONG NUMBER OF ARGUMENTS, PLEASE SEE "
                                "THE HELPER\n\n",
                    print_help(), 0;
        }
    }
    catch (const pcl::PCLException& /*e*/)
    {
        return std::cout << "Can't open depth source" << std::endl, -1;
    }
    //selecting tree files
    std::vector<std::string> tree_files(3);
    pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
    pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
    pc::parse_argument (argc, argv, "-tree2", tree_files[2]);
    //pc::parse_argument (argc, argv, "-tree3", tree_files[3]);

    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf (new RDFBodyPartsDetector (tree_files));
    PeopleDetector people_detector;
    PeopleDetector::Depth depth_device;
    PeopleDetector::Image image_device;
    pcl::PointCloud<unsigned short> depth_host;
    pcl::PointCloud<pcl::RGB> rgba_host;

    people_detector.rdf_detector_ = rdf;
    float alpha, beta;
    bool tracking;
    std::string svm_filename;
    if (pc::parse_argument (argc, argv, "-alpha", alpha) > 0)
    {
        people_detector.setAlphaTracking (alpha);
    }
    if (pc::parse_argument (argc, argv, "-beta", beta) > 0)
    {
        people_detector.setBetaTracking (beta);
    }
    if (pc::parse_argument (argc, argv, "-tracking", tracking) > 0)
    {
        people_detector.setActiveTracking (tracking);
    }

    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    pcl::people::GroundBasedPeopleDetectionApp<PointT>
            people_detector_ground_plane;

    pc::parse_argument(argc, argv, "-svm", svm_filename);
    person_classifier.loadSVMFromFile (svm_filename);
    people_detector_ground_plane.setVoxelSize (voxel_size);
    rgb_intrinsics_matrix << 525, 0.0, 319.5,
            0.0, 525, 239.5,
            0.0, 0.0, 1.0;
    people_detector_ground_plane.setIntrinsics (rgb_intrinsics_matrix);
    people_detector_ground_plane.setClassifier (person_classifier);
    people_detector_ground_plane.setHeightLimits (min_height, max_height);
    people_detector_ground_plane.setSamplingFactor (sampling_factor);
    people_detector_ground_plane.setMinimumDistanceBetweenHeads (1.0);

    bool new_cloud_flag = false;
    PointCloudT::Ptr cloud(new PointCloudT);

    boost::function<void
            (const PointCloudT::ConstPtr&)> f =
            boost::bind (&callback, _1, cloud, &new_cloud_flag);
    capture->registerCallback (f);
    capture->start();



    while(!viewer.wasStopped())
    {
        while (!new_cloud_flag)
            boost::this_thread::sleep (boost::posix_time::milliseconds (1));
        new_cloud_flag = false;
        std::cout << "New cloud!" << std::endl;
        if(cloud_mutex.try_lock())
        {
            // Perform people detection on the new cloud:
            std::vector<pcl::people::PersonCluster<PointT> > clusters;
            viewer.removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb
                    (cloud);
            viewer.addPointCloud<PointT>(cloud, rgb, "cloud");
            viewer.setCameraPosition(-1,-1,-2,-1,-1,0,0);
            Eigen::Vector4f plane();
            pcl::ModelCoefficients plane_coeff;
            plane_coeff.values.resize (4);    // We need 4 values
            plane_coeff.values[0] = -0.0255749;
            plane_coeff.values[1] = 0.983198;
            plane_coeff.values[2] = 0.180739;
            plane_coeff.values[3] = -1.24205;
            viewer.addPlane(plane_coeff);
            viewer.spin();
            cloud_mutex.unlock();
        }
    }
    return 0;
}
