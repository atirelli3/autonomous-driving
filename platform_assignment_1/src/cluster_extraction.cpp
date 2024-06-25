#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"

/*
CMake 17
*/
#include <filesystem>
namespace fs = std::filesystem;


// Define the PCL Library (it will be instantiated in the HEAP memory).
#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

// Setup KDTree.
//
// This function sets up the custom KDTreee using a give point cloud.
//
// Parameters:
//
//      - [cloud]: point cloud to be insert in the KDTree
//
//      - [tree]: KD Tree to interate and insert each point of the input cloud.
//
//      - [dimension]: dimension of the input cloud.
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension) {
    // insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back
	//for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
	return clusters;	
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /*
        1) D O W N S A M P L E  T H E  D A T A S E T
        
        Step:
            + Create filtering objcet.
            + Create cloud object to host the filtered cloud.
            + Give the input cloud to the filterging object.
            + Downsample the dataset using a Xcm of leaf size.
            + Apply the filter and save it.
    */
    // Filtering object.
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    // Host the original cloud but filtered.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Set the cloud to be filtered.
    vg.setInputCloud(cloud);
    // Downsample the dataset by 1dm leaf size.
    vg.setLeafSize(0.08f, 0.08f, 0.08f); //Set the voxel grid
    // Apply the filter and store it in the host cloud.
    vg.filter(*cloud_filtered);
    // [DEBUG] output.
    std::cout<<"PointCloud after filtering has: " << cloud_filtered->size()<<" data points."<< std::endl;


    /*
        2) C R O P  P O I N T S
    */
    // Crop the points that are far away from us.
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 


    /*
        3) S E G M E N T A T I O N  &&  R A N S A C

        Step:
            + Create segmentation object.
            + Create cloud object to host the segmented cloud.
            + Set all the param for the segmentation obejct.
    */
    // Segmentation object.
    pcl::SACSegmentation<pcl::PointXYZ>seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Host the original cloud but segmented.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    
    // pcl::PCDWriter writer;
    // Set all the param for the segmentation object.
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.4);

    
    /*
        4) R E M O V E  T H E  P L A N A R  I N L I E R S

        Step:
            + Iterate until there are more than ground plane.
            Loop:
                > Segment the largest planar component from the remaining cloud.
                > Extract the planar inliers from the input cloud.
                > Get the points associated with the planar surface.
                > Remove the planar inliers, extract the rest.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    int i=0, nr_points = (int) cloud_filtered->size();
    // While there are more than ground plane.
    while(cloud_filtered->size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud.
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        // Error condition.
        if(inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud.
        pcl::ExtractIndices<pcl::PointXYZ>extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface.
        extract.filter(*cloud_plane);
        std::cout<<"PointCloud representing the planar component: "<<cloud_plane->size()<<" data points."<< std::endl;

        // Remove the planar inliers, extract the rest.
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    
    // Vector of PointIndices, which contain the actual index information in a vector<int>. 
    // The indices of each detected cluster are saved here.
    std::vector<pcl::PointIndices> cluster_indices;

    #ifdef USE_PCL_LIBRARY
        /*
            5) K D T r e e  & &  P o i n t I n d i c e s

            Step:
                + Create KdTree object.
                + Populate the KdTree with the necessary point cloud.
                + Create the EuclideanCluster object.
                + Create the vector of PointIndices. (already done, for bypass the namespace
                    in the ifdef statement)
        */
        // KdTree object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // Polutate KdTree, in this case we are using our [cloud_filtered] object.
        // This because [cloud_filtered] contain the initial cloud + filtering + segmentation.
        tree->setInputCloud(cloud_filtered);
        // EuclideanCluster object.
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        /*
            6) S P A T I A L  T O L E R A N C E

            Step:
                + Set the spatial tolereance, by Xm (meter, e.g. 2cm => 0.02)
                + Set min and max points for the cluster.
                + Give the data structure where search.
                + Give the cloud dataset.
                + Save the detected cluster in the vector of PointIndices.
        */
        // Set the spatial tolerance for new cluster candidates.
        ec.setClusterTolerance(0.3); // 1dm
        // We impose that the clusters found must have at least setMinClusterSize() 
        // points and maximum setMaxClusterSize() points.
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        // Search on the previous created [tree].
        ec.setSearchMethod(tree);
        // Dataset [cloud] to work.
        ec.setInputCloud(cloud_filtered);

        // The indices of each detected cluster are saved here.
        ec.extract(cluster_indices);
    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(0,1,0)};


    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        renderer.RenderPointCloud(cloud_cluster, "clusterCloud"+std::to_string(clusterId), colors[2]);
        renderer.RenderPointCloud(cloud_plane, "planeCloud"+std::to_string(clusterId), colors[5]);
        //----------

        // Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z};
        std::cout<<"The veichle J = "<<j<<" - position ("<<(minPt.x + maxPt.x) / 2<<", "<<(minPt.y + maxPt.y) / 2<<", "<<(minPt.z + maxPt.z) / 2<<")."<<endl;
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        if (box.x_min < 5 && box.x_max> -5) {
            std::cout<<"@@@ Veichle J = "<<j<<" in range!"<<endl;
            renderer.RenderBox(box, j, colors[0]);
        } else {
            renderer.RenderBox(box, j, colors[4]);
        }
        

        ++clusterId;
        j++;
    }  

}

/*
std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/home/andreatirelli/Developments/platform/perception/assignment_1/dataset_1/"},
                                             boost::filesystem::directory_iterator{});
*/


int main(int argc, char* argv[]) {
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    /*
    CMake 13
    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/home/andreatirelli/Developments/platform/perception/assignment_1/dataset_1/"},
                                             boost::filesystem::directory_iterator{});
    */


    /*
    CMake 17
    */
    std::vector<fs::path> stream;
    for (const auto& entry : fs::directory_iterator("/home/andreatirelli/Developments/platform/perception/assignment_1/dataset_1/")) {
        stream.push_back(entry.path());
    }

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while(not renderer.WasViewerStopped()) {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        /**
         * TODO: Enable this output IF DEBUG=True
        */
        // std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded " << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}
