#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

// This node implements basic segmentation for tunnel point clouds
// More advanced methods would be implemented based on the papers

class TunnelSegmentation : public rclcpp::Node {
public:
    TunnelSegmentation() : Node("tunnel_segmentation") {
        // Parameters
        this->declare_parameter("cluster_tolerance", 0.2);  // in meters
        this->declare_parameter("min_cluster_size", 50);   // points
        this->declare_parameter("max_cluster_size", 25000); // points
        this->declare_parameter("voxel_size", 0.1);        // downsampling voxel size
        
        // Publishers
        segmented_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "segmented_cloud", 10);
        
        // Subscribers
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "tunnel_cloud", 10, 
            std::bind(&TunnelSegmentation::cloud_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Tunnel Segmentation node initialized");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %u points", 
            msg->width * msg->height);
        
        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        RCLCPP_INFO(this->get_logger(), "Converted PCL cloud with %zu points", cloud->size());
        
        // Print a few sample points for debugging
        for (size_t i = 0; i < std::min(size_t(5), cloud->size()); ++i) {
            RCLCPP_INFO(this->get_logger(), "Point %zu: %.2f, %.2f, %.2f, %.2f", 
                        i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].intensity);
        }
        
        // Downsample the cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        double voxel_size = this->get_parameter("voxel_size").as_double();
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*cloud_filtered);
        
        RCLCPP_INFO(this->get_logger(), "Downsampled to %zu points with voxel size %.3f", 
                   cloud_filtered->size(), voxel_size);
        
        // Segment planes (ground, walls)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.setMaxIterations(100);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "Could not find any planes in the point cloud");
            
            // Just color the original points for visualization
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            colored_cloud->resize(cloud_filtered->size());
            
            for (size_t i = 0; i < cloud_filtered->size(); ++i) {
                colored_cloud->points[i].x = cloud_filtered->points[i].x;
                colored_cloud->points[i].y = cloud_filtered->points[i].y;
                colored_cloud->points[i].z = cloud_filtered->points[i].z;
                // Color based on height (z-value)
                colored_cloud->points[i].r = 255 * (1.0 - (cloud_filtered->points[i].z - (-2.0)) / 4.0);
                colored_cloud->points[i].g = 255 * ((cloud_filtered->points[i].z - (-2.0)) / 4.0);
                colored_cloud->points[i].b = 100;
            }
            
            colored_cloud->width = colored_cloud->size();
            colored_cloud->height = 1;
            colored_cloud->is_dense = true;
            
            // Convert back to ROS message
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*colored_cloud, output);
            output.header = msg->header;
            
            // Publish
            segmented_pub_->publish(output);
            RCLCPP_INFO(this->get_logger(), "Published colored point cloud without segmentation");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Found plane with %zu inliers", inliers->indices.size());
        
        // Extract non-plane points for further clustering
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_objects);
        
        RCLCPP_INFO(this->get_logger(), "Extracted %zu non-planar points", cloud_objects->size());
        
        // Euclidean clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_objects);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double());
        ec.setMinClusterSize(this->get_parameter("min_cluster_size").as_int());
        ec.setMaxClusterSize(this->get_parameter("max_cluster_size").as_int());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_objects);
        ec.extract(cluster_indices);
        
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters", cluster_indices.size());
        
        // Color each cluster with a different color
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        std::vector<unsigned char> colors[] = {
            {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0},
            {0, 255, 255}, {255, 0, 255}, {255, 127, 0}, {127, 255, 0},
            {0, 127, 255}, {127, 0, 255}, {255, 0, 127}, {0, 255, 127}
        };
        
        int j = 0;
        for (const auto& indices : cluster_indices) {
            for (const auto& idx : indices.indices) {
                pcl::PointXYZRGB point;
                point.x = cloud_objects->points[idx].x;
                point.y = cloud_objects->points[idx].y;
                point.z = cloud_objects->points[idx].z;
                point.r = colors[j % 12][0];
                point.g = colors[j % 12][1];
                point.b = colors[j % 12][2];
                colored_cloud->push_back(point);
            }
            j++;
        }
        
        // Add the planar components with a specific color (white)
        for (const auto& idx : inliers->indices) {
            pcl::PointXYZRGB point;
            point.x = cloud_filtered->points[idx].x;
            point.y = cloud_filtered->points[idx].y;
            point.z = cloud_filtered->points[idx].z;
            point.r = 255;
            point.g = 255;
            point.b = 255;
            colored_cloud->push_back(point);
        }
        
        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;
        
        RCLCPP_INFO(this->get_logger(), "Created colored cloud with %zu points", colored_cloud->size());
        
        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*colored_cloud, output);
        output.header = msg->header;
        
        // Publish
        segmented_pub_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published segmented point cloud");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TunnelSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 