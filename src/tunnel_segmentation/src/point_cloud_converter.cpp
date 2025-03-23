#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <string>
#include <vector>

// Custom point type with intensity and label
struct PointTunnelXYZIL {
    float x;
    float y;
    float z;
    float intensity;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// PCL point type registration
POINT_CLOUD_REGISTER_POINT_STRUCT(PointTunnelXYZIL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, label, label)
)

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("point_cloud_converter") {
        // Declare parameters
        this->declare_parameter("data_file", "");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("publish_rate", 1.0);
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "tunnel_cloud", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / this->get_parameter("publish_rate").as_double())),
            std::bind(&PointCloudConverter::publish_cloud, this));
        
        RCLCPP_INFO(this->get_logger(), "Point Cloud Converter initialized");
    }

private:
    void publish_cloud() {
        std::string data_file = this->get_parameter("data_file").as_string();
        if (data_file.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data file specified. Use 'data_file' parameter");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Reading point cloud from: %s", data_file.c_str());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        try {
            // Read point cloud from text file
            std::ifstream file(data_file);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", data_file.c_str());
                return;
            }
            
            std::string line;
            float x, y, z, intensity;
            float label;
            
            while (std::getline(file, line)) {
                if (line.empty()) continue;
                
                std::istringstream iss(line);
                if (!(iss >> x >> y >> z >> intensity >> label)) {
                    continue;  // Skip malformed lines
                }
                
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                
                cloud->push_back(point);
                
                // Only read a small subset for testing
                if (cloud->size() >= 100000) {
                    RCLCPP_INFO(this->get_logger(), "Read 100000 points, limiting for testing");
                    break;
                }
            }
            
            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = false;
            
            RCLCPP_INFO(this->get_logger(), "Read %zu points from file", cloud->size());
            
            if (cloud->empty()) {
                RCLCPP_ERROR(this->get_logger(), "No points were read from the file!");
                return;
            }
            
            // Print the first few points for debugging
            for (size_t i = 0; i < std::min(size_t(5), cloud->size()); ++i) {
                RCLCPP_INFO(this->get_logger(), "Point %zu: %.2f, %.2f, %.2f, %.2f", 
                            i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].intensity);
            }
            
            // Convert to ROS message
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.frame_id = this->get_parameter("frame_id").as_string();
            output.header.stamp = this->now();
            
            // Publish
            publisher_->publish(output);
            RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points", cloud->size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 