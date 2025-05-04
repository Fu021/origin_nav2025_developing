#include <small_gicp_localization/small_gicp_localization.hpp>

#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace small_gicp_localization
{
    SmallGicpLocalization::SmallGicpLocalization(const rclcpp::NodeOptions& options)
    : Node("small_gicp_localization", options)
    {
        this->declare_parameter("num_threads", 4);
        this->declare_parameter("num_neighbors", 20);
        this->declare_parameter("global_leaf_size", 0.25);
        this->declare_parameter("registered_leaf_size", 0.25);
        this->declare_parameter("max_dist_sq", 1.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_link_frame", "base_link");
        this->declare_parameter("pcd_file", "");
        this->declare_parameter("cloud_registered_topic","cloud_registered");

        this->get_parameter("num_threads",num_threads);
        this->get_parameter("num_neighbors",num_neighbors);
        this->get_parameter("global_leaf_size",global_leaf_size);
        this->get_parameter("registered_leaf_size",registered_leaf_size);
        this->get_parameter("max_dist_sq",max_dist_sq);
        this->get_parameter("map_frame",map_frame);
        this->get_parameter("odom_frame",odom_frame);
        this->get_parameter("base_link_frame",base_link_frame);
        this->get_parameter("pcd_file",pcd_file);
        this->get_parameter("cloud_registered_topic",cloud_registered_topic);
        
        ready = 0;

        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();  // 队列深度设为100
        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this, qos);
        localization_register = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP>>();

        init_T = Eigen::Isometry3d().Identity();
        publish_T = Eigen::Isometry3d().Identity();

        load_pcd_file(pcd_file);

        cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_registered_topic,10,std::bind(&SmallGicpLocalization::cloud_callback,this,std::placeholders::_1)
        );
        init_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose",10,std::bind(&SmallGicpLocalization::init_pose_callback,this,std::placeholders::_1)
        );

        publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),std::bind(&SmallGicpLocalization::publish_transform,this)
        );
        localization_timer = this->create_wall_timer(
            std::chrono::milliseconds(500),std::bind(&SmallGicpLocalization::perform_localization,this)
        );
    }

    void SmallGicpLocalization::load_pcd_file(const std::string& pcd_file)
    {
        pcl::PointCloud<pcl::PointXYZI> PCD = pcl::PointCloud<pcl::PointXYZI>();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file,PCD) == -1)
        {
            RCLCPP_ERROR(this->get_logger(),"fail reading %s", pcd_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(),"success reading %s",pcd_file.c_str());
        PCD_downsampled = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>,pcl::PointCloud<pcl::PointCovariance>>(PCD,global_leaf_size);
        PCD_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(PCD_downsampled, small_gicp::KdTreeBuilderOMP(num_threads));
    }

    void SmallGicpLocalization::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud = pcl::PointCloud<pcl::PointXYZI>();

        last_cloud_stamp = msg->header.stamp;
        cloud_frame_id = msg->header.frame_id;
        pcl::fromROSMsg(*msg, cloud);

        cloud_downsampled = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(cloud,registered_leaf_size);
        small_gicp::estimate_covariances_omp(*cloud_downsampled, num_neighbors, num_threads);
        ready = 1;
    }

    void SmallGicpLocalization::publish_transform()
    {
        geometry_msgs::msg::TransformStamped publish_transform = tf2::eigenToTransform(publish_T);
        publish_transform.header.stamp = last_cloud_stamp + rclcpp::Duration::from_seconds(0.01);
        publish_transform.header.frame_id = map_frame;
        publish_transform.child_frame_id = odom_frame;
        broadcaster->sendTransform(publish_transform);
    }

    void SmallGicpLocalization::perform_localization()
    {
        if (!ready)
            return;
        
        localization_register->reduction.num_threads = num_threads;
        localization_register->rejector.max_dist_sq = max_dist_sq;

        auto result = localization_register->align(*PCD_downsampled,*cloud_downsampled,*PCD_tree,init_T);

        if (!result.converged)
        {
            return;
        }

        init_T = result.T_target_source;
        publish_T = result.T_target_source;
    }

    void SmallGicpLocalization::init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
    {
        Eigen::Isometry3d map_to_base_link = Eigen::Isometry3d::Identity();
        map_to_base_link.translation() = Eigen::Vector3d(msg->pose.pose.position.x,
                                                        msg->pose.pose.position.y,
                                                        msg->pose.pose.position.z);
        map_to_base_link.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                        msg->pose.pose.orientation.x,
                                                        msg->pose.pose.orientation.y,
                                                        msg->pose.pose.orientation.z).toRotationMatrix();

        if (cloud_frame_id != base_link_frame)
        {
            geometry_msgs::msg::TransformStamped base_link_to_cloud;
            try
            {
                base_link_to_cloud = buffer->lookupTransform(base_link_frame,cloud_frame_id,rclcpp::Time(0));
            }
            catch(const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(),"Could't find transform from %s to %s: %s",
                            base_link_frame.c_str(),cloud_frame_id.c_str(),e.what());
                return;
            }

            Eigen::Isometry3d base_link_to_cloud_eigen = tf2::transformToEigen(base_link_to_cloud);
            init_T = map_to_base_link * base_link_to_cloud_eigen;
            publish_T = init_T;
            RCLCPP_INFO(this->get_logger(),"get init_T");
        }
        else
        {
            init_T = map_to_base_link;
            publish_T = init_T;
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_localization::SmallGicpLocalization)