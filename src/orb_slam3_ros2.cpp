#include <orb_slam3_ros2.h>

class ORB_SLAM3_ROS : public rclcpp::Node
{
    public:
    // --------- Params -----------
    string orb_config_file, vocab_file_name;
    string cam1_topic, cam2_topic, imu_topic, output_topic;
    string slam_mode_str;
    string pose_frame_id;
    ORB_SLAM3::System::eSensor slam_mode;
    bool with_visualization;

    // ----------- ROS thingies --------
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image1_subber, image2_subber;
    //rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image1_subber, image2_subber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subber;
    message_filters::Subscriber<sensor_msgs::msg::Image> image1_subber, image2_subber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    //------------- Process variables -------------
    std::mutex imu_list_mutex;
    bool imu_initialized = false;
    std::queue<ORB_SLAM3::IMU::Point> imu_list;
    std::vector<ORB_SLAM3::IMU::Point> imu_list_send;
    bool image_initialized = false;
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
    geometry_msgs::msg::PoseStamped pose_msg;
    nav_msgs::msg::Path path_msg;
    double last_pose_time = -1.0;
    double between_path_time;

    ////////////////////////// C O N S T R U C T O R ///////////////////////////////
    ORB_SLAM3_ROS(string exec_path) : Node("orb_slam3_ros")  
    {
        declare_parameter("cam1_topic", "N/A");
        declare_parameter("cam2_topic", "N/A");
        declare_parameter("imu_topic", "N/A");
        declare_parameter("output_topic", "N/A");
        declare_parameter("slam_mode", "N/A");
        declare_parameter("config_file_name", "N/A");
        declare_parameter("vocab_file_name", "N/A");
        declare_parameter("pose_frame_id", "N/A");
        declare_parameter("with_visualization", true);
        declare_parameter("between_path_time", 0.5);

        orb_config_file = get_parameter("config_file_name").as_string();
        cam1_topic = get_parameter("cam1_topic").as_string();
        cam2_topic = get_parameter("cam2_topic").as_string();
        imu_topic = get_parameter("imu_topic").as_string();
        output_topic = get_parameter("output_topic").as_string();
        slam_mode_str = get_parameter("slam_mode").as_string();
        vocab_file_name = get_parameter("vocab_file_name").as_string();
        pose_frame_id = get_parameter("pose_frame_id").as_string();
        with_visualization = get_parameter("with_visualization").as_bool();
        between_path_time = get_parameter("between_path_time").as_double();
        
        cout << "------------ ROS CONFIG FILE LOADED ---------------" << endl;
        cout << "   cam1_topic: " << cam1_topic << endl;
        cout << "   cam2_topic: " << cam2_topic << endl;
        cout << "   imu_topic: " << imu_topic << endl;
        cout << "   output_topic: " << output_topic << endl;
        cout << "   slam_mode: " << slam_mode_str << endl;
        cout << "   config_file_name: " << orb_config_file << endl;
        cout << "   vocab_file_name: " << vocab_file_name << endl;
        cout << "   pose_frame_id: " << pose_frame_id << endl;
        cout << "   with_visualization: " << with_visualization << endl;
        cout << "   between_path_time: " << between_path_time << endl;

        cout << "   done!" << endl;
        // Now parse mode and strore it
        if(!judging_mode(slam_mode_str))
        {
            std::cerr << "  EGG: Egg is broken, the slam_mode setting is too." << endl;
            exit(1);
        }
        
        // Check if config file is there
        cout << "---------------- ORB CONFIG FILE ------------------" << endl;
        string exec_path_abs = exec_path.erase(exec_path.size()-14, 14);
        string config_path_ = exec_path_abs + "/config/ORB/" + orb_config_file;
        string vocab_path_ = exec_path_abs + "/config/ORB/" + vocab_file_name;
        cout << "  Tryna load " << config_path_ << "" << endl;
        if(!is_file_there(config_path_))
        {
            std::cerr << "  ERROR: Where is Germany? *point to the map* Here.\n"  
                << "    Where is the ORB setting file? *point to chest* Here." << endl << endl;
            exit(2);
        } 
        cout << "    done!" << endl;
        cout << "  Tryna load " << vocab_path_ << "" << endl;
        if(!is_file_there(vocab_path_))
        {
            std::cerr << "  ERROR: Where is Italy? *point to the map* Here.\n"  
                << "    Where is the ORB vocab file? *point to chest* Here." << endl << endl;
            exit(3);
        }
        cout << "    done!" << endl;

        // Tryna initialize SLAM 
        cout << "---------------- Initializing ORB_SLAM3 ------------------" << endl;
        mpSLAM = new ORB_SLAM3::System(vocab_path_, config_path_, slam_mode, with_visualization);
        cout << "    done!" << endl;

        cout << "---------------- Initializing Subbers ------------------" << endl;
        // Now create subscribers
        rclcpp::SubscriptionOptions options; 
        options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // For IMU
        if(slam_mode == ORB_SLAM3::System::IMU_MONOCULAR || slam_mode == ORB_SLAM3::System::IMU_STEREO)
        {
            cout << "   Initializing IMU" << endl;
            imu_subber = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 
                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 100), rmw_qos_profile_sensor_data),
                
                std::bind(&ORB_SLAM3_ROS::imu_callback, this, placeholders::_1), 
                options);
        }

        // For Camera
        if(slam_mode == ORB_SLAM3::System::IMU_STEREO || slam_mode == ORB_SLAM3::System::STEREO)
        {
            // W.I.P.
            // cout << "---------------- Initializing Stereo ------------------" << endl;
            // image1_subber.subscribe(this, cam1_topic);
            // image2_subber.subscribe(this, cam2_topic);
            // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
            // message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), image1_subber, image2_subber);
            // syncApproximate.registerCallback(&ORB_SLAM3_ROS::stereo_callback, this); 
        }
        else if(slam_mode == ORB_SLAM3::System::IMU_MONOCULAR)
        {
            cout << "   Initializing Mono IMU" << endl;
            options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            image_subber = this->create_subscription<sensor_msgs::msg::Image>(cam1_topic,
                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 1), rmw_qos_profile_sensor_data),
                std::bind(&ORB_SLAM3_ROS::image_monoimu_callback, this, placeholders::_1), 
                options);
        }
        else if(slam_mode == ORB_SLAM3::System::MONOCULAR)
        {
            cout << "   Initializing Mono IMU" << endl;
            options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            image_subber = this->create_subscription<sensor_msgs::msg::Image>(cam1_topic,
                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 1), rmw_qos_profile_sensor_data),
                std::bind(&ORB_SLAM3_ROS::image_mono_callback, this, placeholders::_1), 
                options);
        }

        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic + "/pose", 1);
        path_pub = this->create_publisher<nav_msgs::msg::Path>(output_topic + "/path", 1);
        cout << "    done!" << endl;
        cout << endl << ".*.*.*.*.*.* [[[[[ Ready to track! ٩(◕‿◕)۶ ]]]]] *.*.*.*.*.*." << endl;
        
    }

    //----------------------------- CALLBACKS --------------------------------------
    void image_monoimu_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if(imu_initialized)
        {
            // Get image
            cv_ptr = cv_bridge::toCvCopy(msg);
            auto image = cv_ptr->image.clone();
            double image_time = rclcpp::Time(cv_ptr->header.stamp).seconds();

            // Reset send list if it has been sent (same condition as the send condition)
            if(imu_list_send.size() > 1)
            {
                imu_list_send.clear();
            }

            // Secure the lock
            imu_list_mutex.lock();
            // If IMU reading in this time period is not empty
            if (imu_list.size()>0)
            {
                // Append any IMU reading that has timestamp before the received image
                while(imu_list.size()>0 && imu_list.front().t <= image_time)
                {
                        imu_list_send.push_back(imu_list.front()); 
                        imu_list.pop();
                }
                // "I don't wanna play with you anymore." - Andy
                imu_list_mutex.unlock();
                // If there are at least 2 IMU reading in the send list (cuz if there's only 1 ORB_SLAM3 will give error.)
                if(imu_list_send.size() > 1)
                {
                    // Send data, and you get in return a POSE!
                    // Best trade deal in the history of trade deals, maybe ever.
                    Sophus::SE3f pose = mpSLAM->TrackMonocular(image , image_time, imu_list_send).inverse();
                    //auto orb_state = mpSLAM->GetTrackingState();
                    publish_pose(pose);
                }
                else
                {
                    // If not then just skip, and next time don't clear the send list
                }
            }
            // "I don't wanna play with you anymore." - Andy
            imu_list_mutex.unlock();
        }
        // Tell the world that your Image is ready.
        if(!image_initialized)
        {
            image_initialized = true;
            cout << "   First image received!" << endl;
        }
    }
    void image_mono_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
        auto image = cv_ptr->image.clone();
        double image_time = rclcpp::Time(cv_ptr->header.stamp).seconds();
        Sophus::SE3f pose = mpSLAM->TrackMonocular(image , image_time);
        publish_pose(pose);
    }
    void stereo_callback(const sensor_msgs::msg::Image::SharedPtr image1, const sensor_msgs::msg::Image::SharedPtr image2)
    { 
        // W.I.P.
    }; 
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Put things into cv::Point3f
        cv::Point3f acc((float)msg->linear_acceleration.x, (float)msg->linear_acceleration.y, (float)msg->linear_acceleration.z);
        cv::Point3f gyr((float)msg->angular_velocity.x, (float)msg->angular_velocity.y, (float)msg->angular_velocity.z);
        
        // Push the imu reading to the list (It's actually a std::queue, not a list)
        imu_list_mutex.lock();
        imu_list.push(ORB_SLAM3::IMU::Point(acc, gyr, rclcpp::Time(msg->header.stamp).seconds()));
        imu_list_mutex.unlock();

        // Tell the world that your IMU is ready.
        if(!imu_initialized)
        {
            imu_initialized = true;
            cout << "   First IMU Message received!"<< endl;
        }
    }

    // ------------------------- Publisher ffffunctions -----------------------
    void publish_pose(Sophus::SE3f pose)
    {   
        double time_now_secs = this->now().seconds();

        // Initialize last_pose_time
        if(last_pose_time == -1.0)
            last_pose_time = time_now_secs;
        
        // Compose pose msg
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = pose_frame_id;
        pose_msg.pose.position.x = pose.translation().x(); 
        pose_msg.pose.position.y = pose.translation().y(); 
        pose_msg.pose.position.z = pose.translation().z(); 
        pose_msg.pose.orientation.x = pose.data()[0];
        pose_msg.pose.orientation.y = pose.data()[1];
        pose_msg.pose.orientation.z = pose.data()[2];
        pose_msg.pose.orientation.w = pose.data()[3];
        // Send it, corporal!
        pose_pub->publish(pose_msg);

        // Add pose to path if "the time is ripe"
        if(time_now_secs - last_pose_time >= between_path_time)
        {
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = pose_frame_id;
            // Add latest pose to the path msg
            path_msg.poses.push_back(pose_msg);
            // Send it, corporal!
            path_pub->publish(path_msg);
            // Reset timer
            last_pose_time = time_now_secs;
        }
    }

    // -------------------- Initialization functions -----------------------------
    bool is_file_there(string filepath)
    {
        ifstream f(filepath.c_str());
        return f.good();
    }
    bool judging_mode(string mode_str)
    {
        //Parse sensor type and store sensor type
        if(mode_str == "MONOCULAR")
        {
            slam_mode = ORB_SLAM3::System::MONOCULAR;
            return true;
        }
        else if(mode_str ==  "STEREO")
        {
            slam_mode = ORB_SLAM3::System::STEREO;
            return true;
        }
        else if(mode_str ==  "IMU_MONOCULAR")
        {
            slam_mode = ORB_SLAM3::System::IMU_MONOCULAR;
            return true;
        }
        else if(mode_str ==  "IMU_STEREO")
        {
            slam_mode = ORB_SLAM3::System::IMU_STEREO;
            return true;
        }
        else
        {
            return false;
        }
    }
};

/////////////////////////////////////// MAIN /////////////////////////////////////////
////////////////////////////// M A I N  M A I N  M A I N /////////////////////////////
/////////////////////////////////////// MAIN /////////////////////////////////////////
int main(int argc, char * argv[])
{
    // cout << "argruments" << endl;
    // for(int i = 0 ; i < argc ; i++)
    // {
    //     cout << argv[i] << endl;
    // }
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    string exec_path(argv[0]);
    rclcpp::Node::SharedPtr node = std::make_shared<ORB_SLAM3_ROS>(exec_path);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}
