#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/bool.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // std::shared_ptr<std_msgs::msg::Bool> msg = std::make_shared<std_msgs::msg::Bool>();
    // msg->data = true;
    std::string topic = "/test_t";
    std::string type = "std_msgs/msg/bool";

    std::shared_ptr<rclcpp::SerializedMessage> msg = std::make_shared<rclcpp::SerializedMessage>(2);

    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();

    std::string out_1 = "/home/yannick/Desktop/test_rec_1";
    std::string out_2 = "/home/yannick/Desktop/test_rec_2";

    auto time_stamp = rclcpp::Time(1); 

    writer->open(out_1);
    writer->write(msg, topic, type, time_stamp);
    writer->close();

    writer->open(out_2);
    writer->write(msg, topic, type, time_stamp);
    writer->close();

    rclcpp::shutdown();
    return 0;
}