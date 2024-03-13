#ifndef RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP
#define RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rqt_gui_cpp/plugin.h>

#include <ui_rqt_bag_recorder.h>

#include <QFileDialog>
#include <QWidget>


namespace rqt_bag_recorder {
    
class BagRecorder: public rqt_gui_cpp::Plugin{
    Q_OBJECT

    public:
        BagRecorder();
        ~BagRecorder();
};

} // rqt_bag_recorder

#endif // RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP