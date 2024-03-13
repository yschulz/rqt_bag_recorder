#ifndef RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP
#define RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rqt_gui_cpp/plugin.h>
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"

#include <ui_rqt_bag_recorder.h>

#include <QFileDialog>
#include <QWidget>


namespace rqt_bag_recorder {

struct TableRow{
    Qt::CheckState record;
    QString status;
    QString type;
    QString topic;
};
    
class BagRecorder: public rqt_gui_cpp::Plugin{
    Q_OBJECT

    public:
        BagRecorder();
        ~BagRecorder();
        void initPlugin(qt_gui_cpp::PluginContext& context) override;
        void shutdownPlugin() override;
        void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
        void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

    protected slots:
        void onRecord(); 
        void onTestTopics();
        void updateTopicList();
        void addTopic();

    private:
        void addRowToTable(TableRow row);
        void genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
        
        // void genericRecorderCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
        std::unique_ptr<rosbag2_cpp::Writer> writer_;
        std::map<std::string, std::vector<std::string>> topic_map_full_;

        rclcpp::Node::SharedPtr node_;
        std::vector<rclcpp::GenericSubscription::SharedPtr> subs_; 

        std::map<std::string, std::vector<std::string>> topic_info_;
        rclcpp::Time temp_time_;
        rclcpp::Duration temp_period_;
        bool logging_time_;

        Ui::BagRecorderWidget ui_;
        QWidget* widget_;
};

} // rqt_bag_recorder

#endif // RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP