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
        void onStopRecord();
        void onTestTopics();
        void onLoadConfig();
        void updateTopicList();
        void addTopic();
        void addAllTopics();
        void onSetOutput();
        void checkOutFile(const QString &file_path);

    private:
        void addRowToTable(TableRow row);
        void genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string topic, std::string type);


        std::unique_ptr<rosbag2_cpp::Writer> writer_;
        std::map<std::string, std::vector<std::string>> topic_map_full_;

        rclcpp::Node::SharedPtr node_;
        std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;

        rclcpp::executors::MultiThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;

        std::unordered_map<std::string, size_t> n_msgs_received_;
        std::map<std::string, std::vector<std::string>> topic_info_;
        bool recording_;
        bool lock_recording_;
        std::string out_folder_path_;

        Ui::BagRecorderWidget ui_;
        QWidget* widget_;
};

} // rqt_bag_recorder

#endif // RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP