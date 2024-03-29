#ifndef RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP
#define RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>

#include <rqt_gui_cpp/plugin.h>
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"

#include <ui_rqt_bag_recorder.h>

#include <QFileDialog>
#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QButtonGroup>
#include <QStorageInfo>
#include <QSortFilterProxyModel>

#include <future>


namespace rqt_bag_recorder {

struct TableRow{
    Qt::CheckState record;
    QString status;
    QString type;
    QString topic;
};

struct SetItem{
    QFileInfo file_info;
    YAML::Node yaml_node;
    QPushButton* set_button;
    int versions = 0;
    std::vector<std::string> bag_names;
    std::vector<double> bag_length;
    std::vector<int> file_size;
    
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

    signals:
        void sendRecordStatus(int record);

    protected slots:
    
        void onFormatChanged(const QString &text);
        void onRecord();
        void onTestTopics();
        void onLoadConfig();
        void onSaveConfig();
        
        void updateTopicList();
        void addTopic();
        void addAllTopics();
        void onSetOutput();
        void onBasePathChanged(const QString &file_path);
        void onBagNameChanged(const QString &bag_name);


        void onSelectAll();
        void onDeselectAll();

        void onLoadSet();
        void onExportSet();

        void onSetButtonClicked(int button_id);

        void onToggleCompression(int state);

        void onToggleBagSize(int state);
        void onToggleBagLength(int state);
        void onBagSizeSpin(int value);
        void onBagLengthSpin(int value);

        void onFilterTextChanged(const QString &filter_text);

        void updateFreeSpace();

    private:
        
        void checkFilePath();
        void addRowToTable(TableRow row);
        void genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string topic, std::string type);

        void updateSubscribers();

        void updateCompressionOptions();

        void setConfig(const YAML::Node& root);

        void lockAllWidgets(bool lock);

        std::unique_ptr<rosbag2_cpp::Writer> writer_;

        rosbag2_cpp::ConverterOptions converter_options_;
        rosbag2_storage::StorageOptions storage_options_;
        rosbag2_compression::CompressionOptions compression_options_;


        std::map<std::string, std::vector<std::string>> topic_map_full_;

        rclcpp::Node::SharedPtr node_;
        std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;

        rclcpp::executors::MultiThreadedExecutor executor_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;

        std::unordered_map<std::string, size_t> n_msgs_received_;
        std::map<std::string, std::vector<std::string>> topic_info_;
        bool recording_;
        bool lock_recording_;

        QString base_output_folder_;
        QString bag_name_;

        QHash<int, SetItem> set_item_hash_;

        QStorageInfo q_storage_info_;
        QTimer* free_space_timer_;

        QButtonGroup* b_group_;
        int total_set_items_ = 0;
        int current_set_ = -1;

        std::future<void> compression_future_;

        Ui::BagRecorderWidget ui_;
        QWidget* widget_;

        QSortFilterProxyModel* filter_proxy_;
};

} // rqt_bag_recorder

#endif // RQT_BAG_RECORDER__RQT_BAG_RECORDER_HPP