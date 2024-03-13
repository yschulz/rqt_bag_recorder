#include "rqt_bag_recorder/rqt_bag_recorder.hpp"

#include "libstatistics_collector/moving_average_statistics/types.hpp"

namespace rqt_bag_recorder{

BagRecorder::BagRecorder(): 
    rqt_gui_cpp::Plugin(), 
    widget_(nullptr),
    temp_time_(0),
    temp_period_(0, 0){
    setObjectName("BagRecorder");

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("rqt_bag_recorder");
}

BagRecorder::~BagRecorder(){}

void BagRecorder::initPlugin(qt_gui_cpp::PluginContext& context){
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1) {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    // setup topic bar
    updateTopicList();
    connect(ui_.refresh_topics_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));
    connect(ui_.add_topic_button, SIGNAL(pressed()), this, SLOT(addTopic()));

    QStringList labels;
    labels << "record" << "status" << "type" << "topic";

    ui_.topic_tree->setColumnCount(labels.size());
    ui_.topic_tree->setHeaderLabels(labels);

    ui_.topic_tree->setColumnWidth(0, 80);
    ui_.topic_tree->setColumnWidth(1, 160);
    ui_.topic_tree->setColumnWidth(2, 350);


    connect(ui_.test_topics_button, SIGNAL(pressed()), this, SLOT(onTestTopic()));
    connect(ui_.record_button, SIGNAL(pressed()), this, SLOT(onRecord()));

}

void BagRecorder::shutdownPlugin(){

}

void BagRecorder::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& /*instance_settings*/) const {

}

void BagRecorder::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/, const qt_gui_cpp::Settings& /*instance_settings*/){

}

void BagRecorder::onRecord(){

}

void BagRecorder::onTestTopics(){

    auto rows = ui_.topic_tree->topLevelItemCount();

    for(auto i=0; i<rows; i++){
        auto item_ptr = ui_.topic_tree->topLevelItem(i);
        std::cout << item_ptr->text(3).toStdString() << "\n";

        rclcpp::QoS test(10);
        std::string std_topic = item_ptr->text(3).toStdString();
        std::string std_type = item_ptr->text(2).toStdString(); 
        rclcpp::GenericSubscription::SharedPtr new_sub;
        new_sub = node_->create_generic_subscription(std_topic, std_type, test, std::bind(&BagRecorder::genericTimerCallback, this, std::placeholders::_1));

        bool have_msg = false;
        // give it enough tries to connect
        for(size_t i=0; i<10000; i++){
            rclcpp::spin_some(node_);
            if(logging_time_){
                while(logging_time_){
                    rclcpp::spin_some(node_);
                }
                break;
            }
        }
        
        if(temp_period_.nanoseconds() == 0){
            item_ptr->setText(1, "no messages received");
            item_ptr->setBackground(1, QBrush(Qt::red));
        }
        else{
            item_ptr->setText(1, "messages received");
            item_ptr->setBackground(1, QBrush(Qt::green));
        }
        std::cout << std_topic << temp_period_.nanoseconds() << "\n";
        new_sub.reset();
        temp_time_ = rclcpp::Time(0);
        temp_period_ = rclcpp::Duration(0, 0);
        
    }

}

void BagRecorder::updateTopicList() {
    // get all topic names from ros node and save it
    topic_info_ = node_->get_topic_names_and_types();

    // transport to qt
    QList<QString> topics;

    for(const auto & topic_pair : topic_info_)
        topics.append(QString::fromStdString(topic_pair.first));
    
    QString selected = ui_.topics_combo_box->currentText();
    std::sort(topics.begin(), topics.end());

    // fill combo box
    ui_.topics_combo_box->clear();
    for(const auto &topic : topics){
        ui_.topics_combo_box->addItem(topic);
    }

    // reset index to blank
    ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
}

void BagRecorder::addTopic(){
    auto topic = ui_.topics_combo_box->currentText();
    auto widget_list = ui_.topic_tree->findItems(topic, Qt::MatchExactly, 3);

    if( widget_list.size() || 
        topic.isEmpty() || 
        topic_info_.find(topic.toStdString()) == topic_info_.end())
        return;

    TableRow new_row = {Qt::Unchecked, "good", QString::fromStdString(topic_info_[topic.toStdString()][0]), topic};
    addRowToTable(new_row);
}

void BagRecorder::genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    if(!logging_time_){
        temp_time_ = node_->get_clock()->now();
        logging_time_ = true;
    }
    else{
        temp_period_ = node_->get_clock()->now() - temp_time_;
        logging_time_ = false;
    }
}

void BagRecorder::addRowToTable(TableRow row){
    QTreeWidgetItem *item = new QTreeWidgetItem(ui_.topic_tree);

    item->setCheckState(0, row.record);
    item->setText(1, row.status);
    item->setText(2, row.type);
    item->setText(3, row.topic);

    rclcpp::QoS test(10);
    std::string std_topic = row.topic.toStdString();
    std::string std_type = row.type.toStdString(); 

    ui_.topic_tree->addTopLevelItem(item);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rqt_bag_recorder::BagRecorder, rqt_gui_cpp::Plugin)
