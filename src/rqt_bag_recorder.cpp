#include "rqt_bag_recorder/rqt_bag_recorder.hpp"

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include <QTextStream>

namespace rqt_bag_recorder{

BagRecorder::BagRecorder(): 
    rqt_gui_cpp::Plugin(), 
    widget_(nullptr){

    setObjectName("BagRecorder");

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("rqt_bag_recorder");
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    executor_.add_node(node_);
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


    connect(ui_.test_topics_button, SIGNAL(pressed()), this, SLOT(onTestTopics()));
    connect(ui_.record_button, SIGNAL(pressed()), this, SLOT(onRecord()));
    connect(ui_.stop_recording, SIGNAL(pressed()), this, SLOT(onStopRecord()));

}

void BagRecorder::shutdownPlugin(){

}

void BagRecorder::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& /*instance_settings*/) const {

}

void BagRecorder::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/, const qt_gui_cpp::Settings& /*instance_settings*/){

}

void BagRecorder::onStopRecord(){
    recording_ = false;
}

void BagRecorder::onRecord(){
    recording_ = true;

    // first remove all subscriptions of deselected items
    QTreeWidgetItemIterator it(ui_.topic_tree);
    while(*it){
        if((*it)->checkState(0) == Qt::Unchecked){
            std::string topic = (*it)->text(3).toStdString();
            subs_[topic].reset();
        }
        ++it;
    }

    // spin ros
    while(recording_){
        executor_.spin_some();
        QCoreApplication::processEvents();
    }

    rclcpp::QoS qos(10);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;


    // now create subscriptions again

    it = QTreeWidgetItemIterator(ui_.topic_tree);
    while(*it){
        if((*it)->checkState(0) == Qt::Unchecked){
            std::string topic = (*it)->text(3).toStdString();
            std::string type = topic_info_[topic][0];

            std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback_function = std::bind(&BagRecorder::genericTimerCallback, this, std::placeholders::_1, topic, type);
            subs_[topic] = node_->create_generic_subscription(topic, type, qos, callback_function, options);
        }

        ++it;
    }
}

void BagRecorder::onTestTopics(){
    // run for 2 seconds and count all incoming messages
    auto time_in_2 = node_->get_clock()->now() + rclcpp::Duration(2,0);
    while(node_->get_clock()->now() < time_in_2){
        executor_.spin_some();
        QCoreApplication::processEvents();
    }

    // now draw the amount of messages received in the tree
    for(auto &topic_pair : n_msgs_received_){
        auto tree_item_list = ui_.topic_tree->findItems(QString::fromStdString(topic_pair.first), Qt::MatchExactly, 3);

        // throw if topic is not exactly once in list
        if(tree_item_list.size() != 1)
            throw std::runtime_error("Topic either does not exist or more than once!");

        QString s;
        QTextStream ss(&s);
        ss << topic_pair.second << " messages received";
        tree_item_list.at(0)->setText(1, s);

        // draw background to indicate goodness
        if(topic_pair.second > 0)
            tree_item_list.at(0)->setBackground(1, QBrush(Qt::green));
        else
            tree_item_list.at(0)->setBackground(1, QBrush(Qt::red));
        
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

    TableRow new_row = {Qt::Checked, "not tested", QString::fromStdString(topic_info_[topic.toStdString()][0]), topic};
    addRowToTable(new_row);
}


void BagRecorder::genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string topic, std::string type){
    n_msgs_received_[topic] += 1;
}

void BagRecorder::addRowToTable(TableRow row){
    QTreeWidgetItem *item = new QTreeWidgetItem(ui_.topic_tree);

    item->setCheckState(0, row.record);
    item->setText(1, row.status);
    item->setText(2, row.type);
    item->setText(3, row.topic);

    rclcpp::QoS qos(10);
    std::string std_topic = row.topic.toStdString();
    std::string std_type = row.type.toStdString(); 

    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;

    std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback_function = std::bind(&BagRecorder::genericTimerCallback, this, std::placeholders::_1, std_topic, std_type);

    subs_.emplace(std_topic, node_->create_generic_subscription(std_topic, std_type, qos, callback_function, options));
    n_msgs_received_.emplace(std_topic, 0);

    // add item to the tree
    ui_.topic_tree->addTopLevelItem(item);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rqt_bag_recorder::BagRecorder, rqt_gui_cpp::Plugin)
