#include "rqt_bag_recorder/rqt_bag_recorder.hpp"
#include "rqt_bag_recorder/bag_recorder_utilities.hpp"

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include <QTextStream>
#include <QMessageBox>
#include <QTimer>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>


namespace rqt_bag_recorder{

BagRecorder::BagRecorder(): 
    rqt_gui_cpp::Plugin(), 
    widget_(nullptr),
    b_group_(nullptr),
    // out_folder_path_(""),
    converter_options_({rmw_get_serialization_format(), rmw_get_serialization_format()}),
    storage_options_({"", "sqlite3"}){

    setObjectName("BagRecorder");

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("rqt_bag_recorder");
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    executor_.add_node(node_);

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
}

BagRecorder::~BagRecorder(){}

void BagRecorder::initPlugin(qt_gui_cpp::PluginContext& context){
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    b_group_ = new QButtonGroup(ui_.set_tree);

    if (context.serialNumber() > 1) {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    free_space_timer_ = new QTimer(this);
    free_space_timer_->setInterval(2000);

    connect(free_space_timer_, SIGNAL(timeout()), this, SLOT(updateFreeSpace()));

    // setup topic bar
    updateTopicList();
    connect(ui_.refresh_topics_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));
    connect(ui_.add_topic_button, SIGNAL(pressed()), this, SLOT(addTopic()));
    connect(ui_.add_all_topics, SIGNAL(pressed()), this, SLOT(addAllTopics()));

    QStringList labels;
    labels << "record" << "status" << "type" << "topic";

    ui_.topic_tree->setColumnCount(labels.size());
    ui_.topic_tree->setHeaderLabels(labels);


    filter_proxy_  = new QSortFilterProxyModel(widget_);
    filter_proxy_->setSourceModel(ui_.topic_tree->model());
    filter_proxy_->setFilterKeyColumn(-1);
    ui_.topic_tree_view->setModel(filter_proxy_);
    ui_.topic_tree->hide();

    ui_.topic_tree_view->setColumnWidth(0, 80);
    ui_.topic_tree_view->setColumnWidth(1, 160);
    ui_.topic_tree_view->setColumnWidth(2, 350);



    connect(ui_.o_format_cbox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(onFormatChanged(const QString &)));

    connect(ui_.test_topics_button, SIGNAL(pressed()), this, SLOT(onTestTopics()));
    connect(ui_.record_button, SIGNAL(pressed()), this, SLOT(onRecord()));
    connect(ui_.load_config, SIGNAL(pressed()), this, SLOT(onLoadConfig()));
    connect(ui_.save_config, SIGNAL(pressed()), this, SLOT(onSaveConfig()));


    connect(ui_.set_output, SIGNAL(pressed()), this, SLOT(onSetOutput()));
    connect(ui_.out_file_box, SIGNAL(textChanged(const QString&)), this, SLOT(onBasePathChanged(const QString &)));
    connect(ui_.bag_name_box, SIGNAL(textChanged(const QString&)), this, SLOT(onBagNameChanged(const QString &)));

    connect(this, SIGNAL(sendRecordStatus(int)), ui_.record_w, SLOT(setRecordStatus(int)));

    ui_.o_format_cbox->addItem(QString("sqlite3"));
    ui_.o_format_cbox->addItem(QString("mcap"));
    ui_.o_format_cbox->setCurrentIndex(ui_.o_format_cbox->findText("sqlite3"));

    ui_.o_compression_format_cbox->addItem(QString("Zstd"));
    ui_.o_compression_format_cbox->setCurrentIndex(ui_.o_format_cbox->findText(""));
    ui_.o_compression_format_cbox->setDisabled(true);

    ui_.o_compression_mode_cbox->addItem(QString("file"));
    ui_.o_compression_mode_cbox->addItem(QString("message"));
    ui_.o_compression_mode_cbox->setCurrentIndex(ui_.o_format_cbox->findText(""));
    ui_.o_compression_mode_cbox->setDisabled(true);

    connect(ui_.o_compression_toggle, SIGNAL(stateChanged(int)), this, SLOT(onToggleCompression(int)));

    connect(ui_.o_mx_size_toggle, SIGNAL(stateChanged(int)), this, SLOT(onToggleBagSize(int)));
    connect(ui_.o_mx_size_spin, SIGNAL(valueChanged(int)), this, SLOT(onBagSizeSpin(int)));
    ui_.o_mx_size_spin->setDisabled(true);
    ui_.o_mx_size_spin->setMaximum(std::numeric_limits<int>::max());

    connect(ui_.o_mx_length_toggle, SIGNAL(stateChanged(int)), this, SLOT(onToggleBagLength(int)));
    connect(ui_.o_mx_length_spin, SIGNAL(valueChanged(int)), this, SLOT(onBagLengthSpin(int)));
    ui_.o_mx_length_spin->setDisabled(true);
    ui_.o_mx_length_spin->setMaximum(std::numeric_limits<int>::max());

    connect(ui_.select_all_button, SIGNAL(pressed()), this, SLOT(onSelectAll()));
    connect(ui_.deselect_all_button, SIGNAL(pressed()), this, SLOT(onDeselectAll()));


    QStringList set_labels;
    set_labels << "name" << "set";
    ui_.set_tree->setColumnCount(set_labels.size());
    ui_.set_tree->setHeaderLabels(set_labels);

    ui_.set_tree->resize(400, ui_.set_tree->height());
    ui_.set_tree->setFixedWidth(400);
    ui_.set_tree->setColumnWidth(0, 340);
    ui_.set_tree->setColumnWidth(1, 50);

    connect(ui_.load_set_button, SIGNAL(pressed()), this, SLOT(onLoadSet()));

    connect(b_group_, SIGNAL(idClicked(int)), this, SLOT(onSetButtonClicked(int)));

    connect(ui_.topic_filter, SIGNAL(textChanged(const QString &)), this, SLOT(onFilterTextChanged(const QString &)));

    connect(ui_.export_set_button, SIGNAL(pressed()), this, SLOT(onExportSet()));
}

void BagRecorder::shutdownPlugin(){

}

void BagRecorder::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& /*instance_settings*/) const {

}

void BagRecorder::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/, const qt_gui_cpp::Settings& /*instance_settings*/){

}

void BagRecorder::onExportSet(){
    QString save_file = QFileDialog::getSaveFileName(widget_, "Export set metadata", "/home", tr("Yaml-file (*.yaml *.yml)"));

    if(save_file.isEmpty()) return;

    std::ofstream root_file;
    root_file.open(save_file.toStdString());

    YAML::Emitter out(root_file);
    out << YAML::BeginMap;

    for(const auto & bag_config : set_item_hash_){
        out << YAML::Key << bag_config.file_info.fileName().toStdString();
        out << YAML::BeginMap;
        out << YAML::Key << "bag_names" << YAML::BeginSeq;
        for(const auto & name : bag_config.bag_names){
            out << name;
        }
        out << YAML::EndSeq << YAML::Key << "bag_lengths" << YAML::BeginSeq;
        for(const auto & length : bag_config.bag_length){
            out << length;
        }

        out << YAML::EndSeq << YAML::Key << "bag_sizes" << YAML::BeginSeq;
        for(const auto & size : bag_config.file_size){
            out << size;
        }

        out << YAML::EndSeq << YAML::EndMap;
    }
    out << YAML::EndMap;

    root_file.close();

}

void BagRecorder::onFilterTextChanged(const QString &filter_text){
    filter_proxy_->setFilterRegularExpression(filter_text);
}

void BagRecorder::onFormatChanged(const QString &text){
    storage_options_.storage_id = text.toStdString();
}

void BagRecorder::setConfig(const YAML::Node& root){
    ui_.bag_name_box->setText(QString::fromStdString(root["bag_name"].as<std::string>()));

    onDeselectAll();

    if(root["compression"] && root["compression"]["compression_format"] && root["compression"]["compression_mode"]){
        ui_.o_compression_toggle->setCheckState(Qt::Checked);
        ui_.o_compression_format_cbox->setCurrentIndex(ui_.o_compression_format_cbox->findText(QString::fromStdString(root["compression"]["compression_format"].as<std::string>())));
        ui_.o_compression_mode_cbox->setCurrentIndex(ui_.o_compression_mode_cbox->findText(QString::fromStdString(root["compression"]["compression_mode"].as<std::string>())));

    }
    else
        ui_.o_compression_toggle->setCheckState(Qt::Unchecked);


    if(root["max_bag_length"]){
        ui_.o_mx_length_toggle->setCheckState(Qt::Checked);
        ui_.o_mx_length_spin->setValue(root["max_bag_length"].as<int>());
    }
    else
        ui_.o_mx_length_toggle->setCheckState(Qt::Unchecked);

    if(root["max_bag_size"]){
        ui_.o_mx_size_toggle->setCheckState(Qt::Checked);
        ui_.o_mx_size_spin->setValue(root["max_bag_size"].as<int>());
    }
    else
        ui_.o_mx_size_toggle->setCheckState(Qt::Unchecked);

    QString s;
    QTextStream ss(&s);

    for (const auto &topic : root["topics"]){
        auto topic_string = topic.as<std::string>();
        auto tree_item_list = ui_.topic_tree->findItems(QString::fromStdString(topic_string), Qt::MatchExactly, 3);

        if(tree_item_list.isEmpty()){
            ss << QString::fromStdString(topic_string) << "\n";
            continue;
        }

        tree_item_list.at(0)->setCheckState(0, Qt::Checked);
    }

    if(!s.isEmpty()){
        QMessageBox::warning(widget_, "Topics skipped!", "Topics are not in the list update your ros environment and the topic list and try again:\n" + s);
    }
}

void BagRecorder::onSetButtonClicked(int button_id){
    setConfig(set_item_hash_.value(button_id).yaml_node);
    current_set_ = button_id;
}

void BagRecorder::onLoadSet(){
    QString config_path = QFileDialog::getExistingDirectory(widget_, "Loading recording set", "/home", QFileDialog::Option::ShowDirsOnly);
    QDir config_dir(config_path);

    QStringList yaml_filter = {"*.yaml"};
    
    for (const QFileInfo &file : config_dir.entryInfoList(yaml_filter, QDir::Files)){
        set_item_hash_.insert(total_set_items_, {file, YAML::LoadFile(file.absoluteFilePath().toStdString()), new QPushButton("Set")});
        QTreeWidgetItem *item = new QTreeWidgetItem(ui_.set_tree);
        QTreeWidgetItem *child_item = new QTreeWidgetItem(item);
        b_group_->addButton(set_item_hash_.value(total_set_items_).set_button, total_set_items_);

        QLabel* label = new QLabel(ui_.set_tree);
        if(set_item_hash_.value(total_set_items_).yaml_node["description"]){
            label->setText(QString::fromStdString(set_item_hash_.value(total_set_items_).yaml_node["description"].as<std::string>()));
        }
        else{
            label->setText("");
        }
        label->setWordWrap(true);


        item->addChild(child_item);
        item->setText(0, file.fileName());

        ui_.set_tree->addTopLevelItem(item);

        ui_.set_tree->setItemWidget(item, 1, set_item_hash_.value(total_set_items_).set_button);
        ui_.set_tree->setItemWidget(child_item, 0, label);

        total_set_items_++;
    }
}

void BagRecorder::onSelectAll(){
    QTreeWidgetItemIterator it = QTreeWidgetItemIterator(ui_.topic_tree);
    while(*it){
        (*it)->setCheckState(0, Qt::Checked);
        ++it;
    }
}

void BagRecorder::onDeselectAll(){
    QTreeWidgetItemIterator it = QTreeWidgetItemIterator(ui_.topic_tree);
    while(*it){
        (*it)->setCheckState(0, Qt::Unchecked);
        ++it;
    }
}

void BagRecorder::onBagSizeSpin(int value){
    storage_options_.max_bagfile_size = value;
}

void BagRecorder::onBagLengthSpin(int value){
    storage_options_.max_bagfile_duration = value;
}

void BagRecorder::onToggleBagSize(int state){
    switch (state)
    {
    case Qt::Checked:
        ui_.o_mx_size_spin->setDisabled(false);
        break;
    case Qt::Unchecked:
        ui_.o_mx_size_spin->setValue(0);
        ui_.o_mx_size_spin->setDisabled(true);
        break;
    
    default:
        break;
    }
}

void BagRecorder::onToggleBagLength(int state){
    switch (state)
    {
    case Qt::Checked:
        ui_.o_mx_length_spin->setDisabled(false);
        break;
    case Qt::Unchecked:
        ui_.o_mx_length_spin->setValue(0);
        ui_.o_mx_length_spin->setDisabled(true);
        break;
    
    default:
        break;
    }
}

void BagRecorder::onToggleCompression(int state){

    switch (state)
    {
    case Qt::Checked:
        ui_.o_compression_format_cbox->setDisabled(false);
        ui_.o_compression_mode_cbox->setDisabled(false);
        ui_.o_compression_format_cbox->setCurrentIndex(ui_.o_compression_format_cbox->findText("Zstd"));
        ui_.o_compression_mode_cbox->setCurrentIndex(ui_.o_compression_mode_cbox->findText("file"));
        break;
    case Qt::Unchecked:
        ui_.o_compression_format_cbox->setCurrentIndex(ui_.o_compression_format_cbox->findText(""));
        ui_.o_compression_mode_cbox->setCurrentIndex(ui_.o_compression_mode_cbox->findText(""));
        ui_.o_compression_format_cbox->setDisabled(true);
        ui_.o_compression_mode_cbox->setDisabled(true);
        break;
    
    default:
        break;
    }
}

void BagRecorder::checkFilePath(){
    QDir full_dir(base_output_folder_);
    if(QDir(full_dir.filePath(bag_name_)).exists()){
        QPalette palette;
        palette.setColor(QPalette::Base, Qt::red);
        palette.setColor(QPalette::Text, Qt::black);
        ui_.bag_name_box->setPalette(palette);
        lock_recording_ = true;
        storage_options_.uri = std::string("");
    }
    else{
        QPalette palette;
        palette.setColor(QPalette::Base, Qt::white);
        palette.setColor(QPalette::Text, Qt::black);
        ui_.bag_name_box->setPalette(QApplication::palette());
        lock_recording_ = false;
        storage_options_.uri = full_dir.filePath(bag_name_).toStdString();
    }
}

void BagRecorder::onBagNameChanged(const QString &bag_name){
    bag_name_ = bag_name;
    checkFilePath();
}

void BagRecorder::onBasePathChanged(const QString &file_path){
    base_output_folder_ = file_path;
    q_storage_info_ = QStorageInfo(QDir(base_output_folder_).root());
    updateFreeSpace();
    checkFilePath();
}

void BagRecorder::onSaveConfig(){
    QString save_file = QFileDialog::getSaveFileName(widget_, "Saving configuration file", "/home", tr("Yaml-file (*.yaml *.yml)"));

    if(save_file.isEmpty()) return;

    std::ofstream root_file;
    root_file.open(save_file.toStdString());

    YAML::Emitter out(root_file);
    out << YAML::BeginMap;
    out << YAML::Key << "bag_name" << YAML::Value << bag_name_.toStdString();
    out << YAML::Key << "format" << YAML::Value << ui_.o_format_cbox->currentText().toStdString();

    if(ui_.o_compression_toggle->checkState() == Qt::Checked){
        out << YAML::Key << "compression" << YAML::BeginMap <<
                YAML::Key << "compression_format" << YAML::Value << ui_.o_compression_format_cbox->currentText().toStdString() << 
                YAML::Key << "compression_mode" << YAML::Value << ui_.o_compression_mode_cbox->currentText().toStdString() << YAML::EndMap;
    }

    if(ui_.o_mx_length_toggle->checkState() == Qt::Checked){
        out << YAML::Key << "max_bag_length" << YAML::Value << ui_.o_mx_length_spin->value();
    }

    if(ui_.o_mx_size_toggle->checkState() == Qt::Checked){
        out << YAML::Key << "max_bag_size" << YAML::Value << ui_.o_mx_size_spin->value();
    }

    out << YAML::Key << "topics" << YAML::BeginSeq;

    QTreeWidgetItemIterator it(ui_.topic_tree);
    while(*it){
        if((*it)->checkState(0) == Qt::Checked){
            out << (*it)->text(3).toStdString();
        }

        ++it;
    }
    out << YAML::EndSeq << YAML::EndMap;
    root_file.close();
}

void BagRecorder::onLoadConfig(){
    topic_info_ = node_->get_topic_names_and_types();

    QString config_path =  QFileDialog::getOpenFileName(widget_, tr("Open Config file"), "/home", tr("Images (*.yaml *.yml)"));
    if(config_path.isEmpty()) return;

    YAML::Node config = YAML::LoadFile(config_path.toStdString());

    QString s;
    QTextStream ss(&s);

    for (const auto &topic : config["topics"]){
        auto topic_string = topic.as<std::string>();

        auto tree_item_list = ui_.topic_tree->findItems(QString::fromStdString(topic_string), Qt::MatchExactly, 3);

        if(topic_info_.find(topic_string) == topic_info_.end() || !tree_item_list.isEmpty()){
            ss << QString::fromStdString(topic_string) << "\n";
            continue;
        }
        TableRow new_row = {Qt::Checked, "not tested", QString::fromStdString(topic_info_[topic_string][0]), QString::fromStdString(topic_string)};
        addRowToTable(new_row);
    }

    if(!s.isEmpty()){
        QMessageBox::warning(widget_, "Topics skipped!", "Topics are either duplicates or not listed in ros:\n" + s);
    }

}

void BagRecorder::onSetOutput(){
    QString out_folder_path = QFileDialog::getSaveFileName(widget_, tr("Save File"));
    ui_.out_file_box->setText(out_folder_path);
    base_output_folder_ = out_folder_path;
}

void BagRecorder::updateCompressionOptions(){
    std::cout << ui_.o_compression_mode_cbox->currentText().toStdString() << "\n";
    if(ui_.o_compression_format_cbox->currentText() == QString("Zstd")){
        compression_options_.compression_format = "zstd";
    }

    if(ui_.o_compression_mode_cbox->currentText() == QString("file")){
        compression_options_.compression_mode = rosbag2_compression::CompressionMode::FILE;
    }
    else if(ui_.o_compression_mode_cbox->currentText() == QString("message")){
        compression_options_.compression_mode = rosbag2_compression::CompressionMode::MESSAGE;
    }
    else{
        compression_options_.compression_mode = rosbag2_compression::CompressionMode::NONE;
    }
}

void BagRecorder::onRecord(){

    if(lock_recording_) return;

    // first we evaluate what is happening while it is recording
    if(recording_){
        recording_ = false;
        free_space_timer_->stop();
        onBagNameChanged(bag_name_);
        return;
    }

    if(base_output_folder_.isEmpty() || bag_name_.isEmpty()){
        QMessageBox::warning(widget_, "Cannot record.", "Folder or bag name is not valid!");
        return;
    }

    if(!QDir(base_output_folder_).mkpath(base_output_folder_) || !QDir(base_output_folder_).exists()){
        QString s;
        QTextStream ss(&s);
        ss << "Could not create base folder " << base_output_folder_ << ". Please change the folder.";
        QMessageBox::warning(widget_, "Cannot record.", s);
        return;
    }

    // At this point it is safe to record, lock all widgets and prepare
    lockAllWidgets(true);
    ui_.record_button->setText("Stop");

    // now go recording
    updateSubscribers();

    // give status update on topics
    QTreeWidgetItemIterator it(ui_.topic_tree);
    while(*it){
        if((*it)->checkState(0) == Qt::Unchecked){
            (*it)->setText(1, "idle");
            (*it)->setBackground(1, QBrush(Qt::gray));
        }
        else{
            (*it)->setText(1, "recording...");
            QBrush recording_brush;
            QColor brown(210, 147, 84, 255);
            recording_brush.setColor(brown);
            (*it)->setBackground(1, recording_brush);
            (*it)->setBackground(1, recording_brush);
        }
        ++it;
    }

    // reset writer and create either compressing writer or simple sequential
    writer_.reset();
    if(ui_.o_compression_toggle->checkState() == Qt::Checked){
        updateCompressionOptions();
        writer_ = std::make_unique<rosbag2_cpp::Writer>(std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options_));
    }
    else{
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
    }
    
    writer_->open(storage_options_, converter_options_);
    recording_ = true;
    free_space_timer_->start();
    emit sendRecordStatus(1);

    for(auto &topic_pair : n_msgs_received_){
        topic_pair.second = 0;
    }

    auto ros_now = node_->get_clock()->now();
    // spin ros and keep eventloop going
    while(recording_){
        executor_.spin_some();
        QCoreApplication::processEvents();
    }

    auto ros_diff = node_->get_clock()->now() - ros_now;

    emit sendRecordStatus(2);
    ui_.record_button->setText("Compressing");
    lock_recording_ = true;

    compression_future_ = std::async(std::launch::async, [this]() { writer_->close();});
    while(compression_future_.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready){
        QCoreApplication::processEvents();
    }
    
    emit sendRecordStatus(0);
    ui_.record_button->setText("Record");

    lock_recording_ = false;

    // update status after recording
    QTreeWidgetItemIterator it_after = QTreeWidgetItemIterator(ui_.topic_tree);
    while(*it_after){
        if((*it_after)->checkState(0) == Qt::Checked){
            (*it_after)->setText(1, "Done!");
            QBrush recording_brush;
            QColor yellow(245, 235, 94, 1);
            recording_brush.setColor(yellow);
            (*it_after)->setBackground(1, recording_brush);
        }
        ++it_after;
    }
    // release the widgets
    lockAllWidgets(false);

    // QDir(base_output_folder_);
    QDir full_bag_dir(QDir(base_output_folder_).filePath(bag_name_));

    qint64 full_size = 0;
    for (const QFileInfo &file : full_bag_dir.entryInfoList()){
        full_size += file.size();
    }

    QString s;
    QTextStream ss(&s);
    ss << "Bag with name " << bag_name_ << ":\nRecorded for " << QString::fromStdString(toStringSeconds(ros_diff.seconds())) << "\nBag is " << QString::fromStdString(toStringFileSize(full_size)) << " big.";
    
    QMessageBox::information(widget_, "Recording successful!", s);
    if(current_set_ == -1){
        return;
    }
    auto tree_item_list = ui_.set_tree->findItems(set_item_hash_.value(current_set_).file_info.fileName(), Qt::MatchExactly, 0);
    auto tree_item = tree_item_list.at(0);
    set_item_hash_[current_set_].versions += 1;

    set_item_hash_[current_set_].bag_length.push_back(ros_diff.seconds());
    set_item_hash_[current_set_].file_size.push_back(full_size);
    set_item_hash_[current_set_].bag_names.push_back(bag_name_.toStdString());

    if(set_item_hash_.value(current_set_).versions == 1){
        tree_item->setBackground(0, QBrush(Qt::green));

        QTreeWidgetItem *bytes_item = new QTreeWidgetItem(tree_item);
        bytes_item->setText(0, "Bytes: ");
        bytes_item->setText(1, QString::fromStdString(toStringFileSize(full_size)));
        tree_item->addChild(bytes_item);

        QTreeWidgetItem *length_item = new QTreeWidgetItem(tree_item);
        length_item->setText(0, "Length in seconds: ");
        length_item->setText(1, QString::fromStdString(toStringSeconds(ros_diff.seconds())));
        tree_item->addChild(length_item);

        QTreeWidgetItem *versions_item = new QTreeWidgetItem(tree_item);
        versions_item->setText(0, "Versions: ");
        versions_item->setText(1, QString::number(set_item_hash_.value(current_set_).versions));
        tree_item->addChild(versions_item);

        for (const auto &topic : set_item_hash_[current_set_].yaml_node["topics"]){
            QTreeWidgetItem *topic_item = new QTreeWidgetItem(tree_item);
            std::string topic_std = topic.as<std::string>();
            topic_item->setText(0, QString::fromStdString(topic_std));
            topic_item->setText(1, QString::number(n_msgs_received_[topic_std]));
        }
    }
    else{
        auto bytes_item = tree_item->child(1);
        bytes_item->setText(1, QString::fromStdString(toStringFileSize(full_size)));

        auto length_item = tree_item->child(2);
        length_item->setText(1, QString::fromStdString(toStringSeconds(ros_diff.seconds())));

        auto versions_item = tree_item->child(3);
        versions_item->setText(1, QString::number(set_item_hash_[current_set_].versions));
        
        int n = 4;
        while(tree_item->child(n)){
            auto topic_item = tree_item->child(n);
            topic_item->setText(1, QString::number(n_msgs_received_[topic_item->text(0).toStdString()]));
            n++;
        }
    }

    

}

void BagRecorder::onTestTopics(){
    // run for 2 seconds and count all incoming messages
    auto time_before = node_->get_clock()->now();

    for(auto &topic_pair : n_msgs_received_){
        topic_pair.second = 0;
    }
    
    double progress = 0;
    while(progress < 1){
        progress = (node_->get_clock()->now() - time_before).seconds() / rclcpp::Duration(2,0).seconds();
        executor_.spin_some();
        ui_.test_w->drawProgress(progress);
        QCoreApplication::processEvents();
    }
    ui_.test_w->drawProgress(0.0);

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

void BagRecorder::addAllTopics(){
    // get all topic names from ros node and save it
    topic_info_ = node_->get_topic_names_and_types();

    QString s;
    QTextStream ss(&s);

    for(const auto &topic_pair : topic_info_){
        auto tree_item_list = ui_.topic_tree->findItems(QString::fromStdString(topic_pair.first), Qt::MatchExactly, 3);
        if(!tree_item_list.isEmpty()){
            ss << QString::fromStdString(topic_pair.first) << "\n";
            continue;
        }
        TableRow new_row = {Qt::Checked, "not tested", QString::fromStdString(topic_pair.second[0]), QString::fromStdString(topic_pair.first)};
        addRowToTable(new_row);
    }
    if(!s.isEmpty()){
        QMessageBox::warning(widget_, "Found duplicates!", "Did not add topics:\n" + s);
    }
}

void BagRecorder::genericTimerCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string topic, std::string type){
    n_msgs_received_[topic] += 1;

    if(!recording_)
        return;

    rclcpp::Time time_stamp = node_->get_clock()->now();

    writer_->write(msg, topic, type, time_stamp);
}

void BagRecorder::updateSubscribers(){
    // first remove all subscriptions of deselected items and set status
    QTreeWidgetItemIterator it(ui_.topic_tree);
    while(*it){
        std::string topic = (*it)->text(3).toStdString();
        std::string type = topic_info_[topic][0];

        if((*it)->checkState(0) == Qt::Unchecked){

            // keep hash element but reset pointer
            if(subs_[topic])
                subs_[topic].reset();

        }
        else if((*it)->checkState(0) == Qt::Checked){
            if(!subs_[topic]){
                auto endpoint_infos = node_->get_publishers_info_by_topic(topic);

                // replace qos profile only if there are only transient local publishers
                bool replace = true;
                for(const auto &info : endpoint_infos){
                    replace = info.qos_profile().durability() != rclcpp::DurabilityPolicy::Volatile;
                }

                rclcpp::QoS qos(10);
                if(replace)
                    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

                rclcpp::SubscriptionOptions options;
                options.callback_group = cb_group_;

                std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback_function = std::bind(&BagRecorder::genericTimerCallback, this, std::placeholders::_1, topic, type);
                subs_[topic] = node_->create_generic_subscription(topic, type, qos, callback_function, options);
            }
        }
        ++it;
    }
}

void BagRecorder::addRowToTable(TableRow row){
    QTreeWidgetItem *item = new QTreeWidgetItem(ui_.topic_tree);

    item->setCheckState(0, row.record);
    item->setText(1, row.status);
    item->setText(2, row.type);
    item->setText(3, row.topic);

    auto endpoint_infos = node_->get_publishers_info_by_topic(row.topic.toStdString());

    // replace qos profile only if there are only transient local publishers
    bool replace = true;
    for(const auto &info : endpoint_infos){
        replace = info.qos_profile().durability() != rclcpp::DurabilityPolicy::Volatile;
    }

    rclcpp::QoS qos(10);
    if(replace){
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    }
        

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

void BagRecorder::lockAllWidgets(bool lock){

    ui_.out_file_box->setDisabled(lock);
    ui_.bag_name_box->setDisabled(lock);
    ui_.test_topics_button->setDisabled(lock);
    ui_.set_output->setDisabled(lock);
    ui_.o_format_cbox->setDisabled(lock);

    ui_.select_all_button->setDisabled(lock);
    ui_.deselect_all_button->setDisabled(lock);

    ui_.refresh_topics_button->setDisabled(lock);
    ui_.topics_combo_box->setDisabled(lock);
    ui_.add_all_topics->setDisabled(lock);
    ui_.add_topic_button->setDisabled(lock);
    ui_.load_config->setDisabled(lock);
    ui_.save_config->setDisabled(lock);
    ui_.load_set_button->setDisabled(lock);
    ui_.export_set_button->setDisabled(lock);

    ui_.o_mx_size_toggle->setDisabled(lock);
    if(ui_.o_mx_size_toggle->checkState() == Qt::Checked)
        ui_.o_mx_size_spin->setDisabled(lock);

    ui_.o_mx_length_toggle->setDisabled(lock);
    if(ui_.o_mx_length_toggle->checkState() == Qt::Checked)
        ui_.o_mx_length_spin->setDisabled(lock);

    ui_.o_compression_toggle->setDisabled(lock);

    ui_.o_compression_format_label->setDisabled(lock);
    ui_.o_compressio_mode_label->setDisabled(lock);

    if(ui_.o_compression_toggle->checkState() == Qt::Checked){
        ui_.o_compression_format_cbox->setDisabled(lock);
        ui_.o_compression_mode_cbox->setDisabled(lock);
    }

    for(auto &pair : set_item_hash_){
        pair.set_button->setDisabled(lock);
    }
}

void BagRecorder::updateFreeSpace(){
    qint64 free_bytes = q_storage_info_.bytesFree();
    auto free_byte_string = toStringFileSize(free_bytes);
    ui_.free_space_indicator->setText(QString::fromStdString(free_byte_string));

    QColor label_backgorund_color;
    QPalette palette = ui_.free_space_indicator->palette();

    if(free_bytes > (qint64) 5000000000LL){
        label_backgorund_color.setHsl(78, 128, 190, 255);
    }
    else if(free_bytes > (qint64) 1000000000LL){
        label_backgorund_color.setHsl(42, 128, 190, 255);     
    }
    else{
        label_backgorund_color.setHsl(0, 128, 190, 255);
    }
    palette.setColor(QPalette::Window, label_backgorund_color);
    ui_.free_space_indicator->setAutoFillBackground(true);
    ui_.free_space_indicator->setPalette(palette);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rqt_bag_recorder::BagRecorder, rqt_gui_cpp::Plugin)
