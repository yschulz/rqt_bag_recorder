#include "rqt_bag_recorder/rqt_bag_recorder.hpp"

namespace rqt_bag_recorder{

BagRecorder::BagRecorder(){}
BagRecorder::~BagRecorder(){}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rqt_bag_recorder::BagRecorder, rqt_gui_cpp::Plugin)
