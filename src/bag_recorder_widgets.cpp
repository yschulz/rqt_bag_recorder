#include "rqt_bag_recorder/bag_recorder_widgets.hpp"

#include <chrono>
#include <cmath>

#include <QTimer>
#include <iostream>

namespace rqt_bag_recorder{

CPBar::CPBar(QWidget * parent) : QWidget(parent), progress_(0) {
    QWidget::setMinimumSize(box_size_, box_size_);

    progress_pen_.setCapStyle(Qt::FlatCap);
    progress_pen_.setColor(QColor("#30b7e0"));
    progress_pen_.setWidth(pen_stroke_);

    placeholder_pen_.setWidth(pen_stroke_);
    placeholder_pen_.setColor(QColor("#d7d7d7"));
    placeholder_pen_.setCapStyle(Qt::FlatCap);
    placeholder_pen_.setDashPattern(QVector<qreal>{0.5, 1.105});
    placeholder_pen_.setDashOffset(2.2);
}

void CPBar::drawProgress(qreal new_progress) {
    if (progress_ == new_progress) return;
    if(new_progress < -0.01 || new_progress > 1.01) return;

    progress_ = new_progress;
    QWidget::update();
}

void CPBar::paintEvent(QPaintEvent *) {
    qreal pd = progress_ * 360;
    qreal rd = 360 - pd;
    QPainter painter(this);

    painter.translate(painter_offset_ / 2, painter_offset_ / 2);
    painter.setRenderHint(QPainter::Antialiasing);

    QPainterPath progress_path, placeholder_path;

    progress_path.moveTo(box_size_ / 2, 0);
    progress_path.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, -pd);

    placeholder_path.moveTo(box_size_ / 2, 0);
    placeholder_path.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, rd);
    
    if(progress_ != 0)
        painter.strokePath(progress_path, progress_pen_);
    painter.strokePath(placeholder_path, placeholder_pen_);
}



RecordDot::RecordDot(QWidget * parent) : QWidget(parent){
    QWidget::setMinimumSize(box_size_, box_size_);
    record_ = 0;

    timer_ = new QTimer(this);
    timer_->setInterval(100);

    connect(timer_, SIGNAL(timeout()), this, SLOT(update()));

    ellipse_path_.addEllipse(QRect(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_));

    circle_path_.moveTo(box_size_ / 2, 0);
    circle_path_.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, 360);
}

void RecordDot::setRecordStatus(int record){
    record_ = record;
    QWidget::update();

    if(record)
        timer_->start();
    else
        timer_->stop();

}

void RecordDot::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.translate(painter_offset_ / 2, painter_offset_ / 2);

    QBrush ellipse_brush;
    QRadialGradient radial_gradient(QPointF((box_size_ - painter_offset_) / 2, (box_size_ - painter_offset_) / 2), (box_size_ - painter_offset_) / 2);
    QPen pen;
    pen.setWidth(pen_stroke_);

    if(record_){
        QColor full, dark, light;

        // set red_light based on time
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        double amplitude = 30;
        double period = milliseconds * M_PI / 1000;

        uint8_t light_value = 190 + std::sin(period) * amplitude;

        if(record_ == 1){
            full.setHsl(0, 255, 127, 255);
            dark.setHsl(0, 255, 64, 255);
            light.setHsl(0, 255, light_value, 255);
        }
        else if(record_ == 2){
            full.setHsl(210, 255, 127, 255);
            dark.setHsl(210, 255, 64, 255);
            light.setHsl(210, 255, light_value, 255);
        }


        ellipse_brush = QBrush(full);

        radial_gradient.setColorAt(0, light);
        radial_gradient.setColorAt(0.5, light);
        radial_gradient.setColorAt(1, full);

        pen.setColor(dark);
    }
    else{
        QColor gray, gray_dark, gray_light;

        gray.setHsl(0, 0, 127, 255);
        gray_dark.setHsl(0, 0, 64, 255);
        gray_light.setHsl(0, 0, 190, 255);

        ellipse_brush = QBrush(gray);

        radial_gradient.setColorAt(0, gray_light);
        radial_gradient.setColorAt(0.5, gray_light);
        radial_gradient.setColorAt(1, gray);

        pen.setColor(gray_dark);
    }

    painter.fillPath(ellipse_path_, radial_gradient);
    painter.strokePath(circle_path_, pen);
}

}