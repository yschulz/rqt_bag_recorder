#include "rqt_bag_recorder/bag_recorder_widgets.hpp"

namespace rqt_bag_recorder{

CPBar::CPBar(QWidget * parent) : QWidget(parent), progress_(0) {
    QWidget::setMinimumSize(box_size_, box_size_);
    // QWidget::setMinimumSize(208, 208);
}

void CPBar::drawProgress(qreal pp) {
    if (progress_ == pp) return;
    progress_ = pp;
    QWidget::update();
}

void CPBar::paintEvent(QPaintEvent *) {
    qreal pd = progress_ * 360;
    qreal rd = 360 - pd;
    QPainter painter(this);
    // painter.fillRect(rect(), Qt::white);
    painter.translate(painter_offset_ / 2, painter_offset_ / 2);
    painter.setRenderHint(QPainter::Antialiasing);
    QPainterPath path, path2;

    path.moveTo(box_size_ / 2, 0);
    path.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, -pd);
    QPen pen, pen2;
    pen.setCapStyle(Qt::FlatCap);
    pen.setColor(QColor("#30b7e0"));
    pen.setWidth(pen_stroke_);
    painter.strokePath(path, pen);


    path2.moveTo(box_size_ / 2, 0);
    pen2.setWidth(pen_stroke_);
    pen2.setColor(QColor("#d7d7d7"));
    pen2.setCapStyle(Qt::FlatCap);
    pen2.setDashPattern(QVector<qreal>{0.5, 1.105});

    path2.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, rd);
    pen2.setDashOffset(2.2);
    painter.strokePath(path2, pen2);
}



RecordDot::RecordDot(QWidget * parent) : QWidget(parent){
    QWidget::setMinimumSize(box_size_, box_size_);
    record_ = false;
}

void RecordDot::setRecordStatus(bool record){
    record_ = record;
    QWidget::update();
}

void RecordDot::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.translate(painter_offset_ / 2, painter_offset_ / 2);
    // painter.drawEllipse(QRect(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_));

    QPainterPath ellipse_path;
    QBrush ellipse_brush;
    if(record_){
        ellipse_brush = QBrush(Qt::red);
    }
    else{
        ellipse_brush = QBrush(QColor("#a6a6a6"));
    }

    ellipse_path.addEllipse(QRect(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_));
    painter.fillPath(ellipse_path, ellipse_brush);
    
    QPainterPath gradient_path;

    QRadialGradient radialGrad(QPointF((box_size_ - painter_offset_) / 2, (box_size_ - painter_offset_) / 2), (box_size_ - painter_offset_) / 2);
    if(record_){
        radialGrad.setColorAt(0, QColor("#ff5959"));
        radialGrad.setColorAt(0.5, QColor("#ff5959"));
        radialGrad.setColorAt(1, Qt::red);
    }
    else{
        radialGrad.setColorAt(0, QColor("#e0e0e0"));
        radialGrad.setColorAt(0.5, QColor("#e0e0e0"));
        radialGrad.setColorAt(1, QColor("#a6a6a6"));  
    }

    painter.fillPath(ellipse_path, radialGrad);

    QPainterPath path;
    QPen pen;

    pen.setWidth(pen_stroke_);
    if(record_){
        pen.setColor(QColor("#730d0d"));
    }
    else{
        pen.setColor(QColor("#6e6e6e"));
    }
    pen.setWidth(pen_stroke_);

    auto inner_circle_offset = pen_stroke_;
    // path.moveTo(box_size_ / 2 + inner_circle_offset, inner_circle_offset);
    path.moveTo(box_size_ / 2, 0);
    path.arcTo(QRectF(0, 0, box_size_ - painter_offset_, box_size_ - painter_offset_), 90, 360);
    // path.arcTo(QRectF(0, 0, box_size_ - painter_offset_ - inner_circle_offset, box_size_ - painter_offset_ - inner_circle_offset), 90, -360);

    painter.strokePath(path, pen);
}

}