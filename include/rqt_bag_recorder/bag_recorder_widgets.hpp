#ifndef RQT_BAG_RECORDER__BAG_RECORDER_WIDGETS_HPP
#define RQT_BAG_RECORDER__BAG_RECORDER_WIDGETS_HPP

#include <QWidget>
#include <QPainter>
#include <QPainterPath>

namespace rqt_bag_recorder{

class CPBar : public QWidget {
    Q_OBJECT
    public:
        CPBar(QWidget * parent = nullptr);
        void drawProgress(qreal pp);
        void paintEvent(QPaintEvent *);
    private:
        qreal progress_; // progress 0.0 to 1.0
        int box_size_ = 25;
        int painter_offset_ = 6;
        int pen_stroke_ = 4;
};

class RecordDot : public QWidget {
    Q_OBJECT
    public:
        RecordDot(QWidget * parent = nullptr);
        void setRecordStatus(bool record);
        void paintEvent(QPaintEvent *);

    private:
        int box_size_ = 25;
        int painter_offset_ = 6;
        int pen_stroke_ = 3;
        bool record_;
};

}

#endif