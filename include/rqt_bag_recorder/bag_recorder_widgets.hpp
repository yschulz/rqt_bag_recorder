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
        void drawProgress(qreal new_progress);
        void paintEvent(QPaintEvent *);

    private:
        // progress 0.0 to 1.0
        qreal progress_; 

        QPen progress_pen_;
        QPen placeholder_pen_;

        int box_size_ = 25;
        int painter_offset_ = 6;
        int pen_stroke_ = 4;
};

class RecordDot : public QWidget {
    Q_OBJECT
    public:
        RecordDot(QWidget * parent = nullptr);
        void paintEvent(QPaintEvent *);


    protected slots:
        void setRecordStatus(int record);

    private:
        QTimer* timer_;

        QPen outline_pen_;
        QBrush background_brush_;

        QPainterPath ellipse_path_;
        QPainterPath circle_path_;
        
        int record_;
        int box_size_ = 25;
        int painter_offset_ = 6;
        int pen_stroke_ = 3;
        
};

}

#endif