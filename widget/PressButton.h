#ifndef __PRESSBUTTON_H__
#define __PRESSBUTTON_H__

#include <QWidget>
#include <QSize>
//#include <QPropertyAnimation>

class PressButton : public QWidget {
    Q_OBJECT
public:
    explicit PressButton(QWidget *parent = nullptr);
    virtual ~PressButton() = default;
//    void j_setPosition(int x, int y);
//    void j_setSize(int w, int h);
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
//    void enterEvent(QEvent *event);
//    void leaveEvent(QEvent *event);
signals:
    void clicked();
private slots:
    void onClicked();
private:
    QPoint imagePosition;
    QImage *image;
//    QPropertyAnimation *ani;
};

#endif /* __PRESSBUTTON_H__ */