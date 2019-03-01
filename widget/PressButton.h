#ifndef __PRESSBUTTON_H__
#define __PRESSBUTTON_H__

#include <QWidget>
#include <QSize>
//#include <QPropertyAnimation>

class PressButton : public QWidget {
    Q_OBJECT
public:
    explicit PressButton(const QString& pic, QWidget *parent = nullptr);
    virtual ~PressButton() = default;
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
signals:
    void clicked();
private slots:
    void onClicked();
private:
    QPoint imagePosition;
    QImage *image;
};

#endif /* __PRESSBUTTON_H__ */