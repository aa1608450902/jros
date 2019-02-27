#ifndef __VIEWWND_H__
#define __VIEWWND_H__

#include <QWidget>
#include "CtrlWnd.h"
#include <QPoint>

class ViewWnd : public QWidget {
	Q_OBJECT
public:
	explicit ViewWnd(QWidget *parent = nullptr);
	virtual ~ViewWnd() = default;
signals:
    void moveV(int dis);
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

//    void paintEvent(QPaintEvent *event);
private:
//    QPoint wndPos;
    QPoint cursorStartPoint;
    QPoint cursorLastPoint;
    bool isMoving = false;
};

#endif /* __VIEWWND_H__ */