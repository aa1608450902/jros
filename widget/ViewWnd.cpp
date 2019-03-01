#include "ViewWnd.h"
#include <QPalette>
#include <QColor>
#include <QMouseEvent>
#include <QtMath>
#include <iostream>
using namespace std;

#define MIN_MOVE 10

ViewWnd::ViewWnd(QWidget *parent) : QWidget(parent) {
	// setFixedSize(900, 600);
	setAutoFillBackground(true);
	QPalette p(palette());
	p.setColor(QPalette::Background, QColor(0x30,0x30,0x30));
	setPalette(p);
	cout << "--- view wnd construct ... width: " << width() << "; height: " << height() << endl;
}

void ViewWnd::mousePressEvent(QMouseEvent *event) {
//	isMoving = true;
	cursorStartPoint.setX(event->x());
	cursorStartPoint.setY(event->y());
	cout << "moving true" << endl;
}

void ViewWnd::mouseReleaseEvent(QMouseEvent *event) {
	cout << "moving false" << endl;
	if (isMoving) {
	    emit autoAdjust();
	}
	isMoving = false;
}

void ViewWnd::mouseDoubleClickEvent(QMouseEvent *event) {
    cout << "double clicked" << endl;
	emit displayCtrlWnd();
}

void ViewWnd::mouseMoveEvent(QMouseEvent *event) {
    cout << "--- pos: ( " << event->x() << "\t, " << event->y() << " )" << endl;
    if (qAbs(event->y() - cursorStartPoint.y()) < MIN_MOVE)
		return;
    if (!isMoving) {
		isMoving = true;
    }
    emit moveV(event->y() - cursorStartPoint.y());

//    cursorLastPoint.setX(event->x());
//    cursorLastPoint.setY(event->y());
}

//void ViewWnd::paintEvent(QPaintEvent *event) {
//	setGeometry(wndPos.x(), wndPos.y(), width(), height());
//}
