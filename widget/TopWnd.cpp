#include "TopWnd.h"
#include <QPalette>
#include <QApplication>
#include <QDesktopWidget>
#include <iostream>
using namespace std;

#define CTRL_WIDTH 500
#define GAP 0
#define HEAD_HEIGHT 300

TopWnd::TopWnd(QWidget *parent) : QWidget(parent) {
//	QPalette p(palette());
//	p.setColor(QPalette::Background, Qt::black);
//	setPalette(p);
	_ctrlWnd = new CtrlWnd(this);
	_viewWnd = new ViewWnd(this);
	_ctrlWnd->setGeometry(0,0, CTRL_WIDTH, QApplication::desktop()->height());
	_viewWnd->setGeometry(CTRL_WIDTH+GAP, 0, QApplication::desktop()->width()-CTRL_WIDTH-GAP, QApplication::desktop()->height());
	viewWndPos.setX(CTRL_WIDTH+GAP); viewWndPos.setY(0);
	QObject::connect(_viewWnd, SIGNAL(moveV(int)), this, SLOT(viewWndMoveV(int)));
}

//void TopWnd::paintEvent(QPaintEvent *event) {
//	_ctrlWnd->setGeometry(0,0, CTRL_WIDTH, height());
//	_viewWnd->setGeometry(viewWndPos.x(), viewWndPos.y(), width()-CTRL_WIDTH-GAP, height());

//	cout << "top width: " << width() << endl;
//	cout << "top height: " << height() << endl;
//	cout << "ctrl wnd width: " << _ctrlWnd->width() << endl;
//	cout << "ctrl wnd height: " << _ctrlWnd->height() << endl;
//}

void TopWnd::viewWndMoveV(int dis) {
//    cout << "+++ y: " << _viewWnd->y() << endl;
//	if (_viewWnd->y() >= HEAD_HEIGHT || _viewWnd->y() < 0) return;
	viewWndPos.setY(viewWndPos.y() + dis);
    _viewWnd->setGeometry(CTRL_WIDTH+GAP, viewWndPos.y(), width()-CTRL_WIDTH-GAP, height());
	repaint();
//	cout << "---1" << endl;
}