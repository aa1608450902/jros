#include "TopWnd.h"
#include <QPalette>
#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>
#include <iostream>
using namespace std;

#define HEAD_HEIGHT 60
#define CTRL_WIDTH 500
#define GAP 10
#define AUTO_ADJUST_LEN 50
#define HEAD_BUTTON_LEN 40
#define BORDER_LEN 10

TopWnd::TopWnd(QWidget *parent) : QWidget(parent) {
    setAutoFillBackground(true);
    QPalette plt;
    plt.setColor(QPalette::Background, QColor(0xff,0xff,0xfb));
    setPalette(plt);
//	_ctrlWnd = new CtrlWnd(this);
    settings = new PressButton("/home/aiyo/jpro-2/widget/setting-1.png", this);
    ptzButton = new PressButton("/home/aiyo/jpro-2/widget/ptz.png", this);
	_viewWnd = new ViewWnd(this);
	manageWnd = new ManageWnd(this);
	viewWndAni = new QPropertyAnimation(_viewWnd, "pos");
    manageWndAni = new QPropertyAnimation(manageWnd, "pos");
//	_ctrlWnd->setGeometry(0,0, CTRL_WIDTH, QApplication::desktop()->height());
//	_viewWnd->setGeometry(CTRL_WIDTH+GAP, 0, QApplication::desktop()->width()-CTRL_WIDTH-GAP, QApplication::desktop()->height());
	_viewWnd->setGeometry(0, 0, QApplication::desktop()->width(), QApplication::desktop()->height());
	manageWnd->setGeometry(0 - manageWnd->width(), HEAD_HEIGHT, manageWnd->width(), manageWnd->height());
//	settings->setGeometry(QApplication::desktop()->width() - settings->width(), 0, settings->width(), settings->height());
	settings->setGeometry(0, 0, settings->width(), settings->height());
	ptzButton->setGeometry(settings->width(), 0, ptzButton->width(), ptzButton->height());
//	_viewWnd.
//    settings->j_setPosition(QApplication::desktop()->width() - HEAD_HEIGHT / 2 * 3, 0);
//	settings->setGeometry(QApplication::desktop()->width() - HEAD_HEIGHT / 2 * 3, 0, HEAD_HEIGHT / 2 * 3, HEAD_HEIGHT);
//	viewWndPos.setX(CTRL_WIDTH+GAP); viewWndPos.setY(0);
	viewWndPos.setX(0); viewWndPos.setY(0);
	viewWndAni->setDuration(500);
	viewWndAni->setEasingCurve(QEasingCurve::InOutQuad);
	manageWndAni->setDuration(500);
    manageWndAni->setEasingCurve(QEasingCurve::OutCurve);

	QObject::connect(_viewWnd, SIGNAL(moveV(int)), this, SLOT(viewWndMoveV(int)));
	QObject::connect(_viewWnd, SIGNAL(autoAdjust()), this, SLOT(viewAutoAdjust()));
//	QObject::connect(_viewWnd, SIGNAL(displayCtrlWnd()), this, SLOT(displayCtrlWnd()));
    QObject::connect(settings, SIGNAL(clicked()), this, SLOT(manageWndDisplay()));
    logo = new QImage;
    logo->load("/home/aiyo/jpro-2/widget/LOGO3.svg");
}

void TopWnd::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.drawImage(QRect((width() - logo->width()) / 2, (height() - logo->height()) / 2, logo->width(), logo->height()), *logo);
//	_ctrlWnd->setGeometry(0,0, CTRL_WIDTH, height());
//	_viewWnd->setGeometry(viewWndPos.x(), viewWndPos.y(), width()-CTRL_WIDTH-GAP, height());
//	cout << "top width: " << width() << endl;
//	cout << "top height: " << height() << endl;
//	cout << "ctrl wnd width: " << _ctrlWnd->width() << endl;
//	cout << "ctrl wnd height: " << _ctrlWnd->height() << endl;
}

void TopWnd::viewWndMoveV(int dis) {
//    cout << "+++ y: " << _viewWnd->y() << endl;
//	if (_viewWnd->y() >= HEAD_HEIGHT || _viewWnd->y() < 0) return;
	viewWndPos.setY(viewWndPos.y() + dis);
    _viewWnd->setGeometry(0, viewWndPos.y(), width(), height());
	repaint();
//	cout << "---1" << endl;
}

void TopWnd::viewAutoAdjust() {
//	viewWndAni->setDirection();
    viewWndAni->setStartValue(QPoint(_viewWnd->x(), _viewWnd->y()));
    if (_viewWnd->y() > AUTO_ADJUST_LEN && !displayedViewWnd) {
        viewWndAni->setEndValue(QPoint(0, HEAD_HEIGHT));
        displayedViewWnd = true;
    } else {
        viewWndAni->setEndValue(QPoint(0, 0));
        displayedViewWnd = false;
        if (displayedManageWnd) displayCtrlWnd();
    }
	viewWndAni->start();
}

void TopWnd::displayCtrlWnd() {
    viewWndAni->setStartValue(QPoint(_viewWnd->x(), _viewWnd->y()));
    if (_viewWnd->y() < 5) {
        viewWndAni->setEndValue(QPoint(0, HEAD_HEIGHT));
    } else {
        viewWndAni->setEndValue(QPoint(0, 0));
        if (displayedManageWnd) manageWndDisplay();
    }
    viewWndAni->start();
}

void TopWnd::manageWndDisplay() {
    if (displayedManageWnd) {
        manageWndAni->setStartValue(QPoint(manageWnd->x(), manageWnd->y()));
        manageWndAni->setEndValue(QPoint(0 - manageWnd->width(), manageWnd->y()));
    } else {
        manageWndAni->setStartValue(QPoint(manageWnd->x(), manageWnd->y()));
        manageWndAni->setEndValue(QPoint(0, manageWnd->y()));
    }
    displayedManageWnd = !displayedManageWnd;
    manageWndAni->start();
}