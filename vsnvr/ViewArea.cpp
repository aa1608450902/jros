#include "ViewArea.h"
#include "header.h"
#include <QPalette>
#include <QKeyEvent>

ViewArea::ViewArea(QWidget *parent) : QWidget(parent) {
//    QPalette qPalette(_playWnd->palette());
    QPalette qPalette;
    qPalette.setColor(QPalette::Background, Qt::black);
    for (int i = 0; i < 4; i++) {
        auto p = new QFrame(this);
        p->setAutoFillBackground(true);
        p->setPalette(qPalette);
        _playWnds.push_back(p);
        if (i == 0) {
            p->setGeometry(4, 4, (width() - 12) / 2, (height() - 12) / 2);
        } else {
            p->resize(0, 0);
        }
    }
    QObject::connect(this, SIGNAL(exitFullScreen()), this, SLOT(exitFullScreenP()));
//    _playWnd = _playWnds.at(0);

//    _playWnd->setAutoFillBackground(true);
//    _playWnd->setPalette(qPalette);
//    _playWnds.push_back(_playWnd);
//    _playWnd->resize(this->width(), this->height());
//    std::cout << "[ vsnvr ] view area width: " << width() << ", height: " << height() << std::endl;
}

ViewArea::~ViewArea() {

}

void ViewArea::paintEvent(QPaintEvent *event) {
//    std::cout << "[ vsnvr ] viewarea paint event" << std::endl;
    if (_wndCount == 1) {
//        std::cout << "[ vsnvr ] 1 view wnd" << std::endl;
        _playWnds[0]->setGeometry(0, 0, width(), height());
        _playWnds[1]->resize(0, 0);
        _playWnds[2]->resize(0, 0);
        _playWnds[3]->resize(0, 0);
    } else if (_wndCount == 4) {
//        std::cout << "[ vsnvr ] 4 view wnd" << std::endl;
        _playWnds[0]->setGeometry(MULTI_WND_BORDER, MULTI_WND_BORDER, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[1]->setGeometry(MULTI_WND_BORDER * 2 + (width() - MULTI_WND_BORDER * 3) / 2, MULTI_WND_BORDER, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[2]->setGeometry(MULTI_WND_BORDER, MULTI_WND_BORDER * 2 + (height() - MULTI_WND_BORDER * 3) / 2, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[3]->setGeometry(MULTI_WND_BORDER * 2 + (width() - MULTI_WND_BORDER * 3) / 2, MULTI_WND_BORDER * 2 + (height() - MULTI_WND_BORDER * 3) / 2, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
    }
}

void ViewArea::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Escape) {
        emit exitFullScreen();
    }
}

void ViewArea::singleWnd() {
    std::cout << "[ vsnvr ] Single one window mode." << std::endl;
    if (_wndCount != 1) {
        _playWnds[0]->setGeometry(0, 0, width(), height());
        _playWnds[1]->resize(0, 0);
        _playWnds[2]->resize(0, 0);
        _playWnds[3]->resize(0, 0);
    }
    _wndCount = 1;
    repaint();
}

void ViewArea::multiWnd() {
    std::cout << "[ vsnvr ] Fourth window mode." << std::endl;
    if (_wndCount != 4) {
        _playWnds[0]->setGeometry(MULTI_WND_BORDER, MULTI_WND_BORDER, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[1]->setGeometry(MULTI_WND_BORDER * 2 + (width() - MULTI_WND_BORDER * 3) / 2, MULTI_WND_BORDER, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[2]->setGeometry(MULTI_WND_BORDER, MULTI_WND_BORDER * 2 + (height() - MULTI_WND_BORDER * 3) / 2, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
        _playWnds[3]->setGeometry(MULTI_WND_BORDER * 2 + (width() - MULTI_WND_BORDER * 3) / 2, MULTI_WND_BORDER * 2 + (height() - MULTI_WND_BORDER * 3) / 2, (width() - MULTI_WND_BORDER * 3) / 2, (height() - MULTI_WND_BORDER * 3) / 2);
    }
    _wndCount = 4;
    repaint();
}

void ViewArea::fullScreen() {
    _beginFullScreenWidth = width();
    _beginFullScreenHeight = height();
    if (!isFullScreen()) {
        setWindowFlags(Qt::Window);
        showFullScreen();
    }
}

void ViewArea::exitFullScreenP() {
    setWindowFlags(Qt::SubWindow);
    showNormal();
    resize(_beginFullScreenWidth, _beginFullScreenHeight);
}
