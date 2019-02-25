#include "ViewWnd.h"
#include <QPalette>
#include "HCNetSDK.h"
#include "header.h"
#include "global.h"

ViewWnd::ViewWnd(QWidget *parent) : QWidget(parent) {
    setGeometry(0, 50, parent->width(), parent->height() - HEADWND_HEIGHT);

//    _label = new QLabel("view window", this);
    _viewArea   = new ViewArea(this);
    _ctrlArea   = new CtrlArea(this);
    _singleWnd  = new QPushButton("single", this);
    _multiWnd   = new QPushButton("multi", this);
    _stopView   = new QPushButton("interrupt", this);
    _startView  = new QPushButton("start", this);
    _fullScreen = new QPushButton("F", this);

    _ctrlAreaWidght = CTRL_WIDTH_1;
    _viewArea->setGeometry(0, 0, width() - _ctrlAreaWidght, height() - BOTTOM_HEIGHT);
    _ctrlArea->setGeometry(width() - _ctrlAreaWidght, 0, _ctrlAreaWidght, height());
    _stopView->setGeometry(10, height() - BOTTOM_HEIGHT + 10, 100, 30);
    _startView->setGeometry(120, height() - BOTTOM_HEIGHT + 10, 100, 30);
    _fullScreen->setGeometry(230, height() - BOTTOM_HEIGHT + 10, 30, 30);
    _singleWnd->setGeometry(width() - _ctrlAreaWidght - 220, height() - BOTTOM_HEIGHT + 10, 100, 30);
    _multiWnd->setGeometry(width() - _ctrlAreaWidght - 110, height() - BOTTOM_HEIGHT + 10, 100, 30);

    QObject::connect(_fullScreen, SIGNAL(clicked()), _viewArea, SLOT(fullScreen()));
    QObject::connect(dynamic_cast<CtrlArea *>(_ctrlArea)->getFoldButton(), SIGNAL(clicked()), this, SLOT(ctrlWndFold()));
    QObject::connect(_startView, SIGNAL(clicked()), this, SLOT(play()));
    QObject::connect(_stopView, SIGNAL(clicked()), this, SLOT(stopPlay()));

//    setMinimumSize(390, 300);
//    resize();
//    _viewArea->resize(this->width() - BORDER_WIDTH, this->height() - BOTTOM_HEIGHT);
//    QPalette qPalette(_viewArea->palette());
//    qPalette.setColor(QPalette::Background, Qt::black);
//    _viewArea->setAutoFillBackground(true);
//    _viewArea->setPalette(qPalette);

//    _label->setGeometry(0, 0, 100, 20);
//    _stopView->setGeometry(BORDER_WIDTH / 2, this->height() - BOTTOM_HEIGHT, 100, 30);
//    _singleWnd->setGeometry(BORDER_WIDTH / 2 + 120, this->height() - BOTTOM_HEIGHT, 100, 30);
//    _multiWnd->setGeometry(BORDER_WIDTH / 2 + 240, this->height() - BOTTOM_HEIGHT, 100, 30);
//    _glayout = new QGridLayout(this);
//    _glayout->addWidget(_viewArea, 0, 0, 3, 4);
//    _glayout->addWidget(_label, 0, 0, 1, 1);
//    _glayout->addWidget(_stopView, 3, 0, 1, 1);
//    _glayout->addWidget(_singleWnd, 3, 2, 1, 1);
//    _glayout->addWidget(_multiWnd, 3, 3, 1, 1);
    QObject::connect(_singleWnd, SIGNAL(clicked()), _viewArea, SLOT(singleWnd()));
    QObject::connect(_multiWnd, SIGNAL(clicked()), _viewArea, SLOT(multiWnd()));
    link();
}

ViewWnd::~ViewWnd() {
//	delete _label;
    cleanup();
}

void ViewWnd::cleanup() {
    auto& links = gLinkInfo();
    auto& rps = gRealPlayHandle();
    auto& playWnds = dynamic_cast<ViewArea *>(_viewArea)->getPlayWnds();
    int len = (links.length() < playWnds.length() ? links.length() : playWnds.length());
    for (int i = 0; i < len; i++) {
        NET_DVR_StopRealPlay(rps[i].realHandle);
        NET_DVR_Logout(rps[i].userId);
    }
    NET_DVR_Cleanup();
}

void ViewWnd::paintEvent(QPaintEvent *event) {
    _viewArea->setGeometry(0, 0, width() - _ctrlAreaWidght, height() - BOTTOM_HEIGHT);
    _ctrlArea->setGeometry(width() - _ctrlAreaWidght, 0, _ctrlAreaWidght, height());
    _stopView->setGeometry(10, height() - BOTTOM_HEIGHT + 10, 100, 30);
    _singleWnd->setGeometry(width() - _ctrlAreaWidght - 220, height() - BOTTOM_HEIGHT + 10, 100, 30);
    _multiWnd->setGeometry(width() - _ctrlAreaWidght - 110, height() - BOTTOM_HEIGHT + 10, 100, 30);
}

void ViewWnd::link() {
    /// init Hik SDK
    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);

    auto& links = gLinkInfo();
    auto& rps = gRealPlayHandle();
    if (links.length() == 0) {
        std::cout << "[ vsnvr ] No device link!" << std::endl;
        return;
    }
    for (int i = 0; i < links.length(); i++) {
        NET_DVR_DEVICEINFO_V30 struDeviceInfo{0};
        int userId = NET_DVR_Login_V30(
                const_cast<char *>(links[i].ip.c_str()), links[i].port,
                const_cast<char *>(links[i].username.c_str()),
                const_cast<char *>(links[i].password.c_str()), &struDeviceInfo);
        rps[i].userId = userId;
        std::cout << "[ vsnvr ] login status ip - " << links[i].ip << ", user id: " << userId << std::endl;
    }
//    char *ip = "192.168.1.250";
//    char *user = "admin";
//    char *pass = "rc123456";
//    NET_DVR_DEVICEINFO_V30 struDeviceInfo{0};
//    _userID = NET_DVR_Login_V30(ip, 8000, user, pass, &struDeviceInfo);
//    std::cout << "[ vsnvr ] user id: " << _userID << std::endl;

//    QList<RealPlayHandle>& rps = gRealPlayHandle();
//    std::cout << "----1 rps len: " << rps.length() << std::endl;
//    rps.push_back(RealPlayHandle(0, 1));
//    rps.push_back(RealPlayHandle(1, 2));
}

//QFrame* ViewWnd::getFrame() {
//
//}

void ViewWnd::singleWnd() {
//    play();
}

void ViewWnd::ctrlWndFold() {
    if (_ctrlAreaWidght == CTRL_WIDTH_2) {
        _ctrlAreaWidght = CTRL_WIDTH_1;
        dynamic_cast<CtrlArea *>(_ctrlArea)->getFoldButton()->setText("<");
    } else {
        _ctrlAreaWidght = CTRL_WIDTH_2;
        dynamic_cast<CtrlArea *>(_ctrlArea)->getFoldButton()->setText(">");
    }
    repaint();
}

void ViewWnd::stopPlay() {
    auto& links = gLinkInfo();
    auto& rps = gRealPlayHandle();
    auto& playWnds = dynamic_cast<ViewArea *>(_viewArea)->getPlayWnds();
    int len = (links.length() < playWnds.length() ? links.length() : playWnds.length());
    for (int i = 0; i < len; i++) {
        NET_DVR_StopRealPlay(rps[i].realHandle);
        playWnds[i]->repaint();
    }
}

void ViewWnd::play() {
    std::cout << "[ vsnvr ] execute real play task" << std::endl;
    auto& links = gLinkInfo();
    auto& rps = gRealPlayHandle();
    auto& playWnds = dynamic_cast<ViewArea *>(_viewArea)->getPlayWnds();
    int len = (links.length() < playWnds.length() ? links.length() : playWnds.length());
    for (int i = 0; i < len; i++) {
        HWND hWnd = static_cast<HWND>(playWnds[i]->winId());
        NET_DVR_PREVIEWINFO struPlayInfo{0};
        struPlayInfo.hPlayWnd = hWnd;
        struPlayInfo.lChannel = links[i].channel;
        struPlayInfo.dwStreamType = 0;
        struPlayInfo.dwLinkMode = 0;
        struPlayInfo.bBlocked = 1;
        rps[i].realHandle = NET_DVR_RealPlay_V40(rps[i].userId, &struPlayInfo, NULL, NULL);
    }
//    NET_DVR_PREVIEWINFO struPlayInfo{0};
//    ViewArea *pview = dynamic_cast<ViewArea *>(_viewArea);
//    HWND hWnd = static_cast<HWND>(pview->getPlayWnd()->winId());
//    struPlayInfo.hPlayWnd = hWnd;
//    struPlayInfo.lChannel = 33;
//    struPlayInfo.dwStreamType = 0;
//    struPlayInfo.dwLinkMode = 0;
//    struPlayInfo.bBlocked = 1;
//    _realPlayHandle = NET_DVR_RealPlay_V40(_userID, &struPlayInfo, NULL, NULL);
//    if (_realPlayHandle == -1) {
//        QMessageBox msg;
//        msg.setText("Link failed! Please check camera device and link info.");
//        msg.exec();
//    }
//    std::cout << "[ vsnvr ] real play over" << std::endl;
}