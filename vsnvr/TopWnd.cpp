#include "TopWnd.h"

#include "HeadWnd.h"
#include "ViewWnd.h"
#include "ManageWnd.h"
#include "HCNetSDK.h"
#include "header.h"
#include "global.h"
#include <QIcon>

TopWnd::TopWnd(QWidget *parent) : QWidget(parent) {
	/// step 1. window self attributes
	/// step 2. create object
	/// step 3. set object attributes
	showFullScreen();
//	setFixedSize(1200, 800);
	setWindowIcon(QIcon("/home/aiyo/jpro-2/vsnvr/logo.jpg"));

    auto& links = gLinkInfo();
    links.push_back(LinkInfo("", "192.168.1.250", 8000, 33, "admin", "rc123456"));
    auto& rps = gRealPlayHandle();
    rps.push_back(RealPlayHandle());

    _headWnd    = new HeadWnd(this);
	_viewWnd    = new ViewWnd(this);
	_manageWnd  = new ManageWnd(this);

	_headWnd->setGeometry(0, 0, width(), HEADWND_HEIGHT);
	_viewWnd->setGeometry(0, HEADWND_HEIGHT, width(), height() - HEADWND_HEIGHT);
	_manageWnd->setGeometry(0, HEADWND_HEIGHT, width(), height() - HEADWND_HEIGHT);
//	std::cout << "[ vsnvr ] manage wnd width: " << width() << ", height: " << height() -HEADWND_HEIGHT << std::endl;
//	_layout = new QVBoxLayout(this);
//	_gLayout = new QGridLayout(this);
//	_gLayout->addWidget(_headWnd, 0, 0, 1, 3);
//	_gLayout->addWidget(_viewWnd, 1, 0, 5, 3);
//	_gLayout->addWidget(_manageWnd, 1, 0, 5, 3);
//	_layout->addWidget(_headWnd);
//	_layout->addWidget(_bodyWnd);
//	_layout->addWidget(_viewWnd);
//	_layout->addWidget(_manageWnd);
	_viewWnd->show();
	_manageWnd->hide();
	QObject::connect(dynamic_cast<HeadWnd *>(_headWnd)->getSwitchButton(), SIGNAL(clicked()), this, SLOT(switchWnd()));

//	QList<RealPlayHandle>& rps = gRealPlayHandle();
//	std::cout << "----2 rps len: " << rps.length() << std::endl;
//	for (auto& i : rps) {
//		std::cout << "\t user id: " << i.userId << ", real handle:" << i.realHandle << std::endl;
//	}
}

TopWnd::~TopWnd() {
	delete _viewWnd;
	delete _manageWnd;
}

void TopWnd::switchWnd() {
	if (_viewWnd->isHidden()) {
		_manageWnd->hide();
		_viewWnd->show();
	} else {
		_viewWnd->hide();
		_manageWnd->show();
	}

}

void TopWnd::fullScreen() {
    showFullScreen();
}

void TopWnd::maximizedScreen() {
//	std::cout << "[ vsnvr ] maximized screen" << std::endl;
//    showMaximized();
}

//void TopWnd::play() {
//    HWND hWnd;
//    NET_DVR_PREVIEWINFO struPlayInfo;
//    hWnd=viewFrame->winId();
//}

void TopWnd::paintEvent(QPaintEvent *event) {
//	_headWnd->setGeometry(0, 0, this->width(), HEAD_HEIGHT);
//	_viewWnd->setGeometry(0, HEAD_HEIGHT, this->width(), height() - HEAD_HEIGHT);
//	_manageWnd->setGeometry(0, HEAD_HEIGHT, this->width(), height() - HEAD_HEIGHT);
//	std::cout << "[ vsnvr ] width: " << width() << ", height: " << this->height() << std::endl;
}

//void TopWnd::link() {
//    NET_DVR_DEVICEINFO_V30 struDeviceInfo{0};
//    int userID = NET_DVR_Login_V30("192.168.1.250", 8000, "admin", "rc123456", &struDeviceInfo);
//    std::cout << "[ vsnvr ] user id: " << userID << std::endl;
//}