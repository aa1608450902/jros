#include "HeadWnd.h"
#include "header.h"
#include <QImage>
#include <QPixmap>

HeadWnd::HeadWnd(QWidget *parent) : QWidget(parent) {
	resize(parent->width(), HEADWND_HEIGHT);

    _label = new QLabel(this);
    _wndSwitch = new QPushButton("view / link", this);
    // _layout = new QHBoxLayout(this);

//    _label->setGeometry(10, 10, 100, 30);
    _wndSwitch->setGeometry(width() - 160, 10, 150, 30);

    QImage image;
    image.load("/home/aiyo/jpro-2/vsnvr/LOGO/LOGO2.svg");
    image = image.scaled(140, 50);
    _label->setPixmap(QPixmap::fromImage(image));
    _label->setGeometry(10, 0, 140, 50);
//    _label->resize(120, 50);
//    std::cout << "label x: " << _label->x() << ", y: " << _label->y() << std::endl;
    // _layout->addWidget(_label);
    // _layout->addWidget(_wndSwitch);
}

HeadWnd::~HeadWnd() {}

QPushButton* HeadWnd::getSwitchButton() const {
    return _wndSwitch;
}

void HeadWnd::paintEvent(QPaintEvent *event) {
    _label->setGeometry(10, 0, 140, 50);
}
