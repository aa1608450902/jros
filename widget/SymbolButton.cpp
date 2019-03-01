#include "SymbolButton.h"
#include <QPainter>
#include <iostream>
using namespace std;

SymbolButton::SymbolButton(QWidget *parent, bool flag) : QPushButton(parent) {
    resize(50, 50);
//    setAutoFillBackground(true);
//    auto plt = palette();
//    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
//    setPalette(plt);
    image = new QImage();
    if (!flag) {
        image->load("/home/aiyo/jpro-2/widget/plus.png");
    } else {
        image->load("/home/aiyo/jpro-2/widget/sub.png");
    }
}

void SymbolButton::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.drawPixmap(QRect(0, 0, height(), height()), QPixmap::fromImage(*image));
}

void SymbolButton::mousePressEvent(QMouseEvent *event) {
//    auto plt = palette();
//    plt.setColor(QPalette::Background, QColor(0x8a, 0x8c, 0x8e, 0xff));
//    setPalette(plt);
    emit moveMini();
//    emit click();
}
void SymbolButton::mouseReleaseEvent(QMouseEvent *event) {
//    auto plt = palette();
//    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
//    setPalette(plt);
    emit recoverMini();
    emit clicked();
}

//void SymbolButton::focusInEvent(QFocusEvent *) {
//    auto plt = palette();
//    plt.setColor(QPalette::Background, QColor(0x8a, 0x8c, 0x8e));
//    setPalette(plt);
//    cout << "---in" << endl;
//}
//void SymbolButton::focusOutEvent(QFocusEvent *) {
//    auto plt = palette();
//    plt.setColor(QPalette::Background, Qt::transparent);
//    setPalette(plt);
//    cout << "---out" << endl;
//}