#include "TopButton.h"
#include <QPainter>

#define BORDER_LEN 10


TopButton::TopButton(const QString& text, QWidget *parent) : QWidget(parent) {
    resize(parent->width(), 30);
    setAutoFillBackground(true);
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
    setPalette(plt);
    image.load("/home/aiyo/jpro-2/widget/fold.png");
    label = new QLabel(this);
    auto plt2 = label->palette();
    plt2.setColor(QPalette::WindowText,Qt::white);
    label->setFont(QFont("Helvetica"));
    label->setPalette(plt2);
    label->setText(text);
    label->setGeometry(BORDER_LEN * 3, (30 - label->height()) / 2, label->width(), label->height());
}

void TopButton::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.drawPixmap(QRect(0,6, 20,20), QPixmap::fromImage(image));
}

void TopButton::mousePressEvent(QMouseEvent *event) {
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x8a, 0x8c, 0x8e));
    setPalette(plt);
}
void TopButton::mouseReleaseEvent(QMouseEvent *event) {
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
    setPalette(plt);
    emit clicked();
    QMatrix matrix;
    if (folded) {
        matrix.rotate(90);
        image = image.transformed(matrix);
        folded = false;
    } else {
        matrix.rotate(-90);
        image = image.transformed(matrix);
        folded = true;
    }
    repaint();
}