#include "PressButton.h"
#include <QPainter>
#include <QImage>
#include <QFont>
#include <QColor>
#include <QPalette>

#define HEAD_HEIGHT 60
#define GAP 10
#define MOVE_LITTLE 4

PressButton::PressButton(const QString& pic, QWidget *parent) : QWidget(parent) {
    setAutoFillBackground(true);
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0xff,0xff,0xfb));
    setPalette(plt);

    image = new QImage();
    image->load(pic);
    imagePosition.setX((HEAD_HEIGHT / 2 * 3 - HEAD_HEIGHT) / 2);
    imagePosition.setY(0);
    resize(HEAD_HEIGHT / 2 * 3, HEAD_HEIGHT);
}

void PressButton::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.setPen(Qt::NoPen);
    p.drawPixmap(QRect(imagePosition.x(), imagePosition.y(), height(), height()), QPixmap::fromImage(*image));
}
void PressButton::mousePressEvent(QMouseEvent *event) {
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x8a, 0x8c, 0x8e));
    setPalette(plt);
    imagePosition.setX((HEAD_HEIGHT / 2 * 3 - HEAD_HEIGHT) / 2 + MOVE_LITTLE);
    repaint();
}
void PressButton::mouseReleaseEvent(QMouseEvent *event) {
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0xff,0xff,0xfb));
    setPalette(plt);
    emit clicked();
    imagePosition.setX((HEAD_HEIGHT / 2 * 3 - HEAD_HEIGHT) / 2);
    repaint();
}
void PressButton::mouseDoubleClickEvent(QMouseEvent *event) {}
void PressButton::mouseMoveEvent(QMouseEvent *event) {}

void PressButton::onClicked() {

}