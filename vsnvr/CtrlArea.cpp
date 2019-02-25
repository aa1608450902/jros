#include "CtrlArea.h"
#include "header.h"

CtrlArea::CtrlArea(QWidget *parent) : QWidget(parent) {
    _fold = new QPushButton("<", this);
    _fold->setGeometry(0, 0, CTRL_WIDTH_1, parent->height());
}

CtrlArea::~CtrlArea() {

}

QPushButton* CtrlArea::getFoldButton() {
    return _fold;
}