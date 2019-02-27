#include "CtrlWnd.h"
#include <QPalette>
#include <QColor>

CtrlWnd::CtrlWnd(QWidget *parent) : QWidget(parent) {
	setAutoFillBackground(true);
	QPalette p(palette());
	p.setColor(QPalette::Background, QColor(0xff,0xff,0xfb));
	setPalette(p);
}