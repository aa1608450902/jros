#include "ManageWnd.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>
#include <QPen>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QStringList>
#include <QStringLiteral>
#include <QScrollBar>
#include <QTableWidgetItem>
#include <iostream>
using namespace std;

#define HEAD_HEIGHT 60
#define SYMBOL_LEN 50
#define BORDER_LEN 10
#define MINI_LEN 2

ManageWnd::ManageWnd(QWidget *parent) : QWidget(parent) {
    /// manage window setting
    resize(QApplication::desktop()->width() / 4, QApplication::desktop()->height() - HEAD_HEIGHT);
    setAutoFillBackground(true);
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
//    plt.setColor(QPalette::Background, Qt::white);
    setPalette(plt);

    linkWnd = new LinkWnd(this);
    linkWnd->setGeometry(BORDER_LEN, BORDER_LEN, linkWnd->width(), linkWnd->height());

    addLinkWnd = new AddLinkWnd(this);
    addLinkWnd->setGeometry(BORDER_LEN, linkWnd->height() + BORDER_LEN * 2, addLinkWnd->width(), addLinkWnd->height());

    QObject::connect(linkWnd->getTopButton(), SIGNAL(clicked()), this, SLOT(wndUpdate()));
    QObject::connect(addLinkWnd->getAddLinkForm(), SIGNAL(sendLinkInfo(QString, int, int, QString, QString)),
            linkWnd, SLOT(receiveLinkInfo(QString, int, int, QString, QString)));
    /// button
//    plus = new SymbolButton(this);
//    sub  = new SymbolButton(this, true);
//    plus->setGeometry(width() - SYMBOL_LEN * 3 - BORDER_LEN * 2, BORDER_LEN * 2, plus->width(), plus->height());
//    sub->setGeometry(width() - SYMBOL_LEN - BORDER_LEN, BORDER_LEN * 2, plus->width(), plus->height());
/*
    /// table setting
    table = new QTableWidget(this);
    table->setGeometry(BORDER_LEN, BORDER_LEN * 4 + SYMBOL_LEN, width() - BORDER_LEN * 2, height() / 6);
    table->setColumnCount(6);
    table->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
    table->setHorizontalHeaderLabels(QStringList() << "地址" << "状态" << "通道" << "端口" << "用户" << "密码");
    for (int i = 0; i < 6; ++i) {
        auto *header = table->horizontalHeaderItem(i);
        header->setFont(QFont("Helvetica"));
        header->setBackgroundColor(QColor(0x27, 0x27, 0x27));
        header->setTextColor(QColor(0xff,0xff,0xfb));
    }
    table->setColumnWidth(0, table->width() / 12 * 3);
    table->setColumnWidth(1, table->width() / 12);
    table->setColumnWidth(2, table->width() / 12);
    table->setColumnWidth(3, table->width() / 12);
    table->setColumnWidth(4, table->width() / 12 * 2);
//    table->setColumnWidth(5, table->width() / 12 * 2);
    auto plt2 = table->palette();
    plt2.setColor(QPalette::Base, QColor(0x27, 0x27, 0x27));
    table->setPalette(plt2);
    table->verticalHeader()->hide();
    table->horizontalScrollBar()->setStyleSheet("QScrollBar{background:darkgray;height:10px;}"
                                                "QScrollBar::handle{background:lightgray; border:2px solid transparent;border-radius:5px;}"
                                                "QScrollBar::handle:hover{background:gray;}"
                                                "QScrollBar::sub-line{background:transparent;}"
                                                "QScrollBar::add-line{background:transparent;}");
    table->setSelectionBehavior(QAbstractItemView::SelectRows);

    /// signal and slots
//    QObject::connect(plus, SIGNAL(moveMini()), this, SLOT(onMoveMini()));
//    QObject::connect(plus, SIGNAL(recoverMini()), this, SLOT(onRecoverMini()));
//    QObject::connect(plus, SIGNAL(clicked()), this, SLOT(onAddTableRow()));
//    QObject::connect(sub, SIGNAL(moveMini()), this, SLOT(onMoveMini2()));
//    QObject::connect(sub, SIGNAL(recoverMini()), this, SLOT(onRecoverMini2()));
//    QObject::connect(sub, SIGNAL(clicked()), this, SLOT(onDeleteTableRow()));*/
}

void ManageWnd::paintEvent(QPaintEvent *event) {
//    addLinkWnd->setGeometry(BORDER_LEN, linkWnd->height() + BORDER_LEN * 2, addLinkWnd->width(), addLinkWnd->height());
//    QPainter p(this);
//    QPen pen;
//    pen.setColor(QColor(0xff,0xff,0xfb));
//    pen.setWidth(4);
//    p.setPen(pen);
//    p.drawLine(width(), 0, width(), height());
}
void ManageWnd::wndUpdate() {
    addLinkWnd->setGeometry(BORDER_LEN, linkWnd->height() + BORDER_LEN * 2, addLinkWnd->width(), addLinkWnd->height());
    repaint();
}
/*
void ManageWnd::writeTableRow(int index, const QString& ip, bool status, int chan, int port, const QString& user, const QString& password) {
    auto item_0 = new QTableWidgetItem; item_0->setText(ip);
    item_0->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 0, item_0);
    auto item_1 = new QTableWidgetItem; // (QIcon("/home/aiyo/jpro-2/widget/online.png"), tr(""))
    item_1->setIcon(QIcon("/home/aiyo/jpro-2/widget/online.png"));
    if (status) {
        item_1->setFlags((Qt::ItemFlags)32);
    } else {
        item_1->setFlags((Qt::ItemFlags)0);
    }
    item_1->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 1, item_1);
    auto item_2 = new QTableWidgetItem; item_2->setText(QString::number(chan));
    item_2->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 2, item_2);
    auto item_3 = new QTableWidgetItem; item_3->setText(QString::number(port));
    item_3->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 3, item_3);
    auto item_4 = new QTableWidgetItem; item_4->setText(user);
    item_4->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 4, item_4);
    auto item_5 = new QTableWidgetItem; item_5->setText(password);
    item_5->setTextColor(QColor(0xff,0xff,0xfb)); item_0->setFont(QFont("Helvetica")); table->setItem(index, 5, item_5);
}
void ManageWnd::addTableRow(const QString& ip, bool status, int chan, int port, const QString& user, const QString& password) {
    cout << "---- add table row" << endl;
    tableRowCount++;
    table->setRowCount(tableRowCount);
    writeTableRow(tableRowCount - 1, ip, status, chan, port, user, password);
}
void ManageWnd::deleteTableRow(int index) {

}

void ManageWnd::onMoveMini() {
    plus->setGeometry(width() - SYMBOL_LEN * 3 - BORDER_LEN * 2 + MINI_LEN, BORDER_LEN * 2 + MINI_LEN, plus->width(), plus->height());
}

void ManageWnd::onRecoverMini() {
    plus->setGeometry(width() - SYMBOL_LEN * 3 - BORDER_LEN * 2, BORDER_LEN * 2, plus->width(), plus->height());
}

void ManageWnd::onMoveMini2() {
    sub->setGeometry(width() - SYMBOL_LEN - BORDER_LEN + MINI_LEN, BORDER_LEN * 2 + MINI_LEN, plus->width(), plus->height());
}

void ManageWnd::onRecoverMini2() {
    sub->setGeometry(width() - SYMBOL_LEN - BORDER_LEN, BORDER_LEN * 2, plus->width(), plus->height());
}

void ManageWnd::onAddTableRow() {
    addTableRow("", false, -1, -1, "", "");
}
void ManageWnd::onDeleteTableRow() {
    tableRowCount--;
    auto rowIndex = table->currentRow();
    if (rowIndex != -1) table->removeRow(rowIndex);
}*/