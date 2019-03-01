#include "LinkWnd.h"
#include <QLine>
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
#include <QFont>
#include <QPalette>
#include <QMatrix>
#include <iostream>
using namespace std;

#define BORDER_LEN 10

///////////////////////////////////////////////////////////////////////////////////////


LinkWnd::LinkWnd(QWidget *parent) : QWidget(parent) {
    resize(parent->width() - BORDER_LEN * 2, 30);
    std_height = parent->height() / 4;
    setAutoFillBackground(true);
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
    setPalette(plt);

    topButton = new TopButton("链接信息", this);
    topButton->setGeometry(0, 0, topButton->width(), topButton->height());

    /// table setting
    table = new QTableWidget(this);
    table->setGeometry(0, BORDER_LEN + topButton->height(), width(), std_height - BORDER_LEN - 30 - 25 - BORDER_LEN);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
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
    table->verticalScrollBar()->setStyleSheet("QScrollBar{background:darkgray;height:10px;}"
                                                "QScrollBar::handle{background:lightgray; border:2px solid transparent;border-radius:5px;}"
                                                "QScrollBar::handle:hover{background:gray;}"
                                                "QScrollBar::sub-line{background:transparent;}"
                                                "QScrollBar::add-line{background:transparent;}");
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
//    table->hide();

    deleteButton = new QPushButton("删除", this);
    deleteButton->setStyleSheet("QPushButton{background-color:rgb(39, 39, 39);color:white;border-radius:10px;border:1px groove gray;border-style: outset;}"
                                "QPushButton:pressed{background-color:rgb(138, 140, 142);border-style:inset;}");
    deleteButton->setGeometry(table->width() - 80 - BORDER_LEN, table->y() + table->height() + BORDER_LEN, 80, 25);

    QObject::connect(topButton, SIGNAL(clicked()), this, SLOT(fold()));
    QObject::connect(deleteButton, SIGNAL(clicked()), this, SLOT(onDeleteTableRow()));
}

void LinkWnd::paintEvent(QPaintEvent *event) {
//    QPainter p(this);
//    QPen pen;
//    pen.setColor(QColor(0xff,0xff,0xfb));
//    p.setPen(pen);
//    p.drawLine(0, 0, 0, height());
//    p.drawLine(width() - 1, 0, width() - 1, height());
//    p.drawLine(0, 0, width(), 0);
//    p.drawLine(0, height() - 1, width(), height() - 1);
}

void LinkWnd::fold() {
    if (folded) {
        resize(width(), std_height);
        folded = false;
    } else {
        resize(width(), 30);
        folded = true;
    }
}

void LinkWnd::receiveLinkInfo(QString ip, int chan, int port, QString username, QString password) {
    std::cout << "[ vsnvr ] receive link info" << endl;
    addTableRow(ip, false, chan, port, username, password);
}

void LinkWnd::writeTableRow(int index, const QString& ip, bool status, int chan, int port, const QString& user, const QString& password) {
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
void LinkWnd::addTableRow(const QString& ip, bool status, int chan, int port, const QString& user, const QString& password) {
    cout << "---- add table row" << endl;
    tableRowCount++;
    table->setRowCount(tableRowCount);
    writeTableRow(tableRowCount - 1, ip, status, chan, port, user, password);
}

void LinkWnd::onDeleteTableRow() {
    tableRowCount--;
    auto rowIndex = table->currentRow();
    if (rowIndex != -1) table->removeRow(rowIndex);
}