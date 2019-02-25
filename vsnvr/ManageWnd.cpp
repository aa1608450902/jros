#include "ManageWnd.h"
#include <QStringList>
#include <QStringLiteral>
#include <iostream>
#include <QHeaderView>
#include "header.h"
#include <QAbstractItemView>
#include <QMessageBox>
#include "HCNetSDK.h"
#include "global.h"

ManageWnd::ManageWnd(QWidget *parent) : QWidget(parent) {
//    _label = new QLabel("manage window", this);
//    _label->setGeometry(0, 0, 200, 20);
    _add = new QPushButton("add", this);
    _add->setGeometry(10, 10, 100, 30);
    _del = new QPushButton("delete", this);
    _del->setGeometry(120, 10, 100, 30);
    _refresh = new QPushButton("refresh", this);
    _refresh->setGeometry(230, 10, 100, 30);
//    _addLinkDialog = new AddLinkDialog(this);
    QObject::connect(_add, SIGNAL(clicked()), this, SLOT(addLink()));
    resize(parent->width(), parent->height() - HEADWND_HEIGHT);
    _table = new QTableWidget(this);
    _table->setGeometry(10, 50, width(), height() - 50);
    _table->setColumnCount(8);
    _table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _table->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
    _table->setHorizontalHeaderLabels(QStringList() << "alias" << "ip" << "status" << "channel" << "port" << "username" << "password" << "description");
    for (int i = 0; i < 8; i++) {_table->setColumnWidth(i, width() / 6);}
    QObject::connect(_del, SIGNAL(clicked()), this, SLOT(deleteLink()));
    QObject::connect(_refresh, SIGNAL(clicked()), this, SLOT(reconnect()));

    { /// fresh manage window
        auto& links = gLinkInfo();
        auto& rps = gRealPlayHandle();
        for (int i = 0; i < links.length(); i++) {
            QString status;
            if (rps[i].userId == -1)
                status = "offline";
            else status = "online";
            recordLinkInfo("", QString(links[i].ip.data()), status,
                           links[i].port, links[i].channel, QString(links[i].username.data()),
                           QString(links[i].password.data()), "");
        }
//        std::cout << "[ vsnvr ] Fresh manage window - link table." << std::endl;
    }

//    receiveLinkInfo();
}

ManageWnd::~ManageWnd() {}

void ManageWnd::paintEvent(QPaintEvent *event) {
    _add->setGeometry(10, 10, 100, 30);
    _table->setGeometry(10, 50, width() - 20, height() - 55);
}

void ManageWnd::addLink() {
//    std::cout << "[ vsnvr ] add link button pushed" << std::endl;
    AddLinkDialog addLinkDialog(this);
    QObject::connect(&addLinkDialog, SIGNAL(sendLinkInfo(QString, QString, QString, QString, QString)),
            this, SLOT(receiveLinkInfo(QString, QString, QString, QString, QString)));
    addLinkDialog.exec();
//    addLinkDialog.show();
//    std::cout << "[ vsnvr ] add link button pushed exec over" << std::endl;

//    QObject::connect();
}

void ManageWnd::deleteLink() {
    std::cout << "[ vsnvr ] current row: " << _table->currentRow() << std::endl;
    auto rowIndex = _table->currentRow();
    if (rowIndex != -1) _table->removeRow(rowIndex);
}

void ManageWnd::reconnect() {
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
}

void ManageWnd::receiveLinkInfo(QString alias, QString ip, QString port, QString username, QString password) {
    _rowCount++;
    _table->setRowCount(_rowCount);
//    alias = "cansdncasd";
//    std::cout << "row: " << _rowCount - 1
//              << "[ vsnvr ] add device: alias - " << alias.toStdString()
//              << ", ip - " << ip.toStdString()
//              << ", port - " << port.toStdString()
//              << ", username - " << username.toStdString()
//              << ", password - " << password.toStdString()
//              << std::endl;
//    auto isDigit = [](QString s) -> bool {
//        QByteArray bt = s.toLatin1();
//        const char *c = bt.data();
//        while(*c && *c>='0' && *c<='9') c++;
//        if (*c) return false; else true;
//    };
//    if (!isDigit(port)) {
//        QMessageBox msg;
//        msg.setText("port must be number!");
//        msg.exec();
//        return;
//    }
    auto item_0 = new QTableWidgetItem; item_0->setText(alias); _table->setItem(_rowCount - 1, 0, item_0);
    auto item_1 = new QTableWidgetItem; item_1->setText(ip); _table->setItem(_rowCount - 1, 1, item_1);
    auto item_2 = new QTableWidgetItem; item_2->setText(port); _table->setItem(_rowCount - 1, 3, item_2);
    auto item_3 = new QTableWidgetItem; item_3->setText(username); _table->setItem(_rowCount - 1, 4, item_3);
    auto item_4 = new QTableWidgetItem; item_4->setText(password); _table->setItem(_rowCount - 1, 5, item_4);
//    auto item_5 = new QTableWidgetItem; item_5->setText(password); _table->setItem(_rowCount, 6, item_5);
}

void ManageWnd::recordLinkInfo(QString alias, QString ip, QString status, int port, int channel, QString username, QString password, QString desc) {
    _rowCount++;
    _table->setRowCount(_rowCount);
    auto item_0 = new QTableWidgetItem; item_0->setText(alias); _table->setItem(_rowCount - 1, 0, item_0);
    auto item_1 = new QTableWidgetItem; item_1->setText(ip); _table->setItem(_rowCount - 1, 1, item_1);
    auto item_6 = new QTableWidgetItem; item_6->setText(status); _table->setItem(_rowCount - 1, 2, item_6);
    auto item_2 = new QTableWidgetItem; item_2->setText(QString::number(port)); _table->setItem(_rowCount - 1, 3, item_2);
    auto item_3 = new QTableWidgetItem; item_3->setText(QString::number(channel)); _table->setItem(_rowCount - 1, 3, item_3);
    auto item_4 = new QTableWidgetItem; item_4->setText(username); _table->setItem(_rowCount - 1, 4, item_4);
    auto item_5 = new QTableWidgetItem; item_5->setText(password); _table->setItem(_rowCount - 1, 5, item_5);
}