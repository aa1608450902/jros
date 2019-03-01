#include "AddLinkWnd.h"
#include <QPainter>
#include <iostream>
using namespace std;

#define BORDER_LEN 10
//#define STD_HEIGHT

AddLinkForm::AddLinkForm(int std_height, QWidget *parent) : QWidget(parent) {
    resize(parent->width(), std_height - BORDER_LEN - 30);
    ipLabel = new QLabel(this);
    {
        ipLabel->setFont(QFont("Helvetica"));
        auto plt2 = ipLabel->palette();
        plt2.setColor(QPalette::WindowText, Qt::white);
        ipLabel->setPalette(plt2);
        ipLabel->setText("地址");
        ipLabel->setGeometry(BORDER_LEN, BORDER_LEN, 50, 20);
        ipLabel->setAlignment(Qt::AlignVCenter);
    }
    cout << "---- add link width: " << width() << "; height: " << height() << "; parent ....: " << parent->height() - BORDER_LEN - 30 << endl;

    chanLabel = new QLabel(this);
    portLabel = new QLabel(this);
    usernameLabel = new QLabel(this);
    passwordLabel = new QLabel(this);
    {
        chanLabel->setFont(QFont("Helvetica"));auto plt2 = chanLabel->palette();plt2.setColor(QPalette::WindowText, Qt::white);chanLabel->setPalette(plt2);
        chanLabel->setText("通道");chanLabel->setGeometry(BORDER_LEN, BORDER_LEN * 2 + ipLabel->height(), 50, 20);chanLabel->setAlignment(Qt::AlignVCenter);
    }
    {
        portLabel->setFont(QFont("Helvetica"));auto plt2 = portLabel->palette();plt2.setColor(QPalette::WindowText, Qt::white);portLabel->setPalette(plt2);
        portLabel->setText("端口");portLabel->setGeometry(width() / 2 + BORDER_LEN, BORDER_LEN * 2 + ipLabel->height(), 50, 20);portLabel->setAlignment(Qt::AlignVCenter);
    }
    {
        usernameLabel->setFont(QFont("Helvetica"));auto plt2 = usernameLabel->palette();plt2.setColor(QPalette::WindowText, Qt::white);usernameLabel->setPalette(plt2);
        usernameLabel->setText("用户名");usernameLabel->setGeometry(BORDER_LEN, BORDER_LEN * 3 + ipLabel->height() * 2, 50, 20);usernameLabel->setAlignment(Qt::AlignVCenter);
    }
    {
        passwordLabel->setFont(QFont("Helvetica"));auto plt2 = passwordLabel->palette();plt2.setColor(QPalette::WindowText, Qt::white);passwordLabel->setPalette(plt2);
        passwordLabel->setText("密码");passwordLabel->setGeometry(width() / 2 + BORDER_LEN, BORDER_LEN * 3 + ipLabel->height() * 2, 50, 20);passwordLabel->setAlignment(Qt::AlignVCenter);
    }

    ipLine = new QLineEdit(this);
    chanLine = new QLineEdit(this);
    portLine = new QLineEdit(this);
    usernameLine = new QLineEdit(this);
    passwordLine = new QLineEdit(this);
    {ipLine->setStyleSheet("background-color:rgb(48,48,48);color:white;");ipLine->setGeometry(BORDER_LEN + 50, BORDER_LEN, 150, 21);}
    {chanLine->setStyleSheet("background-color:rgb(48,48,48);color:white;");chanLine->setGeometry(BORDER_LEN + 50, BORDER_LEN * 2 + ipLabel->height(), 150, 21);}
    {portLine->setStyleSheet("background-color:rgb(48,48,48);color:white;");portLine->setGeometry(width() / 2 + BORDER_LEN + 50, BORDER_LEN * 2 + ipLabel->height(), 150, 21);}
    {usernameLine->setStyleSheet("background-color:rgb(48,48,48);color:white;");usernameLine->setGeometry(BORDER_LEN + 50, BORDER_LEN * 3 + ipLabel->height() * 2, 150, 21);}
    {passwordLine->setStyleSheet("background-color:rgb(48,48,48);color:white;");passwordLine->setGeometry(width() / 2 + BORDER_LEN + 50, BORDER_LEN * 3 + ipLabel->height() * 2, 150, 21);}

    okButton = new QPushButton("确认", this);
    okButton->setStyleSheet("QPushButton{background-color:rgb(39, 39, 39);color:white;border-radius:10px;border:1px groove gray;border-style: outset;}"
//                            "QPushButton:hover{background-color:white;color:black;}"
                            "QPushButton:pressed{background-color:rgb(138, 140, 142);border-style:inset;}");
//    okButton->setStyleSheet("QPushButton{background-color:black;color:white;border-radius:10px;border:2px groove gray;border-style: outset;}"
//                            "QPushButton:hover{background-color:white;color:black;}"
//                            "QPushButton:pressed{background-color:rgb(85, 170, 255);border-style:inset;}");
//    okButton->setStyleSheet("QPushButton{background-color:black;color:white;border-radius:10px;border-style:outset;}"
//                            "QPushButton:pressed{background-color:rgb(85, 170, 255);border-style:inset;}");
    okButton->setGeometry(BORDER_LEN, BORDER_LEN * 5 + ipLabel->height() * 3, 80, 25);
    QObject::connect(okButton, SIGNAL(clicked()), this, SLOT(onOkClicked()));
}

void AddLinkForm::paintEvent(QPaintEvent *event) {
//    QPainter p(this);
//    QPen pen;
//    pen.setColor(QColor(0xff,0xff,0xfb));
//    p.setPen(pen);
//    p.drawText(0,0,"cnajilsasdjilasdbjcuahsdbcjhbahksdcvajksdvcgasvdhcbaksd\ncbahsdkvchkasvdhkc\nbchasdvchasd\ndcajhsd");
//    p.drawLine(0, 0, 0, height());
//    p.drawLine(width() - 1, 0, width() - 1, height());
//    p.drawLine(0, 0, width(), 0);
//    p.drawLine(0, height() - 1, width(), height() - 1);
}

void AddLinkForm::onOkClicked() {
    QString ip = ipLine->text();
    QString chan = chanLine->text();
    QString port = portLine->text();
    QString username = usernameLine->text();
    QString password = passwordLine->text();
    if (ip.isEmpty() || chan.isEmpty() || port.isEmpty() || username.isEmpty() || password.isEmpty())
        return;


    std::cout << "[ vsnvr ] add device: ip - " << ip.toStdString()
              << ", port - " << port.toStdString()
              << ", username - " << username.toStdString()
              << ", password - " << password.toStdString()
              << std::endl;
    emit sendLinkInfo(ip, chan.toInt(), port.toInt(), username, password);
}

///////////////////////////////////////////////////////////////////////////

AddLinkWnd::AddLinkWnd(QWidget *parent) : QWidget(parent) {
    resize(parent->width() - BORDER_LEN * 2, 30);
    cout << "---- add link wnd width: " << width() << "; height: " << height() << endl;
    std_height = parent->height() / 5;
    setAutoFillBackground(true);
    auto plt = palette();
    plt.setColor(QPalette::Background, QColor(0x27, 0x27, 0x27));
    setPalette(plt);

    topButton = new TopButton("添加链接", this);
    topButton->setGeometry(0, 0, topButton->width(), topButton->height());

    addLinkForm = new AddLinkForm(std_height, this);
    addLinkForm->setGeometry(0, BORDER_LEN + 30, addLinkForm->width(), addLinkForm->height());

    QObject::connect(topButton, SIGNAL(clicked()), this, SLOT(fold()));
}

void AddLinkWnd::fold() {
    if (folded) {
        cout << "---- add link foled" << endl;
        resize(width(), std_height);
        folded = false;
    } else {
        resize(width(), 30);
        folded = true;
        cout << "---- add link unfoled" << endl;
    }
}