#include "AddLinkDialog.h"

AddLinkDialog::AddLinkDialog(QWidget *parent) : QDialog(parent) {
//    setFixedSize(150, 300);
//    _buttonBox = new QDialogButtonBox(this);
//    _buttonBox->setObjectName(QStringLiteral("buttonBox"));
//    _buttonBox->setGeometry(QRect(20, 260, 341, 32));
//    _buttonBox->setOrientation(Qt::Horizontal);
//    _buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
    setFixedSize(220, 300);

    _alias = new QLabel("alias:", this);
    _alias->setGeometry(10, 10, 100, 40);
    _targetIP = new QLabel("ip:", this);
    _targetIP->setGeometry(10, 60, 100, 40);
    _targetPort = new QLabel("port:", this);
    _targetPort->setGeometry(10, 110, 100, 40);
    _username = new QLabel("username:", this);
    _username->setGeometry(10, 160, 100, 40);
    _password = new QLabel("password:", this);
    _password->setGeometry(10, 210, 100, 40);
    _alias_t = new QLineEdit(this);
    _alias_t->setGeometry(110, 15, 100, 30);
    _targetIP_t = new QLineEdit(this);
    _targetIP_t->setGeometry(110, 65, 100, 30);
    _targetPort_t = new QLineEdit(this);
    _targetPort_t->setGeometry(110, 115, 100, 30);
    _username_t = new QLineEdit(this);
    _username_t->setGeometry(110, 165, 100, 30);
    _password_t = new QLineEdit(this);
    _password_t->setGeometry(110, 215, 100, 30);
    _ok = new QPushButton("ok", this);
    _ok->setGeometry(20, 260, 80, 30);
    _cancel = new QPushButton("cancel", this);
    _cancel->setGeometry(width() - 100, 260, 80, 30);

    QObject::connect(_ok, SIGNAL(clicked()), this, SLOT(submit()));
    QObject::connect(_cancel, SIGNAL(clicked()), this, SLOT(reject()));
//    _vlayout = new QVBoxLayout(this);
//    _vlayout->addWidget(_alias)
}

AddLinkDialog::~AddLinkDialog() {

}

void AddLinkDialog::submit() {
    auto alias = _alias_t->text();
    auto ip = _targetIP_t->text();
    auto port = _targetPort_t->text();
    auto username = _username_t->text();
    auto password = _password_t->text();
    std::cout << "[ vsnvr ] add device: alias - " << alias.toStdString()
              << ", ip - " << ip.toStdString()
              << ", port - " << port.toStdString()
              << ", username - " << username.toStdString()
              << ", password - " << password.toStdString()
              << std::endl;
    emit sendLinkInfo(alias, ip, port, username, password);
    done(Accepted);
}
