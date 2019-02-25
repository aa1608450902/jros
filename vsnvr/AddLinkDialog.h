#ifndef __ADDLINKDIALOG_H__
#define __ADDLINKDIALOG_H__

#include <QObject>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include "header.h"

class AddLinkDialog : public QDialog {
    Q_OBJECT
public:
    explicit AddLinkDialog(QWidget *parent = nullptr);
    virtual ~AddLinkDialog();
signals:
    void sendLinkInfo(QString, QString, QString, QString, QString);
private slots:
    void submit();
private:
    QLabel *_alias;
    QLabel *_targetIP;
    QLabel *_targetPort;
    QLabel *_username;
    QLabel *_password;
    QLineEdit *_alias_t;
    QLineEdit *_targetIP_t;
    QLineEdit *_targetPort_t;
    QLineEdit *_username_t;
    QLineEdit *_password_t;
    QPushButton *_ok;
    QPushButton *_cancel;
//    QVBoxLayout *_vlayout;
//    QDialogButtonBox *_buttonBox;
};

#endif /* __ADDLINKDIALOG_H__ */