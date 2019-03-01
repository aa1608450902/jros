#ifndef __ADDLINKWND_H__
#define __ADDLINKWND_H__

#include <QWidget>
#include <QLabel>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include "TopButton.h"

class AddLinkForm : public QWidget {
    Q_OBJECT
public:
    explicit AddLinkForm(int std_height, QWidget *parent = nullptr);
    virtual ~AddLinkForm() = default;
protected:
    void paintEvent(QPaintEvent *event);
signals:
    void sendLinkInfo(QString, int, int, QString, QString);
private slots:
    void onOkClicked();
private:
    QLabel *ipLabel;
    QLabel *chanLabel;
    QLabel *portLabel;
    QLabel *usernameLabel;
    QLabel *passwordLabel;
    QLineEdit *ipLine;
    QLineEdit *chanLine;
    QLineEdit *portLine;
    QLineEdit *usernameLine;
    QLineEdit *passwordLine;
    QPushButton *okButton;
};

class AddLinkWnd: public QWidget {
    Q_OBJECT
public:
    explicit AddLinkWnd(QWidget *parent = nullptr);
    virtual ~AddLinkWnd() = default;
    AddLinkForm* getAddLinkForm() {return addLinkForm;}
//protected:
//    void paintEvent(QPaintEvent *event);
private slots:
    void fold();
private:
    int std_height;
    bool folded = true;
    TopButton* topButton;
    AddLinkForm* addLinkForm;
//    QTableWidget* table;
//    int tableRowCount = 0;
};

#endif /* __ADDLINKWND_H__ */