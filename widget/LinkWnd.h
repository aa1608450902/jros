#ifndef __LINKWND_H__
#define __LINKWND_H__

#include <QWidget>
#include <QLabel>
#include <QTableWidget>
#include <QPushButton>
#include "TopButton.h"

class LinkWnd: public QWidget {
    Q_OBJECT
public:
    explicit LinkWnd(QWidget *parent = nullptr);
    virtual ~LinkWnd() = default;
    TopButton* getTopButton() {return topButton;}
protected:
    void paintEvent(QPaintEvent *event);
private:
    void writeTableRow(int index, const QString& ip, bool status, int chan, int port, const QString& user, const QString& password);
    void addTableRow(const QString& ip, bool status, int chan, int port, const QString& user, const QString& password);
private slots:
    void fold();
    void receiveLinkInfo(QString, int, int, QString, QString);
    void onDeleteTableRow();
private:
    int std_height;
    bool folded = true;
    TopButton* topButton;
    QTableWidget* table;
    QPushButton* deleteButton;
    int tableRowCount = 0;
};

#endif /* __LINKWND_H__ */