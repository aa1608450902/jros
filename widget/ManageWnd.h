#ifndef __MANAGEWND_H__
#define __MANAGEWND_H__

#include <QWidget>
#include "SymbolButton.h"
#include <QTableWidget>
#include "LinkWnd.h"
#include "AddLinkWnd.h"

class ManageWnd : public QWidget {
    Q_OBJECT
public:
    explicit ManageWnd(QWidget* parent = nullptr);
    virtual ~ManageWnd() = default;
protected:
    void paintEvent(QPaintEvent *event);
private:
/*    void writeTableRow(int index, const QString& ip, bool status, int chan, int port, const QString& user, const QString& password);
    void addTableRow(const QString& ip, bool status, int chan, int port, const QString& user, const QString& password);
    void deleteTableRow(int index);*/
private slots:
    void wndUpdate();
/*    void onMoveMini();
    void onRecoverMini();
    void onMoveMini2();
    void onRecoverMini2();
    void onAddTableRow();
    void onDeleteTableRow();*/
private:
    LinkWnd* linkWnd;
    AddLinkWnd* addLinkWnd;
/*    SymbolButton* plus;
    SymbolButton* sub;
    QTableWidget* table;
    int tableRowCount = 0;*/
};

#endif /* __MANAGEWND_H__ */