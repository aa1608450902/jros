#ifndef __MANAGEWND_H__
#define __MANAGEWND_H__

#include <QObject>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
//#include <QTableView>
#include "AddLinkDialog.h"
#include <QTableWidget>

class ManageWnd : public QWidget {
	Q_OBJECT
public:
	explicit ManageWnd(QWidget *parent = nullptr);
	~ManageWnd();
	void paintEvent(QPaintEvent *event);
private slots:
	void addLink();
	void deleteLink();
	void receiveLinkInfo(QString, QString, QString, QString, QString);
	void reconnect();
private:
	void recordLinkInfo(QString, QString, QString, int, int, QString, QString, QString);
private:
//	QLabel *_label = nullptr;
	QPushButton *_add = nullptr;
	QPushButton *_del = nullptr;
	QPushButton *_refresh = nullptr;
	QTableWidget *_table;
	int _rowCount = 0;
//    AddLinkDialog *_addLinkDialog = nullptr;
//	QTableView *_linkList = nullptr;
};

#endif /* __MANAGEWND_H__ */