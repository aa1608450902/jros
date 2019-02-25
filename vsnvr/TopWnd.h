#ifndef __TOPWND_H__
#define __TOPWND_H__

#include "header.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QList>
#include "structs.h"

class TopWnd : public QWidget {
	Q_OBJECT
public:
	explicit TopWnd(QWidget *parent = nullptr);
	virtual ~TopWnd();
private slots:
	void switchWnd();
	void fullScreen();
	void maximizedScreen();
//    void play();
public:
    void paintEvent(QPaintEvent *event);
//    void link();
private:
	QWidget *_headWnd = nullptr;
//	QWidget *_bodyWnd = nullptr;
	QWidget *_viewWnd = nullptr;
	QWidget *_manageWnd = nullptr;
};

#endif /* __TOPWND_H__ */