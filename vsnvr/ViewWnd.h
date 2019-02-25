#ifndef __VIEWWND_H__
#define __VIEWWND_H__

#include "header.h"
#include <QGridLayout>
#include "ViewArea.h"
#include "CtrlArea.h"

class ViewWnd : public QWidget {
	Q_OBJECT
public:
	explicit ViewWnd(QWidget *parent = nullptr);
	virtual ~ViewWnd();
public:
//	QFrame* getFrame();
    void paintEvent(QPaintEvent *event);
	void link();
private:
    void cleanup();
private slots:
	void play();
	void singleWnd();
    void ctrlWndFold();
    void stopPlay();
private:
//	QFrame *_playWnd;
//    int _userID;
//    long _realPlayHandle;
//    QGridLayout *_glayout = nullptr;
//	QLabel *_label = nullptr;
	QWidget *_viewArea = nullptr;
	QWidget *_ctrlArea = nullptr;
	QPushButton *_singleWnd = nullptr;
	QPushButton *_multiWnd = nullptr;
	QPushButton *_stopView = nullptr;
	QPushButton *_startView = nullptr;
	QPushButton *_fullScreen = nullptr;
	int _ctrlAreaWidght;
};

#endif /* __VIEWWND_H__ */