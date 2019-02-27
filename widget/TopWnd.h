#ifndef __TOPWND_H__
#define __TOPWND_H__

#include <QWidget>
#include <QPaintEvent>
#include "CtrlWnd.h"
#include "ViewWnd.h"
#include <QPoint>

class TopWnd : public QWidget {
	Q_OBJECT
public:
	explicit TopWnd(QWidget *parent = nullptr);
	virtual ~TopWnd() = default;
//	void updateGeometry();
protected:
//	void paintEvent(QPaintEvent *event);
private slots:
    void viewWndMoveV(int dis);
private:
    QPoint viewWndPos;
	CtrlWnd* _ctrlWnd;
	ViewWnd* _viewWnd;
};

#endif /* __TOPWND_H__ */