#ifndef __TOPWND_H__
#define __TOPWND_H__

#include <QWidget>
#include <QPaintEvent>
#include "CtrlWnd.h"
#include "ViewWnd.h"
#include <QPoint>
#include <QPropertyAnimation>
#include "PressButton.h"
#include "ManageWnd.h"

class TopWnd : public QWidget {
	Q_OBJECT
public:
	explicit TopWnd(QWidget *parent = nullptr);
	virtual ~TopWnd() = default;
//	void updateGeometry();
protected:
	void paintEvent(QPaintEvent *event);
private slots:
    void viewWndMoveV(int dis);
    void viewAutoAdjust();
    void displayCtrlWnd();
    void manageWndDisplay();
private:
    bool displayedManageWnd = false;
    bool displayedViewWnd = false;
    QPoint viewWndPos;
//	CtrlWnd* _ctrlWnd;
	ViewWnd* _viewWnd;
	ManageWnd* manageWnd;
	PressButton* settings;
    QPropertyAnimation *viewWndAni;
    QPropertyAnimation *manageWndAni;
    QImage *logo;
};

#endif /* __TOPWND_H__ */