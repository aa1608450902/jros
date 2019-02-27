#ifndef __CTRLWND_H__
#define __CTRLWND_H__

#include <QWidget>

class CtrlWnd : public QWidget {
	Q_OBJECT
public:
	explicit CtrlWnd(QWidget *parent = nullptr);
	virtual ~CtrlWnd() = default;
};

#endif /* __CTRLWND_H__ */