#ifndef __HEADWND_H__
#define __HEADWND_H__

#include <QObject>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>

class HeadWnd : public QWidget {
	Q_OBJECT
public:
	explicit HeadWnd(QWidget *parent = nullptr);
	~HeadWnd();
	QPushButton* getSwitchButton() const;
	void paintEvent(QPaintEvent *event);
private:
	// QHBoxLayout *_layout = nullptr;
	QLabel *_label = nullptr;
	QPushButton *_wndSwitch = nullptr;
};

#endif /* __HEADWND_H__ */