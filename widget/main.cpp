#include <iostream>
using namespace std;
#include <QApplication>
#include "TopWnd.h"

int main(int argc, char* argv[]) {
	cout << "--- testing ..." << endl;
	QApplication app(argc, argv);
	TopWnd w;
	w.showFullScreen();
//	w.updateGeometry();
//	w.repaint();
	return QApplication::exec();
}