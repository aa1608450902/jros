#include <iostream>
#include <QApplication>
#include "TopWnd.h"
#include <QStyleFactory>

int main(int argc, char* argv[]) {
	std::cout << "[ vsnvr ] loading ..." << std::endl;
	QApplication app(argc, argv);
	TopWnd w; w.show();
	return QApplication::exec();
}
