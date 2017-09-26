#pragma once
#include "main_widget.h"
#include <QtWidgets\QMainWindow>

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();

protected:
	void keyPressEvent(QKeyEvent *);

private:
	Ui::MainWindow *ui;
	void setupObjects();
	void setupConnections();
};
