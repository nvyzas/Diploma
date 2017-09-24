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

private:
	Ui::MainWindow *ui;
	void CreateActions();
};
