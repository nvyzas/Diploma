#pragma once

// Project
namespace Ui {
	class MainWindow;
}
// Qt
#include <QtWidgets\QMainWindow>

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();
private slots:
	void loadActiveBoneInfo();
	void setActiveBoneVisibility(bool state);
private:
	void setupObjects();
	void setupConnections();

	Ui::MainWindow *ui;
};
