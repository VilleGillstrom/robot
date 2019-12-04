#pragma once

#include <QWidget>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QWidget(parent) {};
    virtual ~MainWindow();
private:
    //Ui::MainWindow *ui;
};
