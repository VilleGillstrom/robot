#pragma once

#include <QWidget>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QCloseEvent>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QWidget(parent) {};
    virtual ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override {
        event->accept();
    }
private:
    //Ui::MainWindow *ui;
};
