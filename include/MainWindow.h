#pragma once

#include <QWidget>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QCloseEvent>


/**
 * A simple window for showing the gui
 */
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
};
