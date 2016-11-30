#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QTimer>
#include <QPixmap>
#include <QPainter>
#include <QDebug>
#include <QMouseEvent>
#include <QKeyEvent>
#include <vector>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    void showCamera();
    void pauseCamera();
    void closeCamera();
    void UpdateImage();
    void ProcessImage();
    void setDecetion(bool onoff);
    void setRect(const QPoint &start, const QPoint &end);

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
#define RETANGLE 1
#define CIRCLE 2
#define POLYGON 3
#define DEFALT 4


    QPoint m_pointStart;
    QPoint m_pointEnd;
    const int CAM_NO=0;
    bool decetion=false;
    bool isPress=false;
    bool isPolyClosed=false;
    int rectType=DEFALT;
    QTimer timer;
    cv::Mat CamImg,mask;
    cv::VideoCapture capture;
    Ui::MainWindow *ui;
    cv::RotatedRect roRect;
    std::vector<cv::Point> polyPoints;
    cv::Point tmp;
    cv::Point lineEndPoint;

protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_Button_decetion_clicked();
    void on_Button_retangle_clicked();
    void on_Button_circle_clicked();
    void on_Button_polygon_clicked();
};

#endif // MAINWINDOW_H
