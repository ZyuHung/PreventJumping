#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <QMainWindow>
#include <QTimer>
#include <QPixmap>
#include <QPainter>
#include <QDebug>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QSound>
#include <QString>

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
    void setDetection(bool onoff);
    void setRect(const QPoint &start, const QPoint &end);
    void setAlarm(bool isAlarm);
    void playwav();
//    bool sortbypoints(cv::Point &v1, cv::Point &v2);
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
#define RETANGLE 1
#define CIRCLE 2
#define POLYGON 3
#define DEFALT 4

    QTimer wav_timer;
    QPoint m_pointStart;
    QPoint m_pointEnd;
    const int CAM_NO=0;
    bool isFirstWav=true;
    bool decetion=false;
    bool isPress=false;
    bool isPolyClosed=false;
    bool isAction=true;
    int rectType=DEFALT;
    QTimer timer;
    cv::Mat CamImg,mask,ForContour;
    cv::VideoCapture capture;
    Ui::MainWindow *ui;
    cv::RotatedRect roRect;
    std::vector<cv::Point> polyPoints;
    std::vector<std::vector<cv::Point>> contours;
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
    void on_DeletePoint_clicked();
};

#endif // MAINWINDOW_H
