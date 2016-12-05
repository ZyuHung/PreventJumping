/*****************************************************************************
* Copyright (c) 2016 GZHU_EENB_LAB629 Corporation
* All Rights Reserved.
*
* Project Name         :   PreventJumping
* File Name            :   mainwindow.h
*
* Create Date          :   2016/11/28
* Author               :   Zhu Zhihong
* Address              :   Guangzhou University(HEMC)

******************************************************************************/

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
    void ShowCamera_v();
    void PauseCamera_v();
    void CloseCamera_v();
    void UpdateImage_v();
    void ProcessImage_v();
    void SetDetection_v(bool onoff);
    void SetRect_v(const QPoint &start, const QPoint &end);
    void SetAlarm_v(bool isAlarm);
    void Playwav_v();
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
#define RETANGLE 1
#define CIRCLE 2
#define POLYGON 3
#define DEFALT 4

    QTimer mWavTimer_qt;
    QPoint mStartPoint_qp;
    QPoint mEndPoint_qp;
    const int CAM_NO=0;
    bool mIsFirstWav_b=true;
    bool mIsDetection_b=false;
    bool mIsPress_b=false;
    bool mIsPolyClosed_b=false;
    bool mIsAction_b=true;
    int mRectType_i=DEFALT;
    QTimer mUpdatingTimer_qt;
    cv::Mat mCamImage_M,mMask_M;
    cv::VideoCapture mCapture_VC;
    Ui::MainWindow *ui;
    cv::RotatedRect mRoRect_RR;
    std::vector<cv::Point> mPolyPoints_v_P;
    cv::Point mClickPoint_P;
    std::size_t mIdentification_uc=230;
    float mAlarmSen_f=0.1;

protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private slots:
    void on_mOpenCamera_bt_clicked();
    void on_mPauseCamera_bt_clicked();
    void on_mStopCamera_bt_clicked();
    void on_mDetection_bt_clicked();
    void on_mRectangle_bt_clicked();
    void on_mCircle_bt_clicked();
    void on_mPolygon_bt_clicked();
    void on_mDeletePoint_bt_clicked();
    void on_mRgnSensitivity_sld_valueChanged(int value);
    void on_mAlarmSensitivity_sld_valueChanged(int value);
};

#endif // MAINWINDOW_H
