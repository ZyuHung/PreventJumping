/*****************************************************************************
* Copyright (c) 2016 GZHU_EENB_LAB629 Corporation
* All Rights Reserved.
*
* Project Name         :   PreventJumping
* File Name            :   mainwindow.cpp
* Abstract Description :   Set a restricted area at image that from camera.
*
* Create Date          :   2016/11/28
* Author               :   Zhu Zhihong
* Address              :   Guangzhou University(HEMC)

******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

cv::Rect gDetectionRange_R;
cv::BackgroundSubtractorMOG2 gBgSubtractor_BGS;

//sort the 2dims vector "contours" by their sizes
//轮廓点按照数量降序排序以找出实际轮廓
bool gSortBySize_b(std::vector<cv::Point> &v1,std::vector<cv::Point> &v2)
{
    return v1.size()>v2.size();
}

//Constructor function
//构造函数
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qDebug()<<"MainWindow";
    ui->setupUi(this);
    ui->Alarming->setVisible(false);
    setWindowFlags(windowFlags()& ~Qt::WindowMaximizeButtonHint);//stop maximizing the window（禁止最大化窗口）
    connect(&mUpdatingTimer_qt,&QTimer::timeout,this,&MainWindow::UpdateImage_v);//set the timer for updating camera images（定时更新摄像头画面）
    connect(&mWavTimer_qt,&QTimer::timeout,this,&MainWindow::Playwav_v);//set the timer for playing alarming audio（设置特定时间内只响一次报警音）
}

//try to open camera and get the first frame
//尝试打开摄像头并获得第一帧（初始化）
void MainWindow::ShowCamera_v()
{
    if (mCapture_VC.open(CAM_NO))
    {
        mCapture_VC>>mCamImage_M;
        //because the color sequence of Mat in opencv is BGR but RGB in Qt, must conver BGR to RGB.
        //opencv的颜色序列为BGR，而Qt的颜色序列为RGB
        cv::cvtColor(mCamImage_M,mCamImage_M,CV_BGR2RGB);
        ui->status->setText(tr("Camera open."));
        mUpdatingTimer_qt.start(30);//FPS = 30;
        mIsAction_b=true;//allow user to click at the image（允许用户点击画面）
        ui->mOpenCamera_bt->setEnabled(false);      //set "OpenCamera" button disabled.
        ui->mPauseCamera_bt->setEnabled(true);     //set "PauseCamera" button abled.
        ui->mStopCamera_bt->setEnabled(true);     //set "CloseCamera" button abled.
        ui->mDetection_bt->setEnabled(true);  //set "StartDetection" button abled.
        ui->mRectangle_bt->setEnabled(true);  //set "retangle" button abled.
        ui->mPolygon_bt->setEnabled(true);   //set "polygon" button abled.
        ui->mCircle_bt->setEnabled(true);    //set "circle" button abled.
    }
    else
    {
        ui->status->setText(tr("Camera open FAILED."));
    }
}

//PauseCamera（暂停视频，响应"PauseCamera"按钮）
void MainWindow::PauseCamera_v()
{
    if (ui->mPauseCamera_bt->text()=="PauseCamera")
    {
        ui->mPauseCamera_bt->setText(tr("ContinueCamera"));
        ui->status->setText(tr("Pause"));
        mIsAction_b=false;//Prevents the user from clicking the image（禁止用户点击画面）
        mUpdatingTimer_qt.stop();//stop updating the camera image（停止刷新摄像头画面）

    }else
    {
        mUpdatingTimer_qt.start(30);//restart updating the camera iamge（重新刷新摄像头画面）
        mIsAction_b=true;//allow the user to click the image（允许用户点击画面）
        ui->mPauseCamera_bt->setText(tr("PauseCamera"));
        ui->status->setText(tr("Camera open."));
    }

}

//CloseCamera（关闭摄像头，响应"CloseCamera"按钮）
void MainWindow::CloseCamera_v()
{
    mUpdatingTimer_qt.stop();//stop updating the camera image（停止刷新摄像头画面）
    SetDetection_v(false);//stop detection
    mRectType_i=DEFALT;//load the default ROI
    mCamImage_M.release();
    ui->status->setText(tr("Camera closed."));
    ui->mOpenCamera_bt->setEnabled(true); //set "OpenCamera" button abled.
    ui->mPauseCamera_bt->setEnabled(false);//set "PauseCamera" button disabled.
    ui->mPauseCamera_bt->setText(tr("PauseCamera"));
    ui->mStopCamera_bt->setEnabled(false);   //set "CloseCamera" button disabled.
    ui->mDetection_bt->setEnabled(false);//set "StartDetection" button disabled.
    ui->mRectangle_bt->setEnabled(false);//set "retangle" button disabled.
    ui->mPolygon_bt->setEnabled(false);//set "polygon" button disabled.
    ui->mCircle_bt->setEnabled(false);//set "circle" button disabled.
    ui->mDeletePoint_bt->setEnabled(false);//set "delete" button disabled.
    ui->mAlarmSensitivity_sld->setEnabled(false);
    ui->mRgnSensitivity_sld->setEnabled(false);
    mCapture_VC.release();
}

//paint the image
void MainWindow::paintEvent(QPaintEvent *event)
{
    //Mat to QImage.
    QImage image2 = QImage((uchar*)(mCamImage_M.data), mCamImage_M.cols, mCamImage_M.rows, QImage::Format_RGB888);
    ui->label->setPixmap(QPixmap::fromImage(image2));
    ui->label->resize(image2.size());
    ui->label->show();
    setFixedSize(this->width(), this->height());
}

void MainWindow::UpdateImage_v()
{
    mCapture_VC>>mCamImage_M;//get Image from camera（从摄像头视频流获取视频画面）
    if (mCamImage_M.data)
    {
        ui->status->setText(tr("Camera open."));
        switch (mRectType_i)
        {
        case RETANGLE:
        {
            //如果鼠标点击事件触发，则实时画框，初始为mStartPoint，鼠标移动过程中的坐标为mEndPoint
            if (mIsPress_b) cv::rectangle(mCamImage_M,
                          cv::Point(mStartPoint_qp.x(),mStartPoint_qp.y()),
                          cv::Point(mEndPoint_qp.x(),mEndPoint_qp.y()),
                                       cv::Scalar(0,0,255),5);
            int width=mEndPoint_qp.x()-mStartPoint_qp.x();
            int height=mEndPoint_qp.y()-mStartPoint_qp.y();
            //判断是往左边画框（取mEndPoint为起点，mStartPoint为终点）还是右边画框（取mStartPoint为起点，mEndPoint为终点），并储存Rect
            gDetectionRange_R.x=width>0?mStartPoint_qp.x():mEndPoint_qp.x();
            gDetectionRange_R.y=height>0?mStartPoint_qp.y():mEndPoint_qp.y();
            gDetectionRange_R.width=abs(mEndPoint_qp.x()-mStartPoint_qp.x());
            gDetectionRange_R.height=abs(mEndPoint_qp.y()-mStartPoint_qp.y());

//            qDebug()<<gDetectionRange_R.x<<gDetectionRange_R.y<<gDetectionRange_R.width<<gDetectionRange_R.height;
            break;
        }
        case CIRCLE:
        {
            int width=mEndPoint_qp.x()-mStartPoint_qp.x();
            int height=mEndPoint_qp.y()-mStartPoint_qp.y();
            //判断是往左边画框（取mEndPoint为起点，mStartPoint为终点）还是右边画框（取mStartPoint为起点，mEndPoint为终点），并储存Rect
            gDetectionRange_R.x=width>0?mStartPoint_qp.x():mEndPoint_qp.x();
            gDetectionRange_R.y=height>0?mStartPoint_qp.y():mEndPoint_qp.y();
            gDetectionRange_R.width=abs(mEndPoint_qp.x()-mStartPoint_qp.x());
            gDetectionRange_R.height=abs(mEndPoint_qp.y()-mStartPoint_qp.y());
            //画椭圆
            mRoRect_RR.center=cv::Point2f(gDetectionRange_R.x+gDetectionRange_R.width/2, gDetectionRange_R.y+gDetectionRange_R.height/2);
            mRoRect_RR.size=cv::Size2f(gDetectionRange_R.width, gDetectionRange_R.height);
            mRoRect_RR.angle=0;  //可放局部变量。（考虑减少一下全局变量的数量）
            cv::ellipse(mCamImage_M, mRoRect_RR, cv::Scalar(0, 0, 255), 3);

            break;
        }
        case POLYGON:
            if (mIsPolyClosed_b && mPolyPoints_v_P.size()>=3)
            {
                int min_x=mCamImage_M.rows,max_x=0,min_y=mCamImage_M.cols,max_y=0;
                //多边形已闭合，获得外矩形框并画出每一条多边形线条
                for (auto i=mPolyPoints_v_P.begin();i!=mPolyPoints_v_P.end()-1;i++)
                {
                    min_x=(*i).x<min_x?(*i).x:min_x;
                    max_x=(*i).x>max_x?(*i).x:max_x;
                    min_y=(*i).y<min_y?(*i).y:min_y;
                    max_y=(*i).y>max_y?(*i).y:max_y;

                    cv::line(mCamImage_M,*i,*(i+1),cv::Scalar(0,0,255),3);
                }
                cv::line(mCamImage_M,*(mPolyPoints_v_P.end()-1),mPolyPoints_v_P[0],cv::Scalar(0,0,255),3);
                //获得Rect
                gDetectionRange_R.x=min_x;
                gDetectionRange_R.y=min_y;
                gDetectionRange_R.width=max_x-min_x;
                gDetectionRange_R.height=max_y-min_y;
            }
            else if (mPolyPoints_v_P.size()&&mPolyPoints_v_P.size()>1)
            {
                //多边形未闭合，画出当前所有线条
                mIsPolyClosed_b=false;
                for (auto i=mPolyPoints_v_P.begin();i!=mPolyPoints_v_P.end()-1;i++)
                {
                    cv::line(mCamImage_M,*i,*(i+1),cv::Scalar(0,0,255),3);
                }                
            }else if(mPolyPoints_v_P.size()==1)//第一个点
            {
                mIsPolyClosed_b=false;
                cv::circle(mCamImage_M,mPolyPoints_v_P[0],1,cv::Scalar(0,0,255),3);
            }
            break;
        case DEFALT:
            //默认侦测范围
            gDetectionRange_R.x=0;
            gDetectionRange_R.y=0;
            gDetectionRange_R.width=mCamImage_M.cols;
            gDetectionRange_R.height=mCamImage_M.rows*0.5;
            break;
        }

        if (mIsDetection_b) ProcessImage_v();
        cv::cvtColor(mCamImage_M,mCamImage_M,CV_BGR2RGB);
        this->update();
    }
    else
    {
        //无法获取视频画面则立刻释放资源并发出通知
        CloseCamera_v();
        ui->status->setText(tr("Camera lost."));
    }
}

//ProcessImage
void MainWindow::ProcessImage_v()
{
    //先获取每一个图形的Rect
    cv::Mat rect_range;
    switch (mRectType_i)
    {
    case RETANGLE:
    {
        rect_range=mCamImage_M(gDetectionRange_R);
        break;
    }
    case CIRCLE:
    {
        cv::Mat circle_mask;
        circle_mask = cv::Mat::zeros(mCamImage_M.rows, mCamImage_M.cols, CV_8UC1);
        cv::ellipse(circle_mask, mRoRect_RR, cv::Scalar(255, 255, 255), -1);
        cv::bitwise_and(mCamImage_M, mCamImage_M, rect_range,circle_mask);
        rect_range=rect_range(gDetectionRange_R);
        break;
    }
    case POLYGON:
    {
        cv::Mat polygon_mask;
        polygon_mask=cv::Mat::zeros(mCamImage_M.rows,mCamImage_M.cols,CV_8UC1);
        const cv::Point* ppt[1] = { &mPolyPoints_v_P[0] };
        int npt[] = { mPolyPoints_v_P.size() };
        fillPoly(polygon_mask, ppt, npt, 1, cv::Scalar(255, 255, 255));

        cv::bitwise_and(mCamImage_M, mCamImage_M, rect_range,polygon_mask);
        rect_range=rect_range(gDetectionRange_R);

        break;
    }
    case DEFALT:
        rect_range=mCamImage_M(gDetectionRange_R);
        cv::rectangle(mCamImage_M,gDetectionRange_R,cv::Scalar(0,0,255),5);
        break;

    }
    //高斯混合模型
    gBgSubtractor_BGS(rect_range, mMask_M, 0.00005);

    threshold(mMask_M, mMask_M, 180, 255, CV_THRESH_BINARY);

    //进行两次腐蚀
    cv::erode(mMask_M, mMask_M, cv::Mat());
    cv::erode(mMask_M, mMask_M, cv::Mat());
    //进行膨胀
    cv::dilate(mMask_M, mMask_M, cv::Mat());

    //获得轮廓点
    cv::Mat forContours;
    std::vector<std::vector<cv::Point>> contours;
    forContours.zeros(mCamImage_M.rows,mCamImage_M.cols,CV_8UC3);
    mMask_M.copyTo(forContours);
    cv::findContours(forContours,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//    cv::drawContours(mCamImage_M,contours,-1,cv::Scalar(0,255,0),1);//描绘轮廓

    //起始点
    cv::Point mClickPoint_P=cv::Point(gDetectionRange_R.x,gDetectionRange_R.y);
    if (contours.size())
    {
        //取点数最多的轮廓的一组坐标点后再取左上及右下坐标点
        std::sort(contours.begin(),contours.end(),gSortBySize_b);

        bool isStop=false;
        int square_sum=0;

        for (auto it=contours.begin();it!=contours.end() && !isStop;it++)
        {
            if (it->size()>=mIdentification_uc)
            {
                cv::RotatedRect rotrect=cv::minAreaRect(*it);
                cv::Rect boundingbox=rotrect.boundingRect()+mClickPoint_P;

                if (boundingbox.x+boundingbox.width>gDetectionRange_R.width+gDetectionRange_R.x)
                    boundingbox.width=gDetectionRange_R.width+gDetectionRange_R.x-boundingbox.x;
                if(boundingbox.x<gDetectionRange_R.x)
                    boundingbox.x=gDetectionRange_R.x;
                if(boundingbox.y+boundingbox.height>gDetectionRange_R.height+gDetectionRange_R.y)
                    boundingbox.height=gDetectionRange_R.height+gDetectionRange_R.y-boundingbox.y;
                if(boundingbox.y<gDetectionRange_R.y)
                    boundingbox.y=gDetectionRange_R.y;

                cv::rectangle(mCamImage_M,boundingbox,cv::Scalar(255,0,0),3);

                square_sum+=boundingbox.width*boundingbox.height;
            }
            else
            {
                isStop=true;
            }
        }

        //设定报警阈值（按照面积占比计算） 
        square_sum>=(gDetectionRange_R.height*gDetectionRange_R.width*mAlarmSen_f)?
                    SetAlarm_v(true):SetAlarm_v(false);
    }

}

void MainWindow::Playwav_v()
{
    mIsFirstWav_b=true;
}

void MainWindow::SetAlarm_v(bool isAlarm)
{
    if (isAlarm)
    {
        ui->Alarming->setVisible(true);

        //两秒触发一次
       if (mIsFirstWav_b)
       {
           QSound::play(":/alarm/alarming.wav");
           mWavTimer_qt.start(2350);
           mIsFirstWav_b=false;
       }
    }
    else
    {
        ui->Alarming->setVisible(false);
    }

}

//鼠标点击事件
void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (mIsAction_b/*考虑左右键press问题*/)
    {
        mIsPress_b=false;
        switch (mRectType_i)
        {
        //设置矩形及椭圆的起始坐标点
        case RETANGLE:
        case CIRCLE:
            SetDetection_v(false);
            //鼠标点击范围判断
            if (event->x()-ui->label->x()>=ui->label->width())
                mStartPoint_qp.setX(ui->label->x()+ui->label->width()-15);
            else if(event->x()<ui->label->x())
                mStartPoint_qp.setX(ui->label->x()-7);
            else
            mStartPoint_qp.setX(event->x()-ui->label->x());

            if(event->y()-ui->label->y()>=ui->label->height())
                mStartPoint_qp.setY(ui->label->y()+ui->label->height()-15);
            else if(event->y()<ui->label->y())
                mStartPoint_qp.setY(ui->label->y()-7);
            else
                mStartPoint_qp.setY(event->y()-ui->label->y());
            mEndPoint_qp=mStartPoint_qp;
            mIsPress_b=true;
            break;
        case POLYGON:
        {
            if (mIsPolyClosed_b)
            {
                SetDetection_v(false);
                mIsPolyClosed_b=false;
                mPolyPoints_v_P.clear();
            }

        //鼠标点击范围判断以及坐标转换
            if (event->x()-ui->label->x()>=ui->label->width())
                mClickPoint_P.x=ui->label->x()+ui->label->width()-15;
            else if(event->x()<ui->label->x())
                mClickPoint_P.x=ui->label->x()-7;
            else
                mClickPoint_P.x=event->x()-ui->label->x();

            if(event->y()-ui->label->y()>=ui->label->height())
                mClickPoint_P.y=ui->label->y()+ui->label->height()-15;
            else if(event->y()<ui->label->y())
                mClickPoint_P.y=ui->label->y()-7;
            else
                mClickPoint_P.y=event->y()-ui->label->y();

            mIsPress_b=true;


            break;
        }
        case DEFALT:
            break;
        }

//        qDebug()<<"mousePress"<<mStartPoint_qp;
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (mIsAction_b)
    {
    switch (mRectType_i)
    {
    case RETANGLE:
    case CIRCLE:
        //get QPoint->opencvPoint && not out of bounding.
        //鼠标坐标点范围判断以及坐标转换
        if (event->x()-ui->label->x()>=ui->label->width())
            mEndPoint_qp.setX(ui->label->x()+ui->label->width()-15);
        else if(event->x()<ui->label->x())
            mEndPoint_qp.setX(ui->label->x()-7);
        else
            mEndPoint_qp.setX(event->x()-ui->label->x());

        if(event->y()-ui->label->y()>=ui->label->height())
            mEndPoint_qp.setY(ui->label->y()+ui->label->height()-15);
        else if(event->y()<ui->label->y())
            mEndPoint_qp.setY(ui->label->y()-7);
        else
            mEndPoint_qp.setY(event->y()-ui->label->y());
        break;

    case POLYGON:
        break;
    case DEFALT:
        break;
    }

//    qDebug()<<"width:"<<ui->label->width()<<"minus:"<<event->x()-ui->label->x();
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (mIsAction_b)
    {
        switch (mRectType_i)
            {
        case RETANGLE:
        case CIRCLE:
            SetDetection_v(true);
        break;
        case POLYGON:
        {
            cv::Point lastClickPoint;//lastclickpoint and mClickPoint_P ? 考虑一次push两个？
            //坐标点范围检测及坐标转换
            if (event->x()-ui->label->x()>=ui->label->width())
                lastClickPoint.x=ui->label->x()+ui->label->width()-15;
            else if(event->x()<ui->label->x())
                lastClickPoint.x=ui->label->x()-7;
            else
                lastClickPoint.x=event->x()-ui->label->x();
            if(event->y()-ui->label->y()>=ui->label->height())
                lastClickPoint.y=ui->label->y()+ui->label->height()-15;
            else if(event->y()<ui->label->y())
                lastClickPoint.y=ui->label->y()-7;
            else
                lastClickPoint.y=event->y()-ui->label->y();
            //自动闭合(auto closed)
            if (mPolyPoints_v_P.size()>2 &&
                    abs(lastClickPoint.x-mPolyPoints_v_P[0].x)<=20 &&
                    abs(lastClickPoint.y-mPolyPoints_v_P[0].y)<=20)
            {
                mPolyPoints_v_P.push_back(mClickPoint_P);
                mIsPolyClosed_b=true;
                SetDetection_v(true);
            }
            if (mIsPress_b && !mIsPolyClosed_b) mPolyPoints_v_P.push_back(mClickPoint_P);
            break;
        }
        case DEFALT:
            break;
        }
    }
}

void MainWindow::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (mIsAction_b)
    {
//    qDebug()<<"DoubleClick";
    if (mRectType_i==POLYGON && mPolyPoints_v_P.size()>2)
    {
        mPolyPoints_v_P.push_back(mClickPoint_P);
        mIsPolyClosed_b=true;
        SetDetection_v(true);
    }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (mIsAction_b)
    {
    if (event->key()==Qt::Key_Delete)
    {
        if (mRectType_i==POLYGON && !mIsPolyClosed_b && !mPolyPoints_v_P.empty())
            mPolyPoints_v_P.pop_back();
    }
    }
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_mOpenCamera_bt_clicked()
{
    ShowCamera_v();
}

void MainWindow::on_mPauseCamera_bt_clicked()
{
    PauseCamera_v();
}

void MainWindow::on_mStopCamera_bt_clicked()
{
    CloseCamera_v();
}

void MainWindow::on_mDetection_bt_clicked()
{
    if (!mIsDetection_b)
    {
        SetDetection_v(true);
    }
    else
    {
        SetDetection_v(false);
    }
}

void MainWindow::SetDetection_v(bool onoff)
{
    if (onoff && gDetectionRange_R.width && gDetectionRange_R.height)
    {
        mIsDetection_b=true;
        ui->mDetection_bt->setText(tr("StopDetection"));
        ui->mAlarmSensitivity_sld->setEnabled(true);
        ui->mRgnSensitivity_sld->setEnabled(true);
    }
    else
    {
        mIsDetection_b=false;
        ui->mDetection_bt->setText(tr("StartDetection"));
        SetAlarm_v(false);
        ui->mAlarmSensitivity_sld->setEnabled(false);
        ui->mRgnSensitivity_sld->setEnabled(false);
    }
}

void MainWindow::on_mRectangle_bt_clicked()
{
    if (mIsAction_b)
    {
        if (mRectType_i!=RETANGLE)
        {
            mStartPoint_qp.setX(0);
            mStartPoint_qp.setY(0);
            mEndPoint_qp.setX(0);
            mEndPoint_qp.setY(0);
            SetDetection_v(false);
        }
        ui->mDeletePoint_bt->setEnabled(false);
        mRectType_i=RETANGLE;
    }
}

void MainWindow::on_mCircle_bt_clicked()
{
    if (mIsAction_b)
    {
        if (mRectType_i!=CIRCLE)
        {
            mStartPoint_qp.setX(0);
            mStartPoint_qp.setY(0);
            mEndPoint_qp.setX(0);
            mEndPoint_qp.setY(0);
            SetDetection_v(false);
        }
        ui->mDeletePoint_bt->setEnabled(false);
        mRectType_i=CIRCLE;
    }
}

void MainWindow::on_mPolygon_bt_clicked()
{
    if (mIsAction_b)
    {
        if (mRectType_i!=POLYGON)
        {
            mStartPoint_qp.setX(0);
            mStartPoint_qp.setY(0);
            mEndPoint_qp.setX(0);
            mEndPoint_qp.setY(0);
        }
        SetDetection_v(false);
        mIsPolyClosed_b=false;
        mPolyPoints_v_P.clear();
        ui->mDeletePoint_bt->setEnabled(true);
        mRectType_i=POLYGON;
    }
}

void MainWindow::on_mDeletePoint_bt_clicked()
{
    if (mIsAction_b)
    {
    if (mRectType_i==POLYGON && !mIsPolyClosed_b && !mPolyPoints_v_P.empty())
        mPolyPoints_v_P.pop_back();
    }
}

void MainWindow::on_mRgnSensitivity_sld_valueChanged(int value)
{
    mIdentification_uc=value;
}

void MainWindow::on_mAlarmSensitivity_sld_valueChanged(int value)
{
    mAlarmSen_f=(float)value/100;
}
