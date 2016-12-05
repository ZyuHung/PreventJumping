#include "mainwindow.h"
#include "ui_mainwindow.h"

cv::Rect range;
cv::BackgroundSubtractorMOG2 bgSubtractor;

bool sortbysize(std::vector<cv::Point> &v1,std::vector<cv::Point> &v2)
{
    return v1.size()>v2.size();
}

bool sortbypoints(cv::Point &v1, cv::Point &v2)
{
    if (v1.x != v2.x)
        return v1.x < v2.x;
    else return v1.y < v2.y;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qDebug()<<"MainWindow";
    ui->setupUi(this);
    ui->Alarming->setVisible(false);
    setWindowFlags(windowFlags()& ~Qt::WindowMaximizeButtonHint);
    connect(&mUpdatingTimer_qt,&QTimer::timeout,this,&MainWindow::UpdateImage_v);
    connect(&mWavTimer_qt,&QTimer::timeout,this,&MainWindow::Playwav_v);
}


void MainWindow::ShowCamera_v()
{
    if (mCapture_VC.open(CAM_NO))
    {
        mCapture_VC>>mCamImage_M;
        cv::cvtColor(mCamImage_M,mCamImage_M,CV_BGR2RGB);
        ui->status->setText(tr("Camera open."));
        mUpdatingTimer_qt.start(30);
        mIsAction_b=true;
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->Button_decetion->setEnabled(true);
        ui->Button_retangle->setEnabled(true);
        ui->Button_polygon->setEnabled(true);
        ui->Button_circle->setEnabled(true);
    }
    else
    {
        ui->status->setText(tr("Camera open FAILED."));
    }
}

void MainWindow::PauseCamera_v()
{
    if (ui->pushButton_2->text()=="PauseCamera")
    {
        ui->pushButton_2->setText(tr("ContinueCamera"));
        ui->status->setText(tr("Pause"));
        mIsAction_b=false;
        mUpdatingTimer_qt.stop();

    }else
    {
        mUpdatingTimer_qt.start(30);
        mIsAction_b=true;
        ui->pushButton_2->setText(tr("PauseCamera"));
        ui->status->setText(tr("Camera open."));
    }

}

void MainWindow::CloseCamera_v()
{
    mUpdatingTimer_qt.stop();
    SetDetection_v(false);
    mRectType_i=DEFALT;
    mCamImage_M.release();
    ui->status->setText(tr("Camera closed."));
    ui->pushButton->setEnabled(true);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_2->setText(tr("PauseCamera"));
    ui->pushButton_3->setEnabled(false);
    ui->Button_decetion->setEnabled(false);
    ui->Button_retangle->setEnabled(false);
    ui->Button_polygon->setEnabled(false);
    ui->Button_circle->setEnabled(false);
    ui->DeletePoint->setEnabled(false);
    mCapture_VC.release();
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QImage image2 = QImage((uchar*)(mCamImage_M.data), mCamImage_M.cols, mCamImage_M.rows, QImage::Format_RGB888);
    ui->label->setPixmap(QPixmap::fromImage(image2));
    ui->label->resize(image2.size());
    ui->label->show();
    setFixedSize(this->width(), this->height());
}

void MainWindow::UpdateImage_v()
{
    mCapture_VC>>mCamImage_M;
    if (mCamImage_M.data)
    {
        ui->status->setText(tr("Camera open."));

        switch (mRectType_i)
        {
        case RETANGLE:
        {
            if (mIsPress_b) cv::rectangle(mCamImage_M,
                          cv::Point(mPointStart_qp.x(),mPointStart_qp.y()),
                          cv::Point(mPointEnd_qp.x(),mPointEnd_qp.y()),
                                       cv::Scalar(0,0,255),5);
            int width=mPointEnd_qp.x()-mPointStart_qp.x();
            int height=mPointEnd_qp.y()-mPointStart_qp.y();
            range.x=width>0?mPointStart_qp.x():mPointEnd_qp.x();
            range.y=height>0?mPointStart_qp.y():mPointEnd_qp.y();
            range.width=abs(mPointEnd_qp.x()-mPointStart_qp.x());
            range.height=abs(mPointEnd_qp.y()-mPointStart_qp.y());
            qDebug()<<range.x<<range.y<<range.width<<range.height;
            break;
        }
        case CIRCLE:
        {
            int width=mPointEnd_qp.x()-mPointStart_qp.x();
            int height=mPointEnd_qp.y()-mPointStart_qp.y();
            range.x=width>0?mPointStart_qp.x():mPointEnd_qp.x();
            range.y=height>0?mPointStart_qp.y():mPointEnd_qp.y();

            range.width=abs(mPointEnd_qp.x()-mPointStart_qp.x());
            range.height=abs(mPointEnd_qp.y()-mPointStart_qp.y());

            mRoRect_RR.center=cv::Point2f(range.x+range.width/2, range.y+range.height/2);
            mRoRect_RR.size=cv::Size2f(range.width, range.height);
            mRoRect_RR.angle=0;  //可放局部变量。（考虑减少一下全局变量的数量）
            cv::ellipse(mCamImage_M, mRoRect_RR, cv::Scalar(0, 0, 255), 3);

            break;
        }
        case POLYGON:
            if (mIsPolyClosed_b && mPolyPoints_v_P.size()>=3)
            {
                int min_x=mCamImage_M.rows,max_x=0,min_y=mCamImage_M.cols,max_y=0;
                for (auto i=mPolyPoints_v_P.begin();i!=mPolyPoints_v_P.end()-1;i++)
                {
                    min_x=(*i).x<min_x?(*i).x:min_x;
                    max_x=(*i).x>max_x?(*i).x:max_x;
                    min_y=(*i).y<min_y?(*i).y:min_y;
                    max_y=(*i).y>max_y?(*i).y:max_y;

                    cv::line(mCamImage_M,*i,*(i+1),cv::Scalar(0,0,255),3);
                }
                cv::line(mCamImage_M,*(mPolyPoints_v_P.end()-1),mPolyPoints_v_P[0],cv::Scalar(0,0,255),3);
                range.x=min_x;
                range.y=min_y;
                range.width=max_x-min_x;
                range.height=max_y-min_y;
            }
            else if (mPolyPoints_v_P.size()&&mPolyPoints_v_P.size()>1)
            {
                mIsPolyClosed_b=false;
                for (auto i=mPolyPoints_v_P.begin();i!=mPolyPoints_v_P.end()-1;i++)
                {
                    cv::line(mCamImage_M,*i,*(i+1),cv::Scalar(0,0,255),3);
                }
            }else if(mPolyPoints_v_P.size()==1)
            {
                mIsPolyClosed_b=false;
                cv::circle(mCamImage_M,mPolyPoints_v_P[0],1,cv::Scalar(0,0,255),3);
            }
            break;
        case DEFALT:
            range.x=0;
            range.y=0;
            range.width=mCamImage_M.cols;
            range.height=mCamImage_M.rows*0.5;
            break;
        }

        if (mIsDetection_b) ProcessImage_v();
        cv::cvtColor(mCamImage_M,mCamImage_M,CV_BGR2RGB);
        this->update();
    }
    else
    {
        CloseCamera_v();
        ui->status->setText(tr("Camera lost."));
    }
}

void MainWindow::ProcessImage_v()
{
    cv::Mat rect_range;
    switch (mRectType_i)
    {
    case RETANGLE:
    {
        rect_range=mCamImage_M(range);
        break;
    }
    case CIRCLE:
    {
        cv::Mat circle_mask;
        circle_mask = cv::Mat::zeros(mCamImage_M.rows, mCamImage_M.cols, CV_8UC1);
        cv::ellipse(circle_mask, mRoRect_RR, cv::Scalar(255, 255, 255), -1);
        cv::bitwise_and(mCamImage_M, mCamImage_M, rect_range,circle_mask);
        rect_range=rect_range(range);
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
        rect_range=rect_range(range);

        break;
    }
    case DEFALT:
        rect_range=mCamImage_M(range);
        cv::rectangle(mCamImage_M,range,cv::Scalar(0,0,255),5);
        break;

    }
    bgSubtractor(rect_range, mMask_M, 0.00005);

    threshold(mMask_M, mMask_M, 180, 255, CV_THRESH_BINARY);

    cv::erode(mMask_M, mMask_M, cv::Mat());
    cv::erode(mMask_M, mMask_M, cv::Mat());
    cv::dilate(mMask_M, mMask_M, cv::Mat());

    cv::Mat forContours;
    std::vector<std::vector<cv::Point>> contours;
    forContours.zeros(mCamImage_M.rows,mCamImage_M.cols,CV_8UC3);
    mMask_M.copyTo(forContours);
    cv::findContours(forContours,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//    cv::drawContours(mCamImage_M,contours,-1,cv::Scalar(0,255,0),3);

    cv::Point mClickPoint_P=cv::Point(range.x,range.y);
    if (contours.size())
    {
        std::sort(contours.begin(),contours.end(),sortbysize);
        for (std::vector<std::vector<cv::Point>>::iterator it = contours.begin(); it != contours.end(); ++it)
            std::sort(it->begin(), it->end(),sortbypoints);
        cv::rectangle(mCamImage_M,contours[0][0]+mClickPoint_P, *(contours[0].end()-1)+mClickPoint_P,cv::Scalar(0,0,255),2);

    int thresold = range.height*range.width*0.10;
    int square=(contours[0][contours[0].size()-1].x-contours[0][0].x)
            *(contours[0][contours[0].size()-1].y-contours[0][0].y);
    if (square>=thresold)
    {
        SetAlarm_v(true);
    }
    else
    {
        SetAlarm_v(false);
    }
    }

}

void MainWindow::Playwav_v()
{
//    QSound::play(":/alarm/alarming.wav");
    mIsFirstWav_b=true;
}

void MainWindow::SetAlarm_v(bool isAlarm)
{
    if (isAlarm)
    {
        ui->Alarming->setVisible(true);

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
//        mWavTimer_qt.stop();
    }

}


void MainWindow::mousePressEvent(QMouseEvent *event)
{//考虑左右键press问题
    if (mIsAction_b)
    {
    mIsPress_b=false;
    switch (mRectType_i)
    {
    case RETANGLE:
    case CIRCLE:
        SetDetection_v(false);
        if (event->x()-ui->label->x()>=ui->label->width())
            mPointStart_qp.setX(ui->label->x()+ui->label->width()-15);
        else if(event->x()<ui->label->x())
            mPointStart_qp.setX(ui->label->x()-7);
            else
            mPointStart_qp.setX(event->x()-ui->label->x());

        if(event->y()-ui->label->y()>=ui->label->height())
            mPointStart_qp.setY(ui->label->y()+ui->label->height()-15);
        else if(event->y()<ui->label->y())
            mPointStart_qp.setY(ui->label->y()-7);
        else
            mPointStart_qp.setY(event->y()-ui->label->y());
        mPointEnd_qp=mPointStart_qp;
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

    qDebug()<<"mousePress"<<mPointStart_qp;
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
        if (event->x()-ui->label->x()>=ui->label->width())
            mPointEnd_qp.setX(ui->label->x()+ui->label->width()-15);
        else if(event->x()<ui->label->x())
            mPointEnd_qp.setX(ui->label->x()-7);
        else
            mPointEnd_qp.setX(event->x()-ui->label->x());

        if(event->y()-ui->label->y()>=ui->label->height())
            mPointEnd_qp.setY(ui->label->y()+ui->label->height()-15);
        else if(event->y()<ui->label->y())
            mPointEnd_qp.setY(ui->label->y()-7);
        else
            mPointEnd_qp.setY(event->y()-ui->label->y());
        break;

    case POLYGON:
        break;
    case DEFALT:
        break;
    }

    qDebug()<<"width:"<<ui->label->width()<<"minus:"<<event->x()-ui->label->x();
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
        cv::Point lastClickPoint;
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
                abs(lastClickPoint.x-mPolyPoints_v_P[0].x)<=20 && abs(lastClickPoint.y-mPolyPoints_v_P[0].y)<=20)
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
    qDebug()<<"DoubleClick";
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

void MainWindow::on_pushButton_clicked()
{
    ShowCamera_v();
}

void MainWindow::on_pushButton_2_clicked()
{
    PauseCamera_v();
}

void MainWindow::on_pushButton_3_clicked()
{
    CloseCamera_v();
}

void MainWindow::on_Button_mIsDetection_b_clicked()
{
    if (ui->Button_decetion->text()=="StartDetection")
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
    if (onoff && range.width && range.height)
    {
        mIsDetection_b=true;
        ui->Button_decetion->setText(tr("StopDetection"));
    }
    else
    {
        mIsDetection_b=false;
        ui->Button_decetion->setText(tr("StartDetection"));
        SetAlarm_v(false);
    }
}

void MainWindow::on_Button_retangle_clicked()
{
    if (mIsAction_b)
    {
    if (mRectType_i!=RETANGLE)
    {
        mPointStart_qp.setX(0);
        mPointStart_qp.setY(0);
        mPointEnd_qp.setX(0);
        mPointEnd_qp.setY(0);
        SetDetection_v(false);
    }
    ui->DeletePoint->setEnabled(false);
    mRectType_i=RETANGLE;
    }
}

void MainWindow::on_Button_circle_clicked()
{
    if (mIsAction_b)
    {
    if (mRectType_i!=CIRCLE)
    {
        mPointStart_qp.setX(0);
        mPointStart_qp.setY(0);
        mPointEnd_qp.setX(0);
        mPointEnd_qp.setY(0);
        SetDetection_v(false);
    }
    ui->DeletePoint->setEnabled(false);
    mRectType_i=CIRCLE;
    }
}

void MainWindow::on_Button_polygon_clicked()
{
    if (mIsAction_b)
    {
    if (mRectType_i!=POLYGON)
    {

        mPointStart_qp.setX(0);
        mPointStart_qp.setY(0);
        mPointEnd_qp.setX(0);
        mPointEnd_qp.setY(0);

    }
    SetDetection_v(false);
    mIsPolyClosed_b=false;
    mPolyPoints_v_P.clear();
    ui->DeletePoint->setEnabled(true);
    mRectType_i=POLYGON;
}
}

void MainWindow::on_DeletePoint_clicked()
{
    if (mIsAction_b)
    {
    if (mRectType_i==POLYGON && !mIsPolyClosed_b && !mPolyPoints_v_P.empty())
        mPolyPoints_v_P.pop_back();
    }
}

