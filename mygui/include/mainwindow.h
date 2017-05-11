#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QTime>
#include <QTimer>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QTabWidget>
#include <QLabel>
#include <QHeaderView>
#include <QPrinter>
#include <sstream>
#include <QStandardItemModel>
#include "qcustomplot.h"
#include <QElapsedTimer>
#include <QTextStream>
#include <QObject>
#include <QFile>
#include <QString>
#include <iostream>
#include <string>
#include <QSignalMapper>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include <sstream>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define SquareAmplitudeV 6000 //mVolts
#define SquareAmplitudeA 1200 //mAmpere
#define SquareAmplitudeAlow 50 //mAmpere
#define SquarePeriod 5400 //msec
#define SinAmplitude 6000 //mVolts
#define SinPeriod 4000 //msec

#define VoltageLimit 12000 //mVolts
#define PWMlimit 4095 //mVolts
#define CurrentLimit 1200 //mA
#define CurrentRMSLimit 1200 //mA
#define ResistanceLimit 16 //Ohm
#define ResistanceRMSLimit 20 //Ohm

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    QTimer *timer,*timer2;
    QElapsedTimer *Exp_time,*Reference_time;
    QStandardItemModel *POTmodel,*SMAmodel;
    QCustomPlot* plot;
    QComboBox *monitor_channel, *monitor_attribute;
    QSignalMapper *m_sigmapper;
    //QFile *outputFile;
    //QTextStream *outputStream;

    int SMArows;
    int SMAcolumns;
    u_int8_t RMSresistance;

    float ADy_prev[6*8];
    float ADx_prev[6*8];
    float mya[6],myb[6];

    int POTrows;
    int POTcolumns;

    double convertPOT;

    ros::Subscriber gui_listen;
    ros::Publisher gui_pub;
    std_msgs::String* pub_str;
    ros::NodeHandle *m,*n;

    enum ctrlFlag{
        cmdOnOff=0x00,
        cmdPWM  =0x01,
        cmdVolt =0x02,
        cmdIm   =0x03,
        cmdIrms =0x04,
        cmdRate =0x05
    };

    struct smaStatus{
        float AD[8]; //32
        float ADrms[8]; //64
        float Vbus; //68
        u_int32_t time; //80
        int16_t ADCV[16]; //80+32=112
        u_int16_t PWMval[8];
        u_int8_t cksum[2]; //114
    }; //114bytes

public Q_SLOTS:

private Q_SLOTS:
    void pltUpd(void);
    void GUIUpd(void);
    void on_stopButton_clicked(void);
    void on_mainSwitch_clicked(bool );
    void on_ManualControl_clicked(bool);
    void on_ControllingMean_currentIndexChanged(int val);
    void on_controllMethod_currentIndexChanged(int val);
    void on_sendUpdate_clicked(void);
    void CTRLUIinit(void);
    void SMAdisabled(void);
    void onPlotVisualizationChannelChange(int);
    void onPlotVisualizationAttributeChange(int);
    void on_Reference_currentIndexChanged(int);

private:
    Ui::MainWindow *ui;
    QSlider *SMAslider, *POTslider;
    QSpinBox *SMAspinbox;
    QCheckBox *SMAcheckbox;
    bool automanual;
    u_int8_t reference_type;
};

#endif // MAINWINDOW_H
