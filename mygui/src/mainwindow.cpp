#include "mainwindow.h"
#include "ui_mainwindow.h"

#define BIN 108
#define BOUT 5

bool publish;

MainWindow::smaStatus smaState;
MainWindow::ctrlFlag ctrlFlg;
cv::Mat imag;
QImage imdisplay;
bool recv=false;

void boardCallback(const std_msgs::Int8MultiArray::ConstPtr& array)
{
    if(array->data[BIN-1]==array->data[BIN-2] && array->data[BIN-1]==33){
        int i;
        // print all the remaining numbers
        u_int8_t* ptr=(u_int8_t *)&smaState;
        for(i=0; i<BIN-2; i++)
        {
            ptr[i] = array->data[i];
            //printf("%d, ",ptr[i]);
        }
        //ROS_INFO("success receice");
    }
    else{
        ROS_INFO("success error!!!!!!!!!");
    }
    return;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    imag=cv_bridge::toCvShare(msg, "bgr8")->image;
    recv=true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Exception");
  }
}

MainWindow::MainWindow(int init_argc, char** init_argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ROS_INFO("start mwindow");
    //u_int8_t* ptr=(u_int8_t *)&smaState;
    ros::init(init_argc,init_argv,"mygui");
    //INIT ROS HANDLER FOR BOARD PUBLISHER/SUBSCRIBER
    ros::NodeHandle n;
    pub_str=new std_msgs::String[BOUT];
    gui_pub = n.advertise<std_msgs::String>("board_commands", 1);
    //INIT ROS HANDLER FOR SUBSCRIBER
    gui_listen = n.subscribe("electronics", 1, boardCallback);
    //INIT ROS HANDLER FOR IMAGE PUBLISHER/SUBSCRIBER
    ros::NodeHandle img;
    image_transport::ImageTransport igm(img);
    image_transport::Subscriber Imsub = igm.subscribe("robot/image_aruco", 1, imageCallback);
    //while(gui_pub.getNumSubscribers()<=0){
    //	ROS_INFO("waiting to subscribe");
    //}
    publish=false;
    //GUI STUFF
    RMSresistance=0;
    myb[0]=0.0*1.0e8;
    myb[1]=0.0*1.0e8;
    myb[2]=0.0*1.0e8;
    myb[3]=0.0*1.0e8;
    myb[4]=0.0*1.0e8;
    myb[5]=9.792629913129004*1.0e8;
    mya[0]=1.0;
    mya[1]=-2.033281476926104*1.0e2;
    mya[2]=-2.067116782205399*1.0e4;
    mya[3]=-1.298807779417730*1.0e6;
    mya[4]=-5.043559043399953*1.0e7;
    mya[5]=-9.792629913129004*1.0e8;
    for(int i=0; i<48;i++){
            ADy_prev[i]=0.0;
            ADx_prev[i]=0.0;
    }
    ////////////////////////////////////////////////////////////////////////////////////
    //////////////// ALL GUI VARIABLES
    /// ////////////////////////////////////////////////////////////////////////////////
    //qint16 width=1600;
    //qint16 height=900;
    //positions first
    //ui->centralWidget->setMinimumSize(width,height);

    // Create a new model for SMA
    SMArows=3;
    SMAcolumns=8;
    SMAmodel = new QStandardItemModel(2*SMArows,SMAcolumns/2,this);
    // Attach the model to the view
    ui->SMAview->setModel(SMAmodel);
    //ui->SMAview->setRowHeight(0,300);
    //ui->SMAview->setRowHeight(0,20);
    //ui->SMAview->setRowHeight(0,20);
    // Generate SMA data
    SMAslider=new QSlider[SMAcolumns];
    SMAspinbox=new QSpinBox[SMAcolumns];
    SMAcheckbox=new QCheckBox[SMAcolumns];
    for(int row=0;row<2;row++){
    for(int col = 0; col < SMAcolumns/2; col++)
    {
        //QSlider *s;
        //QSpinBox *d;
        //s=new QSlider;
        //d=new QSpinBox;
        //s->setMaximum(168e6/2/20000-1);
        //SMAslider[row*4+col].setMaximum(4095);
        //SMAslider[row*4+col].setMinimum(0);
        SMAslider[row*4+col].setRange(0,4095);
        SMAslider[row*4+col].setSingleStep(1);
        //d->setMaximum(168e6/2/20000-1);
        //SMAspinbox[row*4+col].setMaximum(4095);
        //SMAspinbox[row*4+col].setMinimum(0);
        SMAspinbox[row*4+col].setRange(0,4095);
        SMAspinbox[row*4+col].setSingleStep(1);
        //connect(&SMAslider[row*4+col],SIGNAL(sliderMoved(int)),&SMAspinbox[row*4+col],SLOT(setValue(int)));
        connect(&SMAslider[row*4+col],SIGNAL(valueChanged(int)),&SMAspinbox[row*4+col],SLOT(setValue(int)));
        connect(&SMAspinbox[row*4+col],SIGNAL(valueChanged(int)),&SMAslider[row*4+col],SLOT(setValue(int)));
        connect(&SMAslider[row*4+col],SIGNAL(valueChanged(int)),this,SLOT(CTRLUIinit()));
        connect(&SMAcheckbox[row*4+col],SIGNAL(toggled(bool)),&SMAslider[row*4+col],SLOT(setEnabled(bool)));
        //connect(&SMAcheckbox[row*4+col],SIGNAL(toggled(bool)),this,SLOT(SMAdisabled()));
        connect(&SMAcheckbox[row*4+col],SIGNAL(toggled(bool)),&SMAspinbox[row*4+col],SLOT(setEnabled(bool)));
        ui->SMAview->setIndexWidget(ui->SMAview->model()->index(row*3, col), &SMAslider[row*4+col]);
        ui->SMAview->setIndexWidget(ui->SMAview->model()->index(row*3+1, col), &SMAspinbox[row*4+col]);
        ui->SMAview->setIndexWidget(ui->SMAview->model()->index(row*3+2, col), &SMAcheckbox[row*4+col]);
        ui->SMAview->horizontalHeader()->setResizeMode(col, QHeaderView::Stretch);

    }
    //ui->SMAview->verticalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->SMAview->verticalHeader()->setResizeMode(0, QHeaderView::Stretch);
    //ui->SMAview->verticalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->SMAview->verticalHeader()->setResizeMode(0, QHeaderView::Stretch);
    //ui->SMAview->verticalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
    ui->SMAview->verticalHeader()->setResizeMode(3, QHeaderView::Stretch);
    ui->SMAview->verticalHeader()->hide();
    ui->SMAview->horizontalHeader()->hide();
    }
    ui->SMAview->setEnabled(0);

    POTrows=2;
    POTcolumns=8;
    POTmodel = new QStandardItemModel(POTrows,POTcolumns,this);

    // Attach the model to the view
    ui->POTview->setModel(POTmodel);
    //for (u_int8_t i=0;i<POTrows;i++)
    //ui->POTview->setRowHeight(i,100);

    // Generate POT data
    POTslider=new QSlider[POTcolumns*POTrows];
    for(int row = 0; row < POTrows; row++)
    {
        for (int col=0; col<POTcolumns; col++){
            //QSlider *s;
            //s=new QSlider(Qt::Horizontal,this);
            POTslider[row*8+col].setOrientation(Qt::Horizontal);
            //POTslider[row*8+col].setMaximum(20); //mm
            //POTslider[row*8+col].setMinimum(0); //normally it is -10V but with potentiometers no less than 0
            POTslider[row*8+col].setRange(0,2000);
            POTslider[row*8+col].setSingleStep(1);
            //ui->POTview->setColumnWidth(col,1090/POTcolumns);
            ui->POTview->setIndexWidget(ui->POTview->model()->index(row,col), &POTslider[row*8+col]);
            //ui->POTview->horizontalHeader()->setSectionResizeMode(col, QHeaderView::Stretch);
            ui->POTview->horizontalHeader()->setResizeMode(col, QHeaderView::Stretch);
        }
        //ui->POTview->verticalHeader()->setSectionResizeMode(row, QHeaderView::Stretch);
        ui->POTview->verticalHeader()->setResizeMode(row, QHeaderView::Stretch);
    }
    //ui->POTview->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    //ui->POTview->verticalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    //Create resistance graphs
    int graphrows=2;
    int graphcolms=SMAcolumns/graphrows;
    ui->tableWidget->setRowCount(graphrows*3); //increased to 3 to see everything
    ui->tableWidget->setColumnCount(graphcolms);
    plot=new QCustomPlot[SMAcolumns];
    monitor_channel=new QComboBox[graphrows*graphcolms];
    monitor_attribute=new QComboBox[graphrows*graphcolms];
    QStringList fonts;
    fonts << "PWM" << "Voltage" << "Current" << "CurrentRMS" << "Resistance" << "ResistanceRMS";
    QStringList channels;
    for (int i=0;i<graphcolms*graphrows;i++){
    channels += QString::number(i+1);
    }

    //PLOT CREATION
    m_sigmapper = new QSignalMapper(this);
    for (int i=0;i<graphcolms*graphrows;i++)
        monitor_channel[i].addItems(channels);
    for (int i=0;i<graphcolms*graphrows;i++)
        monitor_channel[i].setCurrentIndex(i);
    //attribute selector properties
    for (int i=0;i<graphcolms*graphrows;i++)
        monitor_attribute[i].addItems(fonts);
    for (int i=0;i<graphcolms*graphrows;i++)
        monitor_attribute[i].setCurrentIndex(0);
    //add attributes

    for (u_int8_t i=0;i<graphrows;i++){
        //ui->tableWidget->setRowHeight(i,399/2);
        for (u_int8_t j=0;j<graphcolms;j++){
            //ui->tableWidget->setColumnWidth(j,610/4);
            ui->tableWidget->setCellWidget(i*3,j,&(monitor_channel[i*4+j]));
            ui->tableWidget->setCellWidget(i*3+1,j,&(monitor_attribute[i*4+j]));
            connect(&monitor_channel[i*4+j],SIGNAL(currentIndexChanged(int)),this,SLOT(onPlotVisualizationChannelChange(int)));
            connect(&monitor_attribute[i*4+j],SIGNAL(currentIndexChanged(int)),this,SLOT(onPlotVisualizationAttributeChange(int)));
            ui->tableWidget->setCellWidget(i*3+2,j,&(plot[i*4+j]));
            plot[i*4+j].addGraph(); // blue line
            plot[i*4+j].graph(0)->setPen(QPen(Qt::blue));
            plot[i*4+j].graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
            plot[i*4+j].graph(0)->setAntialiasedFill(false);
            plot[i*4+j].setInteraction(QCP::iRangeDrag, true);
            plot[i*4+j].setInteraction(QCP::iRangeZoom, true);
            plot[i*4+j].yAxis->setNumberPrecision(4);
            plot[i*4+j].yAxis->setNumberFormat("f");
            plot[i*4+j].xAxis->setLabel("time (sec)");
            plot[i*4+j].yAxis->setRange(0,4096);
            QString str;
            str+="SMA no.";
            str+=(i*4+j)+49;//dec to char
            str+=" (Amps)";
            plot[i*4+j].yAxis->setLabel(str);
            //ui->Plots->xAxis->setTickLabelType(QCPAxis::ltDateTime);
            //ui->Plots->xAxis->setDateTimeFormat("hh:mm:ss");
            plot[i*4+j].xAxis->setAutoTickStep(true);
            plot[i*4+j].axisRect()->setupFullAxesBox();
            //ui->tableWidget->horizontalHeader()->setSectionResizeMode(j, QHeaderView::Stretch);
            ui->tableWidget->horizontalHeader()->setResizeMode(j, QHeaderView::Stretch);
        }
        //ui->tableWidget->verticalHeader()->setSectionResizeMode(i*3+2, QHeaderView::Stretch);
        ui->tableWidget->verticalHeader()->setResizeMode(i*3+2, QHeaderView::Stretch);
    }
    ui->tableWidget->verticalHeader()->hide();
    ui->tableWidget->horizontalHeader()->hide();

//    //connect(ui->ControllingMean,SIGNAL(currentIndexChanged(int)),this,SLOT(setValue(int)));

//    ////////////////////////////////////////////////////////////////////////////////////
//    //////////////// MAIN VARIABLES
//    /// ////////////////////////////////////////////////////////////////////////////////
    timer=new QTimer();
    timer->setInterval(10);

    timer2=new QTimer();
    timer2->setInterval(25);

    Exp_time=new QElapsedTimer();
    Exp_time->start();
    Reference_time=new QElapsedTimer();
    //Reference_time->start();

    convertPOT=double ((10.24/32768.0)*2000.0);

    //outputFile=new QFile();
    //outputFile->setFileName("/home/nikosev/SMA_Control/build/experiment");
    //if(outputFile->open(QIODevice::WriteOnly | QIODevice::Text)){
    //    std::cout<<"File opened"<<std::endl;
    //}
    //else{
    //    std::cout<<"File not opened"<<std::endl;
    //}
    //if(outputFile->isOpen())
    //outputStream=new QTextStream(outputFile);

    //this->on_Reference_currentIndexChanged(2);

    ////////////////////////////////////////////////////////////////////////////////////
    //////////////// SIGNALS
    /// ////////////////////////////////////////////////////////////////////////////////
    //SIGNAL TRIES TO ACQUIRE DATA FROM STM32F4 BOARD
    connect(this->timer,SIGNAL(timeout()),this,SLOT(pltUpd()));
    connect(this->timer2,SIGNAL(timeout()),this,SLOT(GUIUpd()));
    //connect(ui->RotatePosition,SIGNAL(valueChanged(double)),this,SLOT(CTRLUIinit()));

    ////////////////////////////////////////////////////////////////////////////////////
    //////////////// PROGRAM START
    /// ////////////////////////////////////////////////////////////////////////////////
    //Controller initial values from window
    //CTRLUIinit();

    //OPEN PORT -- THE REST ARE HANDLED BY SIGNALS
    timer->start();
    timer2->start();

}

void MainWindow::SMAdisabled(){
    std::cout<< "SMAdisabled" <<std::endl;
    for (u_int8_t i=0;i<SMAcolumns;i++)
        if(!SMAcheckbox[i].isChecked())
            SMAslider[i].setValue(0);
}

void MainWindow::CTRLUIinit(){
    //check for disabled SMA
    //CTRL->init(SMAslider,ui->RotatePosition->value(),ui->controllMethod->currentText()
    //           ,ui->ControllingMean->currentText());
}

void MainWindow::on_stopButton_clicked()
{
    std::cout<< "Closing..." <<std::endl; /* writes PWM 0 and closes Main switch*/
    //ros::shutdown();
    //CTRL->closeAll();
    if(ui->mainSwitch->isChecked()){
        ui->mainSwitch->setChecked(false);
        //usleep(1);
    //CTRL->setMainSwitch(false);
    }
    this->~MainWindow();
}

void MainWindow::on_ManualControl_clicked(bool val)
{
    std::cout<< "SMAenabled" <<std::endl;
    ui->SMAview->setEnabled(val);
    ui->AutoUpd->setChecked(!val);
    for(u_int8_t i=0;i<SMAcolumns;i++) SMAslider[i].setValue(0);
    //CTRL->calculateControl(); //if inserted to manual control i must write the zeros
    if(val){
    for(u_int8_t i=0;i<SMAcolumns;i++){
        SMAslider[i].setEnabled(false);
        SMAspinbox[i].setEnabled(false);
        SMAcheckbox[i].setEnabled(val);
    }
    }
    ui->sendUpdate->setEnabled(val);
    //CTRL->uiStatus.sendUpdate=val;
}

void MainWindow::on_mainSwitch_clicked(bool val)
{
    std::cout<< "MainSwitchChanged" <<std::endl;
    pub_str->data.push_back(0x21);
    pub_str->data.push_back(0x04);
    pub_str->data.push_back(0x00);
    pub_str->data.push_back((unsigned char) val);
    publish=true;
    //gui_pub.publish(*pub_str);
    //pub_str->data.clear();
    //CTRL->setMainSwitch(val);
}

void MainWindow::on_Reference_currentIndexChanged(int){
    std::cout<< "Reference type Changed" <<std::endl;
    std::string str=ui->Reference->currentText().toStdString();
    if (str=="Fixed"){
        reference_type=0;
    }
    else if(str=="Square")
        reference_type=1;
    else if(str=="Sine")
        reference_type=2;
    else if(str=="SwitchingSquare")
        reference_type=3;
    else if(str=="MyExperiment1"){
        reference_type=4;
    }
    else reference_type=5;
    ui->RotatePosition->setValue(0.0);
    Reference_time->restart();
}

void MainWindow::pltUpd()
{
    //START ROS CHATTERING
    if(publish){
        ROS_INFO("MUST PUBLISH");
        //std::cout<< (int) pub_str->data[0] << " " << (int) pub_str->data[1] << " " << (int) pub_str->data[2] << " " << (int) pub_str->data[3] << " " <<std::endl;
        gui_pub.publish(*pub_str); 
    }
    ros::spinOnce();
    pub_str->data.clear();
    publish=false;
    int once=0;
    //while(CTRL->SerialRead())
    //   once++;
    if(!ui->ManualControl->isChecked()){
    //auto control -- must calculate reference
    switch(reference_type){
    //edw exw valei volt kanonika to reference prepei na einai thesi tha to allaksw meta ta prwta
    case 0:
        //Nothing to do here fixed by user
        break;
    case 1:{ //square pulse base 0
        double val=(SquareAmplitudeV/2)*(1+pow(-1.0,floor((Reference_time->elapsed()/SquarePeriod)+0.5)));
        if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
    }
        break;
    case 2:{ //sin pulse
        double val=SquareAmplitudeV*sin((2*M_PI*Reference_time->elapsed()/SinPeriod)+0.0);
        if(val!=ui->RotatePosition->value())ui->RotatePosition->setValue(val);
    }
        break;
    case 3:{
        double val=SquareAmplitudeV*pow(-1.0,floor((Reference_time->elapsed()/SquarePeriod)+0.5));
        if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
    }
        break;
    case 4:{
        double val;
        if (Reference_time->elapsed()<1000){
        val=SquareAmplitudeA;
        if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
        }
        else if (Reference_time->elapsed()<1700){
        val=SquareAmplitudeAlow;
        if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
        }
        else if (Reference_time->elapsed()<2700){
            val=-SquareAmplitudeA;
            val=SquareAmplitudeAlow;
            if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
            }
        else if (Reference_time->elapsed()<10400){
            val=-SquareAmplitudeAlow;
            val=SquareAmplitudeAlow;
            if(val!=ui->RotatePosition->value()) ui->RotatePosition->setValue(val);
        }
        else{
        Reference_time->restart();
        }
    }
    break;
    default:{
    //nothing write 0
    ui->RotatePosition->setValue(0.0);
    }
        break;
    }
    }
    if(once){
        /*
        if(outputFile->isOpen())
        (*outputStream)<<ui->RotatePosition->value()<<"\t"<<
                         Exp_time->elapsed()<<"\t"<<
                         Reference_time->elapsed()<<"\t"<<
                         CTRL->smaState.PWMval[3] <<"\t"<<
                         CTRL->smaState.PWMval[1] <<"\t"<<
                         CTRL->smaState.ADrms[0]<<"\t"<<
                         CTRL->smaState.ADrms[1]<<"\t"<<
                         CTRL->smaState.ADCV[6]<<"\t"<<
                         CTRL->smaState.ADCV[7]<<"\t"<<
                         CTRL->smaState.ADCV[14]<<"\t"<<
                         CTRL->smaState.AD[0]<<"\t"<<
                         CTRL->smaState.AD[1]<<"\t"<<
                                          CTRL->smaState.PWMval[2] <<"\t"<<
                                          CTRL->smaState.PWMval[3] <<"\t"<<
                                          CTRL->smaState.ADrms[2]<<"\t"<<
                                          CTRL->smaState.ADrms[3]<<"\t"<<
                                          CTRL->smaState.ADCV[11]<<"\t"<<
                                          CTRL->smaState.ADCV[12]<<"\t"<<
                                             CTRL->smaState.ADCV[13]<<"\t"<<
                                          CTRL->smaState.ADCV[14]<<"\t"<<
                                          CTRL->smaState.AD[2]<<"\t"<<
                                          CTRL->smaState.AD[3]<<"\t"<<
                         CTRL->smaState.ADraw<<"\t"<<
                         CTRL->smaState.Vbus<<endl;
                         */
        //std::cout<<"once " << once <<std::endl;
        if(ui->AutoUpd->isChecked()) this->on_sendUpdate_clicked();
    }
}

void MainWindow::on_sendUpdate_clicked(){
    std::cout<< "SMAdisabled" <<std::endl;
    //CTRL->calculateControl();
}

void MainWindow::GUIUpd()
{
    //Check if new Aruco image arrived in Tab2 and apply
    //if(recv){
       // ui->ArucoImage->setPixmap(QPixmap::fromImage(imdisplay)); //display the image in label
    //    cv::cvtColor(imag,imag,CV_BGR2RGB); //Color reconstruction from CV to QT
        //cv::imshow("view", imag);
        //cv::waitKey(1);
       // QImage imdisplay((uchar*) imag.data, imag.cols, imag.rows, imag.step, QImage::Format_RGB888); //CV->Qt
    //    recv=false;
    //}
    //Check Tab1
    //std::cout<< "GUIUpd" <<std::endl;
    static unsigned int tic;
    double x=smaState.time/1e6;
    for(u_int8_t i=0;i<8;i++){
        int chan=monitor_channel[i].currentIndex();
        switch(monitor_attribute[i].currentIndex()){
        // "PWM" << "Voltage" << "Current" << "CurrentRMS" << "Resistance" << "ResistanceRMS";
            case 0://PWM
                plot[i].graph(0)->addData(x,(double) smaState.PWMval[chan]);
            break;
            case 1://Voltage
                plot[i].graph(0)->addData(x,(double) (smaState.Vbus*(smaState.PWMval[chan]/4096.0)));
            break;
            case 2://Current
                plot[i].graph(0)->addData(x,(double) smaState.AD[chan]);
            break;
            case 3://CurrentRMS
                plot[i].graph(0)->addData(x,(double) smaState.ADrms[chan]);
            break;
            case 4://Resistance
                plot[i].graph(0)->addData(x,(double) (((smaState.PWMval[chan]/4096.0)*smaState.Vbus)/smaState.AD[chan]));
            break;
            case 5://ResistanceRMS
                plot[i].graph(0)->addData(x,(double) (((smaState.PWMval[chan]/4096.0)*smaState.Vbus)/smaState.ADrms[chan]));
            break;
            default:
            break;
        }
        plot[i].graph(0)->removeDataBefore(x-1);
        plot[i].xAxis->setRange(x, 1, Qt::AlignRight);
        plot[i].replot();
    }

    float Itot=0.001;
    for(int i=0;i<SMAcolumns*SMArows;i++) Itot+=smaState.AD[i];
    ui->smaStatus->setText(QString("Dt:%1us, Vbus:%2V, Itot: %3A, Vpot_max:%4V, Experiment elapsed time:%5s").
                                        arg(smaState.time-tic).
                                                   arg(smaState.Vbus, 0, 'f', 3).
                                                                        arg(Itot/1.0e3, 0, 'f', 3).
                                                                                    arg((double (smaState.ADCV[3]))*(10.24/32768.0), 0, 'f', 4).
                                                                                                                    arg((Exp_time->elapsed())/1.0e3,0,'f',3));
    
    tic=smaState.time;
}

void MainWindow::on_ControllingMean_currentIndexChanged(int val)
{
    std::cout<< "SMAdisabled" <<std::endl;
    for(u_int8_t i=0;i<SMAcolumns;i++)
        //sma spinbox value will change automatically
        SMAslider[i].setValue(0);
        if(ui->ManualControl->isChecked())
            //CTRL->calculateControl(); //if inserted to manual control i must write the above zeros

    switch (val){
    case 0: //case PWM
        for(u_int8_t row=0;row<2;row++){
        for(int col = 0; col < SMAcolumns/2; col++)
        {
            SMAslider[row*4+col].setMaximum(PWMlimit);
            SMAspinbox[row*4+col].setMaximum(PWMlimit);
        }
        }
        break;
    case 1: //case Volts
        for(u_int8_t row=0;row<2;row++){
        for(int col = 0; col < SMAcolumns/2; col++)
        {
            SMAslider[row*4+col].setMaximum(VoltageLimit);
            SMAspinbox[row*4+col].setMaximum(VoltageLimit);
        }
        }
        break;
    case 2: //case current Im
        for(u_int8_t row=0;row<2;row++){
        for(int col = 0; col < SMAcolumns/2; col++)
        {
            SMAslider[row*4+col].setMaximum(CurrentLimit);
            SMAspinbox[row*4+col].setMaximum(CurrentLimit);
        }
        }
        break;
    case 3: //case current RMS Irms
        for(u_int8_t row=0;row<2;row++){
        for(int col = 0; col < SMAcolumns/2; col++)
        {
            SMAslider[row*4+col].setMaximum(CurrentRMSLimit);
            SMAspinbox[row*4+col].setMaximum(CurrentRMSLimit);
        }
        }
        break;
    default:
        break;

    }
    //this->CTRLUIinit();
}

void MainWindow::on_controllMethod_currentIndexChanged(int val){
    std::cout<< "SMAdisabled" <<std::endl;
    //this->CTRLUIinit();
}

void MainWindow::onPlotVisualizationChannelChange(int val){
    for(uint8_t i=0;i<8;i++){
        if (sender()==&(monitor_channel[i])){
            std::cout << "I was called by channel "<< (int) i << std::endl;
            //delete _foobar;
            plot[i].graph(0)->clearData();
            plot[i].replot();
        }
    }
}

void MainWindow::onPlotVisualizationAttributeChange(int val){
    for(uint8_t i=0;i<8;i++){
        if (sender()==&(monitor_attribute[i])){
            std::cout << "I was called by attribute "<< (int) i << std::endl;
            plot[i].graph(0)->clearData();
            switch(val){
                    case 0://PWM
                        plot[i].yAxis->setRange(0,PWMlimit);
                    break;
                    case 1://Voltage
                        plot[i].yAxis->setRange(0,VoltageLimit);
                    break;
                    case 2://Current
                        plot[i].yAxis->setRange(0,CurrentLimit);
                    break;
                    case 3://CurrentRMS
                        plot[i].yAxis->setRange(0,CurrentRMSLimit);
                    break;
                    case 4://CurrentRMS
                        plot[i].yAxis->setRange(0,ResistanceLimit);
                    break;
                    case 5://CurrentRMS
                        plot[i].yAxis->setRange(0,ResistanceRMSLimit);
                    break;
                    default:
                        plot[i].yAxis->setRange(0,1);
                    break;
            }
            plot[i].replot();
        }
    }
}

MainWindow::~MainWindow()
{
    //std::cout<< "SMAdisabled" <<std::endl;
    delete ui;

    //close current (to add PWM)
    //CTRL->~controller();

    timer->stop();
    delete timer;
    timer2->stop();
    delete timer2;
    delete Exp_time;
    delete Reference_time;

    delete POTmodel;
    delete SMAmodel;

    for (u_int8_t i=0;i<8;i++) (&plot[i])->~QCustomPlot();
    delete plot;
    delete monitor_channel;
    delete monitor_attribute;

    //outputFile->close();
    //delete outputStream;
    //delete outputFile;
    //delete pub_str;
    delete m_sigmapper;

    delete SMAslider;
    delete SMAspinbox;
    delete SMAcheckbox;
    delete POTslider;
    //delete CTRL;

}
