#include "ft_sensor.h"

ft_sensor::ft_sensor():fStop_(true){
}

ft_sensor::ft_sensor(std::string ftConfig_path):ftDriver_(ftConfig_path),
                                        fStop_(0){
    attachW_= std::vector<double>(3,0);
    attachP_= std::vector<double>(3,0);
    attachSp_= crossMat(attachP_);

    ft_sensor::initRcv();
    return;
}

ft_sensor::ft_sensor(std::string ftConfig_path, std::string drillConfig_path):
                                        ftDriver_(ftConfig_path),
                                        drillConfig_path_(drillConfig_path),
                                        fStop_(0){
    if (drillConfig_path_.at(0)=='~')
        drillConfig_path_= getenv("HOME")+drillConfig_path_.substr(1);
    fConfigExist_= std::ifstream(drillConfig_path_).good();
    if (fConfigExist_){
        ft_sensor::read_drillConfig();
    }else{
        printf("Failed to open payload config file: %s\n",drillConfig_path_.c_str());
        attachW_= std::vector<double>(3,0);
        attachP_= std::vector<double>(3,0);
        attachSp_= crossMat(attachP_);
    }

    ft_sensor::initRcv();
    return;
}

void ft_sensor::setConfig(std::string ftConfig, std::string drillConfig){
    if (!fStop_)
        return;
    ftDriver_.setConfig(ftConfig);
    drillConfig_path_= drillConfig;
    if (drillConfig_path_.at(0)=='~')
        drillConfig_path_= getenv("HOME")+drillConfig_path_.substr(1);
    fConfigExist_= std::ifstream(drillConfig_path_).good();
    if (fConfigExist_){
        ft_sensor::read_drillConfig();
    }else{
        printf("Failed to open payload config file: %s\n",drillConfig_path_.c_str());
        attachW_= std::vector<double>(3,0);
        attachP_= std::vector<double>(3,0);
        attachSp_= crossMat(attachP_);
    }

    ft_sensor::initRcv();
    return;
}

void ft_sensor::initRcv(){
    sensorR_= eye<double>(3,3);

    ftRawBuf_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
    ftTipBuf_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
    ftRawMA_= std::vector<double>(6,0);
    ftTipMA_= std::vector<double>(6,0);
    windowMA_= 300;

    g_= {0,0,9.81};
    com_= ftDriver_.getCom();

    drillBufN_=0;
    drillBufFt_= std::vector<std::vector<double> >(2,std::vector<double>(0,0));
    drillBufRT_= std::vector<std::vector<double> >(0,std::vector<double>(0,0));

//    calBufBias_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
//    biasN1_=0;
//    biasN2_=0;
//    calBiasEp_=0;

    tReadFt_= boost::thread(&ft_sensor::loopFt,this);
    while(ftRaw_.size()==0){
        usleep(1e3);
    }
    return;
}

ft_sensor::~ft_sensor(){
    fStop_=1;
    tReadFt_.interrupt();
    tReadFt_.join();
    return;
}

void ft_sensor::setSensorR(std::vector<std::vector<double> > R){
    sensorR_= R;
    return;
}

void ft_sensor::setSensorQuat(std::vector<double> quat){
    return ft_sensor::setSensorR(quat2rotm(quat));
}

std::vector<std::vector<double> > ft_sensor::getSensorR(){
    std::unique_lock<std::mutex> ftLockSensorR(mtxSensorR_);
    return sensorR_;
}

void ft_sensor::setMAWindow(int sample){
    windowMA_= unsigned(sample);
    return;
}

void ft_sensor::loopFt(){
    fStop_=0;
    while(!fStop_){
        /// Get ftRaw and Remove drill weight on ft Reading
        std::vector<std::vector<double> > R= sensorR_;
        std::vector<double> tipW= R*attachW_;
        std::vector<double> cali= appendVect(tipW, attachSp_*tipW);

        std::unique_lock<std::mutex> ftLockRaw(mtxFtRaw_);
        ftRaw_= ftDriver_.getData();
        ftTip_= ftRaw_-cali;
        ftLockRaw.unlock();


        /// Process and Compute Moving Average
        std::vector<double> ftRawMA(ftRawMA_.size(),0);
        std::vector<double> ftTipMA(ftTipMA_.size(),0);
        for (unsigned int i=0;i<ftRaw_.size();i++){
            ftRawBuf_.at(i).push_back(ftRaw_.at(i));
            ftTipBuf_.at(i).push_back(ftTip_.at(i));
            if (ftRawBuf_.at(i).size()>windowMA_){
                auto it= ftRawBuf_.at(i).end();
                ftRawBuf_.at(i)= std::vector<double>(it-windowMA_,it);
            }
            if (ftTipBuf_.at(i).size()>windowMA_){
                auto it= ftTipBuf_.at(i).end();
                ftTipBuf_.at(i)= std::vector<double>(it-windowMA_,it);
            }

            double sumRaw=0;
            for(auto j:ftRawBuf_.at(i))
                sumRaw+=j;
            ftRawMA.at(i)= sumRaw/ftRawBuf_.at(i).size();

            double sumTip=0;
            for(auto j:ftTipBuf_.at(i))
                sumTip+=j;
            ftTipMA.at(i)= sumTip/ftRawBuf_.at(i).size();
        }

        std::unique_lock<std::mutex> ftLockMA(mtxFtMA_);
        ftRawMA_= ftRawMA;
        ftTipMA_= ftTipMA;
        ftLockMA.unlock();

        boost::this_thread::interruption_point();
        usleep(1e2);
    }
}

void ft_sensor::bufForDrillCal(){
    std::vector<double> ft_temp= ft_sensor::get_ftRaw();

    std::unique_lock<std::mutex> ftLockSensorR(mtxSensorR_);
    std::vector<std::vector<double> > R_temp= sensorR_;
    ftLockSensorR.unlock();

    for (unsigned int i=0;i<6;i++){
        if (i<3){
            drillBufFt_.at(0).push_back(ft_temp.at(i));
            std::vector<double> RTi;
            for (unsigned int j=0;j<R_temp.size();j++)
                RTi.push_back(R_temp.at(j).at(i));
            drillBufRT_.push_back(RTi);
        }else
            drillBufFt_.at(1).push_back(ft_temp.at(i));
    }
    drillBufN_++;
    return;
}

void ft_sensor::episodeBufForDrillCal(std::vector<std::vector<double> > R, double calt){
    ft_sensor::setSensorR(R);
    ft_sensor::episodeBufForDrillCal(calt);
    return;
}

void ft_sensor::episodeBufForDrillCal(double calt){
    if (drillBufFt_.size()==0) drillBufFt_= std::vector<std::vector<double> >(2,std::vector<double>(0,0));
    std::chrono::high_resolution_clock::time_point t0= std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<calt){
        ft_sensor::bufForDrillCal();

        t= std::chrono::duration_cast<std::chrono::duration<double> >(
                    std::chrono::high_resolution_clock::now()-t0).count();
        usleep(1e3);
    }
    return;
}

std::vector<double> ft_sensor::calibrateDrillFromBuf(bool save){
////    tipF= 0R_tipT*0W
    attachW_= lsqQR(drillBufRT_,drillBufFt_.at(0));
//    printf("werr: %.3f\n",norm(caliBufferF_-caliBufferRT_*attachW_));

////    tipTau= tipP x tipW
    std::vector<double> tipW= drillBufRT_*attachW_*-1;
    std::vector<std::vector<double> > Sw;
    for (unsigned int i=0;i<drillBufN_;i++){
        std::vector<double> tipWi(tipW.begin()+3*i,tipW.begin()+3*i+3);

        std::vector<std::vector<double> > Swi= crossMat(tipWi);
        for (unsigned int j=0;j<Swi.size();j++){
            Sw.push_back(Swi.at(j));
        }
    }
    attachP_= lsqQR(Sw,drillBufFt_.at(1));
    if (save) ft_sensor::save_drillCalibration();

    drillBufFt_.clear();
    drillBufRT_.clear();
    drillBufN_=0;
    return appendVect(attachW_,attachP_);
}


std::vector<double> ft_sensor::read_drillConfig(){
    printf("Reading Drill config file.\n");
    drillConfigFile_.readFile(drillConfig_path_);
    attachW_= drillConfigFile_.getXmlVect<double>("drill.weight");
    attachP_= drillConfigFile_.getXmlVect<double>("drill.position");
    attachSp_= crossMat(attachP_);

    printf("Drill weight:[");
    for (unsigned int i=0;i<attachW_.size();i++){
        printf("%.3f, ",attachW_.at(i));
    }printf("]\n\n");

    return appendVect(attachW_,attachP_);
}

void ft_sensor::save_drillCalibration(){
    boost::property_tree::ptree pt;
    pt.put("drill.weight.x",attachW_.at(0));
    pt.put("drill.weight.y",attachW_.at(1));
    pt.put("drill.weight.z",attachW_.at(2));

    pt.put("drill.position.x",attachP_.at(0));
    pt.put("drill.position.y",attachP_.at(1));
    pt.put("drill.position.z",attachP_.at(2));

//    drillConfigFile_.writeFile(drillConfig_path_,pt);
    boost::property_tree::xml_writer_settings<std::string> settings(' ', 2,"ASCII");
    boost::property_tree::write_xml(drillConfig_path_,pt,std::locale(),settings);
    return;
}

void ft_sensor::set_drillConfigPath(std::string path){
    drillConfig_path_= path;
    return;
}


std::vector<double> ft_sensor::get_ftRaw(){
    std::unique_lock<std::mutex> ftLockRaw(mtxFtRaw_);
    return ftRaw_;
}

std::vector<double> ft_sensor::get_ftTip(){
    std::unique_lock<std::mutex> ftLockRaw(mtxFtRaw_);
    return ftTip_;
}

std::vector<double> ft_sensor::get_ftRawMA(){
    std::unique_lock<std::mutex> ftLockMA(mtxFtMA_);
    return ftRawMA_;
}

std::vector<double> ft_sensor::get_ftTipMA(){
    std::unique_lock<std::mutex> ftLockMA(mtxFtMA_);
    return ftTipMA_;
}


void ft_sensor::setSensorCalMode(bool fCalMode){
    return ftDriver_.setCalibrateMode(fCalMode);
}

void ft_sensor::bufForSensorCal(double massi, std::vector<std::vector<double> > R, double calt){
    ft_sensor::setSensorR(R);
    ft_sensor::bufForSensorCal(calt);
    return;
}

void ft_sensor::bufForSensorCal(double massi, double calt){
    ft_sensor::setSensorCalMode(true);

    std::chrono::high_resolution_clock::time_point t0= std::chrono::high_resolution_clock::now();
    double t=0;
    while(t<calt){
        std::vector<double> h= ft_sensor::get_ftRaw();

        std::unique_lock<std::mutex> ftLockSensorR(mtxSensorR_);
        std::vector<std::vector<double> > R_temp= sensorR_;
        ftLockSensorR.unlock();

        std::vector<double> h_x= R_temp*g;
        std::vector<double> tau_x= cross(com_,h_x);
        h_x.insert(h_x.end(),tau_x.begin(),tau_x.end());
        std::vector<double> h_massi= massi*h_x;

        sensorBuf_.at(0).insert(sensorBuf_.at(0).end(),h_massi.begin(),h_massi.end());
        sensorBuf_.at(1).insert(sensorBuf_.at(1).end(),h_x.begin(),h_x.end());
        sensorBuf_.at(2).insert(sensorBuf_.at(2).end(),h.begin(),h.end());

        t= std::chrono::duration_cast<std::chrono::duration<double> >(
                    std::chrono::high_resolution_clock::now()-t0).count();
        usleep(1e3);
    }
    return;
}

std::vector<std::vector<double> > ft_sensor::calibrateSensorFromBuf(){
    unsigned int j=0;
    unsigned int n=6;
    std::vector<std::vector<double> > A(sensorBuf_.at(0).size(),std::vector<double>(2*n+j,0));
    for (unsigned int i=0;i<sensorBuf_.at(0).size();i++){
        A.at(i).at(i%n)= sensorBuf_.at(0).at(i);
        A.at(i).at(i%n+n)= 1;
        if (j==1)
            A.at(i).back()= sensorBuf_.at(1).at(i);
    }

    std::vector<double> x= lsqQR(A,sensorBuf_.at(2));
//    printf("err: %.4f\n",l2norm(sensorBuf_.at(2)-A*x));

    std::vector<double> m_est(x.begin(),x.begin()+n);
    std::vector<double> c_est(x.begin()+n,x.end()-j);
//    m0= x.back();
    for (unsigned int i=0;i<6;i++){
        m_est.at(i)= 1.0/m_est.at(i);
        c_est.at(i)*= m_est.at(i)*-1;
    }
    sensorMc_= {m_est,c_est};
    print_matrix("sensorMc",sensorMc_);
    ft_sensor::setSensorCalMode(false);
    sensorBuf_.clear();
    return sensorMc_;
}

void ft_sensor::save_sensorCalibration(){
    ftDriver_.saveCalibration(sensorMc_);
    return;
}

//void ft_sensor::setCalFtBiasMode(bool calMode){
//    return ftDriver_.setCalibrateMode(calMode);
//}

//void ft_sensor::calBiasBuf(bool oppDir){
//    std::vector<double> ft_temp= ft_sensor::get_ftRaw();
//    for (unsigned int i=0;i<6;i++){
//        calBufBias_.at(i).push_back(ft_temp.at(i));
//    }
//    if (!oppDir) biasN1_++;
//    else biasN2_++;
//    return;
//}

//std::vector<double> ft_sensor::calBiasFromBuf(){
//    std::vector<double> ftAvg= mean(calBufBias_,0)*-1;
//    ft_sensor::set_ftBias(ftAvg);
//    ftDriver_.setCalibrateMode(false);

//    for(unsigned int i=0;i<ftAvg.size();i++){
//        if (i<3) ftAvg.at(i)*=ftDriver_.getScale().at(0);
//        else ftAvg.at(i)*=ftDriver_.getScale().at(1);
//    }
//    printf("calibrated bias:[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
//           ftAvg.at(0),ftAvg.at(1),ftAvg.at(2),ftAvg.at(3),ftAvg.at(4),ftAvg.at(5));

//    calBufBias_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
//    biasN1_=0;
//    biasN2_=0;
//    calBiasEp_=0;
//    return ftAvg;
//}

//bool ft_sensor::checkBiasBuffNEq(){
//    if (biasN1_!=biasN2_) return 0;
//    else if (biasN1_==0) return 0;
//    else return 1;
//}

//void ft_sensor::calBiasEpisode(double calt){
//    ftDriver_.setCalibrateMode(true);

//    std::chrono::high_resolution_clock::time_point t0
//            = std::chrono::high_resolution_clock::now();
//    double t=0;
//    while ( (!calBiasEp_&&t<calt) || (calBiasEp_&&!ft_sensor::checkBiasBuffNEq()) ){
//        t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
//        ft_sensor::calBiasBuf(calBiasEp_);
//        usleep(1e3);
//    }
//    calBiasEp_++; calBiasEp_=calBiasEp_%2;
//    return;
//}

//std::vector<double> ft_sensor::calibrateFtBias(double calt){
//    ftDriver_.setCalibrateMode(true);

//    printf("Calibrating Ft Bias..\n");
//    for (int i=0;i<2;i++){
//        ft_sensor::calBiasEpisode(calt);
//        if (!i){
//            printf("Turn the sensor to opposite direction & Press Enter:");
//            std::string a; std::getline(std::cin,a);
//        }
//    }
//    std::vector<double> bias= ft_sensor::calBiasFromBuf();
//    return bias;
//}

//void ft_sensor::set_ftBias(std::vector<double> bias){
//    return ftDriver_.saveBiasToConfig(bias);
//}
