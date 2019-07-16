#include "ft_sensor.h"

ft_sensor::ft_sensor(std::string ftConfig_path):ftDriver_(ftConfig_path),
                                        fStop_(0),caliN_(0){
    attachW_= std::vector<double>(3,0);
    attachP_= std::vector<double>(3,0);
    attachSp_= crossMat(attachP_);

    ft_sensor::initRcv();
    return;
}

ft_sensor::ft_sensor(std::string ftConfig_path, std::string drillConfig_path):
                                        ftDriver_(ftConfig_path),
                                        drillConfig_path_(drillConfig_path),
                                        fStop_(0),caliN_(0){
    fConfigExist_= std::ifstream(drillConfig_path_).good();
    if (fConfigExist_)
        ft_sensor::read_drillConfig();
    else{
        attachW_= std::vector<double>(3,0);
        attachP_= std::vector<double>(3,0);
        attachSp_= crossMat(attachP_);
    }

    ft_sensor::initRcv();
    return;
}

void ft_sensor::initRcv(){
    I_= eye<double>(3,3);
    sensorR_= I_;

    ftRawBuf_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
    ftTipBuf_= std::vector<std::vector<double> >(6,std::vector<double>(0,0));
    ftRawMA_= std::vector<double>(6,0);
    ftTipMA_= std::vector<double>(6,0);
    windowMA_= 300;

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

std::vector<std::vector<double> > ft_sensor::getSensorR(){
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

//std::vector<double> ft_sensor::calibrateDrill(std::vector<double> ft, std::vector<std::vector<double> > Rs){
//    assert(ft.size()/2==Rs.size() && "ft size and R size not match");
//    int n= int(ft.size()/6);

////    tipF= 0R_tipT*0W
//    std::vector<std::vector<double> > RT;
//    std::vector<double> f;
//    for (unsigned int i=0;i<unsigned(n);i++){
//        std::vector<std::vector<double> > Ri= extractMat(Rs,int(3*i),int(3*i+2),0,-1);
//        RT= appendMat(RT, transpose(Ri));
//        f= appendVect(f,extractVect(ft,int(6*i),int(6*i+2)) );
//    }
//    std::vector<double> w= lsqQR(RT,f);


////    tipTau= tipP x tipW
//    std::vector<std::vector<double> > Sw;
//    std::vector<double> tau;
//    for (unsigned int i=0;i<unsigned(n);i++){
//        std::vector<std::vector<double> > RTi= extractMat(RT,int(3*i),int(3*i+2),0,-1);
//        std::vector<double> tipW= RTi*w*-1;
//        Sw= appendMat(Sw,crossMat(tipW));
//        tau= appendVect(tau,extractVect(ft, int(6*i+3), int(6*i+5)) );
//    }
//    std::vector<double> p= lsqQR(Sw,tau);

//    attachW_= w;
//    attachP_= p;
//    return appendVect(w,p);
//}

void ft_sensor::calDrillBuf(){
    std::vector<double> ft_temp= ft_sensor::get_ftRaw();
    std::vector<std::vector<double> > R_temp= sensorR_;

    for (unsigned int i=0;i<6;i++){
        if (i<3){
            caliBufferF_.push_back(ft_temp.at(i));
            std::vector<double> RTi;
            for (unsigned int j=0;j<R_temp.size();j++){
                RTi.push_back(R_temp.at(j).at(i));
            }
            caliBufferRT_.push_back(RTi);
        }else
            caliBufferTau_.push_back(ft_temp.at(i));
    }
    caliN_++;
    return;
}

std::vector<double> ft_sensor::calibrateDrillFromBuf(){
////    tipF= 0R_tipT*0W
    attachW_= lsqQR(caliBufferRT_,caliBufferF_);

////    tipTau= tipP x tipW
    std::vector<double> tipW= caliBufferRT_*attachW_*-1;
    std::vector<std::vector<double> > Sw;
    for (unsigned int i=0;i<caliN_;i++){
        std::vector<double> tipWi(tipW.begin()+3*i,tipW.begin()+3*i+3);

        std::vector<std::vector<double> > Swi= crossMat(tipWi);
        for (unsigned int j=0;j<Swi.size();j++){
            Sw.push_back(Swi.at(j));
        }
    }
    attachP_= lsqQR(Sw,caliBufferTau_);

    caliBufferF_.clear();
    caliBufferTau_.clear();
    caliBufferRT_.clear();
    caliN_=0;
    return appendVect(attachW_,attachP_);
}

std::vector<double> ft_sensor::calibrateDrill(double calt){
    printf("Calibrating..\n");

    for (int i=0;i<10;i++){
        printf("Position %d..\n",i);

        std::chrono::high_resolution_clock::time_point t0=
                std::chrono::high_resolution_clock::now();
        double t=0;
        while (t<calt){
            ft_sensor::calDrillBuf();

            t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
            usleep(1e3);
        }

        printf("Press Enter to Continue Or 'q' to Finish):");
        std::string a;
        std::getline(std::cin,a);
        if (!a.compare("q"))
            break;
    }

    return ft_sensor::calibrateDrillFromBuf();
}

std::vector<double> ft_sensor::read_drillConfig(){
    drillConfigFile_.readFile(drillConfig_path_);
    attachW_= drillConfigFile_.getXmlVect<double>("drill.weight");
    attachP_= drillConfigFile_.getXmlVect<double>("drill.position");
    attachSp_= crossMat(attachP_);

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

    drillConfigFile_.writeFile(drillConfig_path_,pt);
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



void ft_sensor::setCalFtBiasMode(bool calMode){
    return ftDriver_.setCalibrateMode(calMode);
}

std::vector<std::vector<double> > ft_sensor::calibrateFtBias(double cali_t){
    ftDriver_.setCalibrateMode(true);

    std::vector<std::vector<double> > ftTBuf(6,std::vector<double>(0,0));
    std::chrono::high_resolution_clock::time_point t0;
    t0= std::chrono::high_resolution_clock::now();
    double t=0;
    printf("Calibrating Ft Bias..\n");
    while (t<cali_t){
        t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());

        std::vector<double> ft_temp= ft_sensor::get_ftRaw();

        for (unsigned int i=0;i<6;i++){
            ftTBuf.at(i).push_back(ft_temp.at(i));
        }
        usleep(1e3);
    }
    std::vector<double> ftAvg= mean(ftTBuf,0)*-1;
    printf("avg fx: %.3f | fy: %.3f | fz: %.3f | tx: %.3f | ty: %.3f | tz: %.3f\n",
           ftAvg.at(0),ftAvg.at(1),ftAvg.at(2), ftAvg.at(3),ftAvg.at(4),ftAvg.at(5));

    ftDriver_.setCalibrateMode(false);
    ft_sensor::set_ftBias(ftAvg);
    return ftTBuf;
}

void ft_sensor::set_ftBias(std::vector<double> bias){
    return ftDriver_.saveBiasToConfig(bias);
}
