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
    drillConfig_path_= ft_sensor::processPath(drillConfig);
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
std::string ft_sensor::processPath(std::string path){
    if (path.at(0)=='~')
        path= getenv("HOME")+path.substr(1);
    if (path.find("..")==0){
//        std::string pwd= getenv("PWD");
        char tuh[PATH_MAX];
        std::string pwd=getcwd(tuh,sizeof(tuh));
        do{
            pwd= pwd.substr(0,pwd.rfind("/"));
            path= path.substr(path.find("..")+2);
        }while(path.find("..")<2);
        path= pwd+path;

    }else if(path.at(0)=='.')
        path= getenv("PWD")+path.substr(1);
    if (path.find("/")==path.npos)
        path= getenv("PWD")+std::string("/")+path;
    ft_sensor::processDirectory(path);
    return path;
}
void ft_sensor::processDirectory(std::string dir){
    dir= dir.substr(0,dir.rfind('/'));
    struct stat st;
    if (stat(dir.c_str(),&st)!=0){
        ft_sensor::processDirectory(dir);
        mkdir(dir.c_str(),S_IRWXU);
    }
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
    drillBufFt_= std::vector<std::vector<double> >(0,std::vector<double>(0,0));
    drillBufRT_= std::vector<std::vector<double> >(0,std::vector<double>(0,0));

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

    recorder_calDrill_.write(ft_temp);
    for(unsigned int i=0;i<R_temp.size();i++)
        recorder_calDrill_.write(R_temp.at(i));
    recorder_calDrill_.endLine();

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
    if (drillBufFt_.size()==0){
        drillBufFt_= std::vector<std::vector<double> >(2,std::vector<double>(0,0));
        recorder_calDrill_.openFile("~/Desktop/calDrillRecord.csv");
        recorder_calDrill_.header("h",std::vector<double>(6,0));
        recorder_calDrill_.header("R_row1",std::vector<double>(3,0));
        recorder_calDrill_.header("R_row2",std::vector<double>(3,0));
        recorder_calDrill_.header("R_row3",std::vector<double>(3,0));
        recorder_calDrill_.endLine();
    }
    std::chrono::high_resolution_clock::time_point t0= std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<calt){
        ft_sensor::bufForDrillCal();

        t= std::chrono::duration_cast<std::chrono::duration<double> >(
                    std::chrono::high_resolution_clock::now()-t0).count();
        usleep(1e4);
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
    ft_sensor::bufForSensorCal(massi,calt);
    return;
}

void ft_sensor::bufForSensorCal(double massi, double calt){
    if (sensorBuf_.size()!=13){
        sensorBuf_= std::vector<std::vector<double> >(13,std::vector<double>(0,0));

        recorder_calSensor_.openFile("~/Desktop/calRecord.csv");
        recorder_calSensor_.header("mass");
        recorder_calSensor_.header("h",std::vector<double>(6,0));
        recorder_calSensor_.header("R_row3",std::vector<double>(3,0));
        recorder_calSensor_.endLine();
    }
    ft_sensor::setSensorCalMode(true);

    int n=0;
    std::vector<double> h_avg(6,0);
//    std::vector<double> g= {0,0,9.81};
    double g= 9.81;

    std::unique_lock<std::mutex> ftLockSensorR(mtxSensorR_);
    std::vector<std::vector<double> > R_temp= sensorR_;
    ftLockSensorR.unlock();

    double t=0;
    std::chrono::high_resolution_clock::time_point t0= std::chrono::high_resolution_clock::now();
    while(t<calt){
        std::vector<double> h= ft_sensor::get_ftRaw();

        for (unsigned int i=0;i<h_avg.size();i++){
            h_avg.at(i)+=h.at(i);
        }
        n++;

        t= std::chrono::duration_cast<std::chrono::duration<double> >(
                    std::chrono::high_resolution_clock::now()-t0).count();
        usleep(1e5);
    }


    sensorBuf_.at(0).push_back( massi );
    for (unsigned int i=0;i<h_avg.size();i++){
        h_avg.at(i)*= (1./n);
        sensorBuf_.at(1+i).push_back( h_avg.at(i) );
    }
    std::vector<double> r3= R_temp.back();
    for (unsigned int i=0;i<r3.size();i++){
        sensorBuf_.at(7+i).push_back( r3.at(i) );
    }

    recorder_calSensor_.write(massi);
    recorder_calSensor_.write(h_avg);
    recorder_calSensor_.write(R_temp.back());
    recorder_calSensor_.endLine();

    return;
}

std::vector<std::vector<double> > ft_sensor::calibrateSensorFromBuf(){

    std::vector<std::vector<double> > mcBuf(12,std::vector<double>(0,0));
    for (unsigned int i=0;i<sensorBuf_.at(0).size()-1;i++){
        unsigned int k=sensorBuf_.size();
        std::vector<double> r3_i(3,0);
        for (unsigned int p=3;p>0;p--)
            r3_i.at(p-1)= sensorBuf_.at(--k).at(i);
        std::vector<double> h_i(6,0);
        for (unsigned int p=6;p>0;p--)
            h_i.at(p-1)= sensorBuf_.at(--k).at(i);
        double massi= sensorBuf_.at(--k).at(i);

        for (unsigned int j=i+1;j<sensorBuf_.at(0).size();j++){
            k= sensorBuf_.size();
            std::vector<double> r3_j(3,0);
            for (unsigned int p=3;p>0;p--)
                r3_j.at(p-1)= sensorBuf_.at(--k).at(j);
            std::vector<double> h_j(6,0);
            for (unsigned int p=6;p>0;p--)
                h_j.at(p-1)= sensorBuf_.at(--k).at(j);
            double massj= sensorBuf_.at(--k).at(j);

            if (l2norm(r3_j-r3_i)>1e-3)
                continue;

            std::vector<double> hDiff= h_j-h_i;
            std::vector<double> wDiff= 9.81*(r3_j*massj-r3_i*massi);
            std::vector<double> tauDiff= cross(com_,wDiff);
            wDiff.insert(wDiff.end(),tauDiff.begin(),tauDiff.end());
            for(unsigned ax=0;ax<6;ax++){
                if (fabs(r3_i.at(ax%3))<1e-2){
                    mcBuf.at(ax+6).push_back(-0.5*(h_j.at(ax)+h_i.at(ax))); //bias
                }else if (fabs(hDiff.at(ax))>1e-3){
                    mcBuf.at(ax).push_back( wDiff.at(ax)/hDiff.at(ax) ); //slope
                }
            }
        }
    }

    std::vector<double> mc_avg= mean_cancelOutliner(mcBuf);

    std::vector<double> m_est(mc_avg.begin(),mc_avg.begin()+6);
    std::vector<double> c_est(mc_avg.begin()+6,mc_avg.end());
    sensorMc_= {m_est,c_est};

    ft_sensor::setSensorCalMode(false);
    print_matrix("mc",sensorMc_);
    sensorBuf_.clear();
    return sensorMc_;
}

void ft_sensor::save_sensorCalibration(){
    ftDriver_.saveCalibration(sensorMc_);
    return;
}
