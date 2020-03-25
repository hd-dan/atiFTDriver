#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <math.h>
#include "../util/util.hpp"
#include "../dataFile/data_file.h"
#include "../dataFile/xml_file.h"
#include "netft_rdt_driver.h"

#include <boost/thread.hpp>
#include <mutex>
#include <chrono>

class ft_sensor{
private:
    netft_rdt_driver::NetFTRDTDriver ftDriver_;

    bool fConfigExist_;
    std::string drillConfig_path_;
    xml_file drillConfigFile_;
    std::string processPath(std::string path);
    void processDirectory(std::string dir);
    void initRcv();

    data_file recorder_calSensor_;
    data_file recorder_calDrill_;

    std::vector<double> attachW_; //w_= 0w
    std::vector<double> attachP_; //p_= tipP_com
    std::vector<std::vector<double> > attachSp_;
    std::vector<double> read_drillConfig();

    std::vector<std::vector<double> > I_;
    std::vector<std::vector<double> > sensorR_;
    std::mutex mtxSensorR_;

    bool fStop_;
    void loopFt();
    boost::thread tReadFt_;
    std::mutex mtxFtRaw_;
    std::mutex mtxFtMA_;

    std::vector<double> ftRaw_;
    std::vector<double> ftTip_; //tipF

    unsigned int windowMA_;
    std::vector<std::vector<double> > ftRawBuf_;
    std::vector<std::vector<double> > ftTipBuf_;
    std::vector<double> ftTipMA_;
    std::vector<double> ftRawMA_;

    unsigned int drillBufN_;
//    std::vector<double> drillBufF_;
//    std::vector<double> drillBufTau_;
    std::vector<std::vector<double> >drillBufFt_;
    std::vector<std::vector<double> > drillBufRT_;

    std::vector<double> g_;
    std::vector<double> com_;
    std::vector<std::vector<double> > sensorBuf_;
    std::vector<std::vector<double> > sensorMc_;

//    std::vector<std::vector<double> > calBufBias_;
//    int biasN1_,biasN2_;
//    int calBiasEp_;

public:
    ft_sensor();
    ft_sensor(std::string ftConfig);
    ft_sensor(std::string ftConfig, std::string drillConfig);
    ~ft_sensor();
    void setConfig(std::string ftConfig, std::string drillConfig);

    void setSensorR(std::vector<std::vector<double> > R);
    void setSensorQuat(std::vector<double> quat);
    std::vector<std::vector<double> > getSensorR();
    void setMAWindow(int sample);

    void set_drillConfigPath(std::string path);
    void bufForDrillCal();
    void episodeBufForDrillCal(std::vector<std::vector<double> >R, double calt=5);
    void episodeBufForDrillCal(double calt=5);
    std::vector<double> calibrateDrillFromBuf(bool save=1);
    void save_drillCalibration();

    std::vector<double> get_ftTip();
    std::vector<double> get_ftRaw();
    std::vector<double> get_ftTipMA();
    std::vector<double> get_ftRawMA();


    void setSensorCalMode(bool fcalMode);
    void bufForSensorCal(double mi,std::vector<std::vector<double> > R, double calt=5);
    void bufForSensorCal(double mi, double calt=5);
    std::vector<std::vector<double> > calibrateSensorFromBuf();
    void save_sensorCalibration();


//    void setCalFtBiasMode(bool calMode);
//    void calBiasBuf(bool oppDir=0);
//    bool checkBiasBuffNEq();
//    void calBiasEpisode(double calt=5);
//    std::vector<double> calBiasFromBuf();
//    std::vector<double> calibrateFtBias(double cali_t=5);
//    void set_ftBias(std::vector<double> bias);
};

#endif // FT_SENSOR_H
