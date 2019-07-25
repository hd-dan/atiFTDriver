#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include "../util/util.hpp"
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
    void initRcv();

    std::vector<double> attachW_; //w_= 0w
    std::vector<double> attachP_; //p_= tipP_com
    std::vector<std::vector<double> > attachSp_;
    std::vector<double> read_drillConfig();

    std::vector<std::vector<double> > I_;
    std::vector<std::vector<double> > sensorR_;
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

    unsigned int caliN_;
    std::vector<double> caliBufferF_;
    std::vector<double> caliBufferTau_;
    std::vector<std::vector<double> > caliBufferRT_;

    std::vector<std::vector<double> > calBufBias_;
    int biasN1_,biasN2_;
    int calBiasEp_;

public:
    ft_sensor(std::string ftConfig);
    ft_sensor(std::string ftConfig, std::string drillConfig);
    ~ft_sensor();

    void setSensorR(std::vector<std::vector<double> > R);
    void setSensorQuat(std::vector<double> quat);
    std::vector<std::vector<double> > getSensorR();
    void setMAWindow(int sample);

    void calDrillBuf();
    void calibrateDrillEpisode(std::vector<std::vector<double> >R, double calt=5);
    void calibrateDrillEpisode(std::vector<double> quat, double calt=5);
    std::vector<double> calibrateDrillFromBuf(bool save=1);
//    std::vector<double> calibrateDrill(double calt=3);
    void save_drillCalibration();
    void set_drillConfigPath(std::string path);

    std::vector<double> get_ftTip();
    std::vector<double> get_ftRaw();
    std::vector<double> get_ftTipMA();
    std::vector<double> get_ftRawMA();

    void setCalFtBiasMode(bool calMode);
    void calBiasBuf(bool oppDir=0);
    bool checkBiasBuffNEq();
    void calBiasEpisode(double calt=5);
    std::vector<double> calBiasFromBuf();
    std::vector<double> calibrateFtBias(double cali_t=5);
    void set_ftBias(std::vector<double> bias);
};

#endif // FT_SENSOR_H
