#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include "../util/util.hpp"
#include "../dataFile/xml_file.h"
#include "netft_rdt_driver.h"

#include <boost/thread.hpp>
#include <mutex>

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
    std::vector<std::vector<double> > *sensorR_;
    bool fStop_;
    void loopFt();
    boost::thread tReadFt_;
    std::mutex mtxFt_;
    std::vector<double> ftRaw_;
    std::vector<double> ftTip_; //tipF

    unsigned int caliN_;
    std::vector<double> caliBufferF_;
    std::vector<double> caliBufferTau_;
    std::vector<std::vector<double> > caliBufferRT_;


public:
    ft_sensor(std::string ftConfig);
    ft_sensor(std::string ftConfig, std::string drillConfig);
    ~ft_sensor();

    void setSensorR(std::vector<std::vector<double> > &R);

//    std::vector<double> calibrateDrill(std::vector<double> ft, std::vector<std::vector<double> > Rs);

    void calDrillBuf();
    std::vector<double> calibrateDrillFromBuf();
    std::vector<double> calibrateDrill(double calt=10);
    void save_drillCalibration();
    void set_drillConfigPath(std::string path);

    std::vector<double> get_ftTip();
    std::vector<double> get_ftRaw();

    void setCalFtBiasMode(bool calMode);
    std::vector<std::vector<double> > calibrateFtBias(double cali_t=5);
    void set_ftBias(std::vector<double> bias);
};

#endif // FT_SENSOR_H
