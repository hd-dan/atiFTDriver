#include <iostream>

#include "ft_sensor.h"
#include "../commun/tcpcom.h"
#include "../commun/serialcom.h"

#include <fstream>
#include <unistd.h>

void readFt(){
    tcpcom plotServer("127.0.0.1",8888,0,1);

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml",
                 "~/Dan/atiFTDriver/config/drillConfig.xml");
    ft.setSensorQuat(std::vector<double>{1,0,0,0});

    std::chrono::high_resolution_clock::time_point t0=
            std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<30){
        std::vector<double> f= ft.get_ftRaw();
        std::vector<double> fsub= std::vector<double>(f.begin(),f.begin()+3);
//        std::vector<double> fma= ft.get_ftRawMA();
//        for (unsigned i=0;i<3;i++){
//            fsub.push_back(fma.at(i));
//        }
        print_vector("fsub",fsub);

        plotServer.sendData(fsub);
        t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
        usleep(1e3);

    }
}

void calFt(){

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml",
                 "~/Dan/atiFTDriver/config/drillConfig.xml");

    printf("Calibrate Ft Bias..\n");
    for (int i=0;i<10;i++){
        printf("Input mass:");
        double mi;
        std::cin >> mi;
        ft.bufForSensorCal(mi);
    }
    std::vector<std::vector<double> > mc= ft.calibrateSensorFromBuf();
    return;
}


void testFt(){
    std::string ftConfig= "../../atiFTDriver/config/ftConfig.xml";
    std::string drillConfig= "../../atiFTDriver/config/drillConfig.xml";

    ft_sensor ft;
    ft.setConfig(ftConfig,drillConfig);

    for (int i=0;i<100;i++){
        print_vector("ft",ft.get_ftRaw());
        usleep(1e4);
    }

    printf("end\n");
    return;
}

int main(){
    std::cout << "Hello World!" << std::endl;

    /// yellow: x-axis | blue: y-axis
//    readFt();
//    calFt();

    testFt();

    return 0;
}
