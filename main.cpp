#include <iostream>

#include "ft_sensor.h"
#include "../commun/commun.h"
#include "../commun/serialcom.h"

#include <fstream>
#include <unistd.h>

void readFt(){
    commun plotServer("127.0.0.1",8888,0,1);

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

    printf("Calibrating Ft Bias..\n");
    for (int i=0;i<2;i++){
        ft.calBiasEpisode(10);
        if (!i){
            printf("Turn the sensor to opposite direction & Press Enter:");
            std::string a; std::getline(std::cin,a);
        }
    }
    std::vector<double> bias= ft.calBiasFromBuf();
    return;
}

void calAttach(){
    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml",
                 "~/Dan/atiFTDriver/config/drillConfig.xml");

    std::vector<double> quat_down= {1,0,0,0};
    std::vector<double> quat_up= {0,0,0,1};

    double calt=3;
    printf("Calibrating Payload..\n");
    for (int i=0;i<10;i++){
        printf("Position %d..\n",i);

        if (i) ft.calibrateDrillEpisode(quat_down,calt);
        else ft.calibrateDrillEpisode(quat_up,calt);

        printf("Press Enter to Continue Or 'q' to Finish:");
        std::string a;
        std::getline(std::cin,a);
        if (!a.compare("q"))
            break;
    }

    std::vector<double> attachWP= ft.calibrateDrillFromBuf(true);
    print_vector("attachWP",attachWP);
    return;
}

void calPressureSensor(){
    serialCom arduino("/dev/ttyACM0");
    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml");

    std::vector<double> factorVect(0,0);

    std::chrono::high_resolution_clock::time_point t0=
            std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<30){
        std::vector<double> f= ft.get_ftRawMA();
        std::vector<double> fsub= std::vector<double>(f.begin(),f.begin()+3);

        std::vector<double> pressureRead= arduino.getData();
        std::vector<double> result(3,0);
        result.at(0)= f.at(2);
        result.at(1)= pressureRead.at(0);
        result.at(2)= pressureRead.at(1);

        double factor= result.at(0)/result.at(1);
        if (result.at(1)<1e-7)
            factorVect.push_back(factor);

        printf("ft: %.5f | Pressure1: %.5f | Pressure2: %.5f | factor: %.5f\n",
               result.at(0),result.at(1),result.at(2),factor);

        t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
        usleep(1e3);
    }


    return;
}


int main(){
    std::cout << "Hello World!" << std::endl;

    /// yellow: x-axis | blue: y-axis
//    calPressureSensor();
    readFt();
//    calFt();
//    calAttach();

    return 0;
}
