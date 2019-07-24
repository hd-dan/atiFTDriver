#include <iostream>

#include "ft_sensor.h"
#include "../commun/commun.h"
#include "../commun/serialcom.h"

#include <fstream>
#include <unistd.h>

void readFt(){
    commun plotServer("127.0.0.1",8888,0,1);

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml");

    std::chrono::high_resolution_clock::time_point t0=
            std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<100){
        std::vector<double> f= ft.get_ftRaw();
        std::vector<double> fma= ft.get_ftRawMA();
        std::vector<double> fsub= std::vector<double>(f.begin(),f.begin()+3);
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

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml");
    ft.calibrateFtBias(10);

    return;
}

void calAttach(){
    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml",
                 "~/Dan/atiFTDriver/config/drillConfig.xml");

    double calt=3;

    std::vector<std::vector<double> > Rdown= makeMat<double>(3,3,0);
    std::vector<std::vector<double> > Rup= eye<double>(3,3);
    Rdown.at(1).at(1)=-1; Rdown.at(2).at(2)=-1;

    printf("Calibrating Payload..\n");
    for (int i=0;i<10;i++){
        printf("Position %d..\n",i);

        std::chrono::high_resolution_clock::time_point t0=
                std::chrono::high_resolution_clock::now();
        double t=0;
        while (t<calt){
            if (i) ft.setSensorR(Rdown);
            else ft.setSensorR(Rup);

            ft.calDrillBuf();

            t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
            usleep(1e3);
        }

        printf("Press Enter to Continue Or 'q' to Finish:");
        std::string a;
        std::getline(std::cin,a);
        if (!a.compare("q"))
            break;
    }

    std::vector<double> attachWP= ft.calibrateDrillFromBuf();
    print_vector("attachWP",attachWP);
    ft.save_drillCalibration();
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

//    test();

    return 0;
}
