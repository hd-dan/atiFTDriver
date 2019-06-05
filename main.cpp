#include <iostream>

#include "ft_sensor.h"
#include "../commun/commun.h"

void readFt(){
    commun plotServer("127.0.0.1",8888,0);

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml");
    std::vector<double> f;
    std::chrono::high_resolution_clock::time_point t0=
            std::chrono::high_resolution_clock::now();
    double t=0;
    while (t<60){
        f= ft.get_ftRaw();
        std::vector<double> fsub= std::vector<double>(f.begin(),f.begin()+3);
//        print_vector("f",f);
        plotServer.sendData(fsub);

        t= elapsedTime<double>(t0,std::chrono::high_resolution_clock::now());
        usleep(1e3);
    }
}

void calFt(){

    ft_sensor ft("~/Dan/atiFTDriver/config/ftConfig.xml");
//    ft.calibrateFtBias(10);
    std::vector<double> wp= ft.calibrateDrill(10);
    print_vector("wp",wp);

    return;
}

int main(){
    std::cout << "Hello World!" << std::endl;

//    readFt();
    calFt();

//    std::vector<std::vector<double> > A= {{1,2,3},{5,2,3},{1,1,2}};

//    for (auto i:A){
//        std::cout<< typeid(i).name() <<std::endl;
//        for (auto j:i){
//            printf("%.2f, ",j);
//        }
//        printf("\n");
//    }


    return 0;
}
