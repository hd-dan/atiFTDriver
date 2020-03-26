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

std::vector<double> calBias(std::vector<std::vector<double> > process){
    std::vector<double> bias(6,0);
    for (unsigned int i=0;i<process.size();i++){
        for (unsigned int j=1;j<6;j++){
            bias.at(j-1)+= process.at(i).at(j);
        }
    }
    for (unsigned int i=0;i<bias.size();i++){
        bias.at(i)*= (1./process.size());
    }
    return bias;
}

double calSlope1(std::vector<std::vector<double> > mh, unsigned int hInd){
    std::vector<double> mDiff(0,0);
    std::vector<double> hDiff(0,0);
    std::vector<double> slopeVec(0,0);
    for (unsigned int i=0;i<mh.size()-1;i++){
        for (unsigned int j=i+1;j<mh.size();j++){
            mDiff.push_back(mh.at(i).at(0)-mh.at(j).at(0));
            hDiff.push_back(mh.at(i).at(hInd)-mh.at(j).at(hInd));
            slopeVec.push_back( mDiff.back()*9.81/hDiff.back() );
        }
    }
    double slope_avg=0;
    for (unsigned int i=0;i<slopeVec.size();i++){
        slope_avg+= slopeVec.at(i);
    }
    return slope_avg/slopeVec.size();
}

void calFromData(){
//    std::string dataPath= "/home/airportvision/Desktop/calRecord1.csv";
//    data_file dataFile(dataPath,false);
//    std::vector<std::vector<double> > data= dataFile.getContent();
//    print_matrix("data",data);

    std::vector<std::vector<double> > calData= {
//        zdown
        {0.0132,	-2.50473,	-26.5068,	-10.3492,	0.262448,	0.48479,	0.00991011,	3.43867e-06,	2.09156e-05,	-1},
        {0.0789,	-2.49993,	-26.5187,	-9.50606,	0.266233,	0.482028,	0.010308,	-4.97593e-05,	4.78035e-05,	-1},
        {0.09,	-2.49302,	-26.5138,	-9.4858,	0.264906,	0.480494,	0.0125059,	-1.61942e-05,	-8.00219e-05,	-1},
        {0.1146,	-2.48573,	-26.5125,	-9.17297,	0.267398,	0.480304,	0.0123667,	-2.2197e-05,	2.82358e-05,	-1},
        {0.1866,	-2.49151,	-26.5265,	-8.21832,	0.270914,	0.476718,	0.0123659,	-3.98316e-05,	-8.39695e-05,	-1},
//        zup
        {0.0132,	-2.36553,	-26.6525,	-11.497,	0.255276,	0.482576,	0.00654016,	0.00112196,	0.000950513,	0.999999},
        {0.0789,	-2.36145,	-26.6633,	-12.2705,	0.254633,	0.483287,	0.00407163,	0.00108872,	0.000821678,	0.999999},
        {0.09,	-2.35767,	-26.682,	-12.4972,	0.254407,	0.486653,	0.00520752,	0.0010948,	0.000858578,	0.999999},
        {0.1146,	-2.38403,	-26.7347,	-12.6641,	0.259539,	0.49453,	0.00261892,	0.00111646,	0.000843019,	0.999999},
        {0.1866,	-2.3626,	-26.7947,	-13.594,	0.255541,	0.512345,	0.00221316,	0.00110661,	0.00078276,	0.999999},
//        ydown
        {0.0132,	-2.45754,	-27.1038,	-10.9702,	0.258671,	0.484536,	0.00737069,	-0.00211596,	0.999998,	-5.46305e-07},
        {0.0789,	-2.82041,	-28.1502,	-11.011,	0.332777,	0.488617,	0.0158102,	-0.0020804,	0.999998,	3.4019e-06},
        {0.09,	-2.88106,	-28.3283,	-11.0824,	0.345324,	0.489073,	0.0184793,	-0.00225827,	0.999997,	-2.23332e-05},
        {0.1146,	-3.02481,	-28.714,	-10.9191,	0.372492,	0.490677,	0.0181398,	-0.00210406,	0.999998,	-4.53324e-06},
        {0.1866,	-3.40147,	-29.8548,	-11.0976,	0.455319,	0.49532,	0.0298634,	-0.0022222,	0.999998,	-2.02857e-05},
//        yup
        {0.0132,	-2.45754,	-27.1038,	-10.9702,	0.258671,	0.484536,	0.00737069,	-0.00211596,	0.999998,	-5.46305e-07},
        {0.0789,	-2.82041,	-28.1502,	-11.011,	0.332777,	0.488617,	0.0158102,	-0.0020804,	0.999998,	3.4019e-06},
        {0.09,	-2.88106,	-28.3283,	-11.0824,	0.345324,	0.489073,	0.0184793,	-0.00225827,	0.999997,	-2.23332e-05},
        {0.1146,	-3.02481,	-28.714,	-10.9191,	0.372492,	0.490677,	0.0181398,	-0.00210406,	0.999998,	-4.53324e-06},
        {0.1866,	-3.40147,	-29.8548,	-11.0976,	0.455319,	0.49532,	0.0298634,	-0.0022222,	0.999998,	-2.02857e-05},
//        xdown
        {0.0132,	-1.86506,	-26.6224,	-11.0094,	0.257848,	0.483692,	0.00691626,	-0.999999,	-0.00105617,	-8.44067e-06},
        {0.0789,	-0.827836,	-26.964,	-10.9571,	0.255081,	0.561885,	-0.00385059,	-0.999999,	-0.0011164,	-1.24514e-05},
        {0.09,	-0.639787,	-27.0237,	-11.077,	0.254607,	0.574285,	-0.00335419,	-0.999999,	-0.00103247,	-2.49071e-06},
        {0.1146,	-0.252836,	-27.1475,	-11.2055,	0.253672,	0.603493,	-0.00465657,	-1,	-0.000996913,	1.4575e-06},
        {0.1866,	0.867757,	-27.5073,	-11.0186,	0.250781,	0.687654,	-0.0182022,	-0.999999,	-0.00104485,	-8.46092e-06},
//        xup
        {0.0132,	-2.90063,	-26.5936,	-11.0591,	0.256123,	0.482182,	0.00776117,	0.999998,	-0.00212357,	5.38607e-06},
        {0.0789,	-3.9185,	-26.2554,	-11.0168,	0.258381,	0.406278,	-0.00217625,	0.999998,	-0.00205253,	-4.67498e-07},
        {0.09,	-4.10402,	-26.2262,	-11.1076,	0.258713,	0.393709,	-0.00227637,	0.999998,	-0.00213535,	1.13591e-05},
        {0.1146,	-4.48213,	-26.1027,	-11.2579,	0.259529,	0.364576,	-0.00308635,	0.999998,	-0.00213537,	5.36681e-06},
        {0.1866,	-5.56514,	-25.698,	-11.0318,	0.261187,	0.281143,	-0.0165666,	0.999998,	-0.00207633,	1.49689e-06},
    };

    std::vector<double> com_= {0,0,0.045};

    std::vector<double> m_est(6,0);
    std::vector<double> c_est(6,0);

    std::vector<std::vector<double> > mcBuf(12,std::vector<double>(0,0));

    calData= transpose(calData);
    for (unsigned int i=0;i<calData.at(0).size()-1;i++){
        int k=calData.size();
        std::vector<double> r3_i(3,0);
        for (int p=3;p>0;p--)
            r3_i.at(p-1)= calData.at(--k).at(i);
        std::vector<double> h_i(6,0);
        for (int p=6;p>0;p--)
            h_i.at(p-1)= calData.at(--k).at(i);
        double massi= calData.at(--k).at(i);


        for (unsigned int j=i+1;j<calData.at(0).size();j++){
            int k=calData.size();
            std::vector<double> r3_j(3,0);
            for (int p=3;p>0;p--)
                r3_j.at(p-1)= calData.at(--k).at(j);

            if (l2norm(r3_j-r3_i)>1e-3){
//                print_matrix("mcBuf",mcBuf);
//                return;
                continue;
            }

            std::vector<double> h_j(6,0);
            for (int p=6;p>0;p--)
                h_j.at(p-1)= calData.at(--k).at(j);
            double massj= calData.at(--k).at(j);

            std::vector<double> wDiff= 9.81*(r3_j*massj-r3_i*massi);
            std::vector<double> hDiff= h_j-h_i;
            for(unsigned ax=0;ax<3;ax++){
                if (fabs(r3_i.at(ax))<1e-2){
                    mcBuf.at(ax+6).push_back(0.5*(h_j.at(ax)+h_i.at(ax)));
                }else{
                    if (fabs(hDiff.at(ax))>1e-3){
                        mcBuf.at(ax).push_back( wDiff.at(ax)/hDiff.at(ax) );
//                        if (fabs(mcBuf.at(ax).back())>1){
//                        printf("i:%d, j:%d |axis:%d\n",i,j,ax);
//                        printf("massi:%.3f, massj:%.3f\n",massi,massj);
//                        print_vector("hDiff",hDiff);
//                        print_vector("wDiff",wDiff);
//                        printf("slope: %.3f\n\n",wDiff.at(ax)/hDiff.at(ax));
//                        return;
//                        }
                    }
                }
            }

        }
    }
    print_matrix("mcBuf",mcBuf);

    std::vector<double> mc_avg(0,0);

    print_vector("mc_avg",mc_avg);

}

void calBiasFromData(){
    std::vector<std::vector<double> > zdown= {
        {0.0132,	-2.50473,	-26.5068,	-10.3492,	0.262448,	0.48479,	0.00991011,	3.43867e-06,	2.09156e-05,	-1},
        {0.0789,	-2.49993,	-26.5187,	-9.50606,	0.266233,	0.482028,	0.010308,	-4.97593e-05,	4.78035e-05,	-1},
        {0.09,	-2.49302,	-26.5138,	-9.4858,	0.264906,	0.480494,	0.0125059,	-1.61942e-05,	-8.00219e-05,	-1},
        {0.1146,	-2.48573,	-26.5125,	-9.17297,	0.267398,	0.480304,	0.0123667,	-2.2197e-05,	2.82358e-05,	-1},
        {0.1866,	-2.49151,	-26.5265,	-8.21832,	0.270914,	0.476718,	0.0123659,	-3.98316e-05,	-8.39695e-05,	-1},
    };
    std::vector<std::vector<double> > zup= {
        {0.0132,	-2.36553,	-26.6525,	-11.497,	0.255276,	0.482576,	0.00654016,	0.00112196,	0.000950513,	0.999999},
        {0.0789,	-2.36145,	-26.6633,	-12.2705,	0.254633,	0.483287,	0.00407163,	0.00108872,	0.000821678,	0.999999},
        {0.09,	-2.35767,	-26.682,	-12.4972,	0.254407,	0.486653,	0.00520752,	0.0010948,	0.000858578,	0.999999},
        {0.1146,	-2.38403,	-26.7347,	-12.6641,	0.259539,	0.49453,	0.00261892,	0.00111646,	0.000843019,	0.999999},
        {0.1866,	-2.3626,	-26.7947,	-13.594,	0.255541,	0.512345,	0.00221316,	0.00110661,	0.00078276,	0.999999},
    };
    std::vector<std::vector<double> > ydown= {
        {0.0132,	-2.45754,	-27.1038,	-10.9702,	0.258671,	0.484536,	0.00737069,	-0.00211596,	0.999998,	-5.46305e-07},
        {0.0789,	-2.82041,	-28.1502,	-11.011,	0.332777,	0.488617,	0.0158102,	-0.0020804,	0.999998,	3.4019e-06},
        {0.09,	-2.88106,	-28.3283,	-11.0824,	0.345324,	0.489073,	0.0184793,	-0.00225827,	0.999997,	-2.23332e-05},
        {0.1146,	-3.02481,	-28.714,	-10.9191,	0.372492,	0.490677,	0.0181398,	-0.00210406,	0.999998,	-4.53324e-06},
        {0.1866,	-3.40147,	-29.8548,	-11.0976,	0.455319,	0.49532,	0.0298634,	-0.0022222,	0.999998,	-2.02857e-05},
    };
    std::vector<std::vector<double> > yup= {
        {0.0132,	-2.45754,	-27.1038,	-10.9702,	0.258671,	0.484536,	0.00737069,	-0.00211596,	0.999998,	-5.46305e-07},
        {0.0789,	-2.82041,	-28.1502,	-11.011,	0.332777,	0.488617,	0.0158102,	-0.0020804,	0.999998,	3.4019e-06},
        {0.09,	-2.88106,	-28.3283,	-11.0824,	0.345324,	0.489073,	0.0184793,	-0.00225827,	0.999997,	-2.23332e-05},
        {0.1146,	-3.02481,	-28.714,	-10.9191,	0.372492,	0.490677,	0.0181398,	-0.00210406,	0.999998,	-4.53324e-06},
        {0.1866,	-3.40147,	-29.8548,	-11.0976,	0.455319,	0.49532,	0.0298634,	-0.0022222,	0.999998,	-2.02857e-05},
    };
    std::vector<std::vector<double> > xdown= {
        {0.0132,	-1.86506,	-26.6224,	-11.0094,	0.257848,	0.483692,	0.00691626,	-0.999999,	-0.00105617,	-8.44067e-06},
        {0.0789,	-0.827836,	-26.964,	-10.9571,	0.255081,	0.561885,	-0.00385059,	-0.999999,	-0.0011164,	-1.24514e-05},
        {0.09,	-0.639787,	-27.0237,	-11.077,	0.254607,	0.574285,	-0.00335419,	-0.999999,	-0.00103247,	-2.49071e-06},
        {0.1146,	-0.252836,	-27.1475,	-11.2055,	0.253672,	0.603493,	-0.00465657,	-1,	-0.000996913,	1.4575e-06},
        {0.1866,	0.867757,	-27.5073,	-11.0186,	0.250781,	0.687654,	-0.0182022,	-0.999999,	-0.00104485,	-8.46092e-06},
    };
    std::vector<std::vector<double> > xup= {
        {0.0132,	-2.90063,	-26.5936,	-11.0591,	0.256123,	0.482182,	0.00776117,	0.999998,	-0.00212357,	5.38607e-06},
        {0.0789,	-3.9185,	-26.2554,	-11.0168,	0.258381,	0.406278,	-0.00217625,	0.999998,	-0.00205253,	-4.67498e-07},
        {0.09,	-4.10402,	-26.2262,	-11.1076,	0.258713,	0.393709,	-0.00227637,	0.999998,	-0.00213535,	1.13591e-05},
        {0.1146,	-4.48213,	-26.1027,	-11.2579,	0.259529,	0.364576,	-0.00308635,	0.999998,	-0.00213537,	5.36681e-06},
        {0.1866,	-5.56514,	-25.698,	-11.0318,	0.261187,	0.281143,	-0.0165666,	0.999998,	-0.00207633,	1.49689e-06},
    };

    std::vector<double> bias_z= calBias(zdown);
    print_vector("bias_z",bias_z);
    std::vector<double> bias_y= calBias(ydown);
    print_vector("bias_y",bias_y);
    std::vector<double> bias_x= calBias(xdown);
    print_vector("bias_x",bias_x);

    std::vector<std::vector<double> > biass= {
                    calBias(xdown),calBias(ydown),calBias(zdown)};
    std::vector<double> bias(6,0);

    bias.at(0)= biass.at(1).at(0) + biass.at(2).at(0);
    bias.at(1)= biass.at(0).at(1) + biass.at(2).at(1);
    bias.at(2)= biass.at(0).at(2) + biass.at(1).at(2);
    bias.at(3)= biass.at(1).at(3) + biass.at(2).at(3);
    bias.at(4)= biass.at(0).at(4) + biass.at(2).at(4);
    bias.at(5)= biass.at(0).at(5) + biass.at(1).at(5);

    bias= bias*0.5;
    print_vector("bias",bias);

    printf("slope_x: %.3f | ",calSlope1(xup,1));
    printf("%.3f\n",calSlope1(xdown,1));
    printf("slope_y: %.3f | ",calSlope1(yup,2));
    printf("%.3f\n",calSlope1(ydown,2));
    printf("slope_z: %.3f | ",calSlope1(zup,3));
    printf("%.3f\n",calSlope1(zdown,3));
}

int main(){
    std::cout << "Hello World!" << std::endl;

    /// yellow: x-axis | blue: y-axis
//    readFt();
//    calFt();

//    testFt();
    calFromData();

    return 0;
}
