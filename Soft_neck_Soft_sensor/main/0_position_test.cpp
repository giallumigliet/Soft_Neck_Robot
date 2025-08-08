#include <iostream>
#include <math.h>

#include <bitset>

#include "TestPort.h"
#include "CanBusPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "SoftCC3Tendon.h"

using namespace std;

int main(int argc, char *argv[])
{

    ///prepare ports
    /// Open a port address with a PortBase Object
    string can = "can0";

    //--Can port communications--
    SocketCanPort pm1(can);
    SocketCanPort pm2(can);
    SocketCanPort pm3(can);


    CiA402SetupData sd1(2*2048,24,0.001, 0.144,20); //max amp 20
    CiA402SetupData sd2(2*2048,24,0.001, 0.144,20); //max amp 20
    CiA402SetupData sd3(2*2048,24,0.001, 0.144,20); //max amp 20


    ///Create a joint and give a canopen id, and a 301port (by constructor)
    CiA402Device j1(1,&pm1, &sd1);
    CiA402Device j2(2,&pm2, &sd2);
    CiA402Device j3(3,&pm3, &sd3);

    ///Check the status of the device
    j1.PrintStatus();
    j2.PrintStatus();
    j3.PrintStatus();

    j1.Reset();
    j2.Reset();
    j3.Reset();

    j1.PrintStatus();

    j1.SwitchOn();
    j1.PrintStatus();
    j2.SwitchOn();
    j2.PrintStatus();
    j3.SwitchOn();
    j3.PrintStatus();


    j1.SetupPositionMode(1,1);
    j2.SetupPositionMode(1,1);
    j3.SetupPositionMode(1,1);

//    j1.SetPosition(0.0);
//    j2.SetPosition(0.0);
//    j3.SetPosition(0.0);

//    sleep(2);

//    j1.SetPosition(2);
//    sleep(2);
//    j1.SetPosition(0.0);
//    j2.SetPosition(2);
//    sleep(2);
//    j2.SetPosition(0.0);
//    j3.SetPosition(2);
//    sleep(2);
//    j3.SetPosition(0.0);

//    sleep(2);

//    j1.SetPosition(1.5);
//    sleep(2);
//    j1.SetPosition(0.0);
//    sleep(2);
//    j2.SetPosition(1.5);
//    sleep(2);
//    j2.SetPosition(0.0);
    ////    sleep(2);
    //    j3.SetPosition(1.5);
    //    sleep(2);


    SoftCC3Tendon Soft_Neck(0.113,0.05);

    double roll = 0.3;
    double pitch = 0.3;
    double arcLength = 0.113;

    double rm = 0.0075;

    vector<double> target = {roll,pitch,arcLength};

    Soft_Neck.SetRollPitchArcLen(target);

    vector<double> tendonLengths = Soft_Neck.GetTendons();


    cout << " tendon 1 : " <<tendonLengths[0] << " tendon 2 : " << tendonLengths[1] << " tendon 3 : " <<tendonLengths[2] << endl;




    j1.SetPosition(-tendonLengths[0]/rm);
    j2.SetPosition(-tendonLengths[1]/rm);
    j3.SetPosition(-tendonLengths[2]/rm);

    sleep(2);


return 0;

}
