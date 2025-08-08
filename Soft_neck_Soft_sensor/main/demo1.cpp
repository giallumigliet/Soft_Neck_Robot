#include "Cia402device.h"
#include "imu3dmgx510.h"
#include <iostream>
#include "fcontrol.h"
#include "IPlot.h"


vector<double> rpy = {0.01,0.01,0};
vector<double> tl(3);


long rpy2tendon(vector<double> rpy, vector<double>& tl)
{

    //Considering positive pitch bending the neck forward!!!
    tl[0] = rpy[1]/1.5;
    tl[1] = ( - (rpy[1] / 3) - (rpy[0] / 1.732) );
    tl[2] = ( (rpy[0] / 1.732) - (rpy[1] / 3) );
//    cout<<"tl[0]: "<<tl[0]<<", "<<"tl[1]: "<<tl[1]<<", "<<"tl[2]: "<<tl[2]<<endl;

    return 0;
}

long rpy2mot(vector<double> rpy, vector<double>& tl)
{
    double rm=0.0075; //winch radius

    tl[0] = (rpy[1]/1.5)/rm;
    tl[1] = ( - (rpy[1] / 3) - (rpy[0] / 1.732) )/rm;
    tl[2] = ( (rpy[0] / 1.732) - (rpy[1] / 3) )/rm;

//    cout<<"tl[0]: "<<tl[0]<<", "<<"tl[1]: "<<tl[1]<<", "<<"tl[2]: "<<tl[2]<<endl;

    return 0;
}


int main()
{

    string can = "can0";

    //--Can port communications--
    SocketCanPort pm1(can);
    SocketCanPort pm2(can);
    SocketCanPort pm3(can);


    CiA402SetupData sd1(2*2048,24,0.001, 0.144,10); //max amp 20
    CiA402SetupData sd2(2*2048,24,0.001, 0.144,10); //max amp 20
    CiA402SetupData sd3(2*2048,24,0.001, 0.144,10); //max amp 20


    ///Create a joint and give a canopen id, and a 301port (by constructor)
    CiA402Device m1(1,&pm1, &sd1);
    CiA402Device m2(2,&pm2, &sd2);
    CiA402Device m3(3,&pm3, &sd3);


    //    Motor setup
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();
    //     m33.DisablePDOs();


    //set velocity mode and aceleration (rads/s^2)
    m1.Setup_Velocity_Mode(10);
    m2.Setup_Velocity_Mode(10);
    m3.Setup_Velocity_Mode(10);


    cout << "------------------------------------" <<  endl;
    //--sensor--
    double freq=50; //sensor use values: 50,100,500... (better keep on 50 to avoid issues)
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

//    cout << "Ping: " << misensor.Ping() << endl;

    sleep(1); //wait for sensor
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);
    FPDBlock Cp(2.5773,3.2325,-0.85);
    FPDBlock Cr(2.6299,3.2395,-0.86);

    //Plots
    IPlot roll(dts, "roll", "xLabel", "yLabel");
    IPlot pitch(dts, "pitch", "xLabel", "yLabel");
    IPlot rcs(dts, "rcs", "xLabel", "yLabel");
    IPlot pcs(dts, "pcs", "xLabel", "yLabel");


    for (double t=0;t<5;t+=dts)
        {
    //        misensor.GetPitchRollYaw(pitch,roll,yaw);
    //        cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;

//        cout<<"Calibrando"<<endl;

        //Do not comment the following two lines!!!
        double pitch,roll,yaw;

        misensor.GetPitchRollYaw(pitch,roll,yaw);
//    cout << "Sensor ------------------------------ 222222222222222222222222" << endl;
        cout << "Roll: " << roll << " Pitch: " <<pitch  << " Yaw: " << yaw << " time: " << t << endl;
        }

    cout << "Sensor started" << endl;



    vector<double> rpy = {0,0,0};
    vector<double> target_rpy = {0.3,0.3,0};
    vector<double> mv(3); //motor velocities
    vector<double> cs(3); //control signals
    double ep=0,er=0;
    cout << "----TEST STARTED----" << endl;

    double interval=20; //in seconds        DURATION OF THE TEST
    for (double t=0;t<interval; t+=dts)
    {
        misensor.GetPitchRollYaw(rpy[1],rpy[0],rpy[2]);
        rpy[1]=-rpy[1]; //Pitch angle opposite direction

        er = target_rpy[0] - rpy[0];
        ep = target_rpy[1] - rpy[1];

        //PLOT DE DATOS
        roll.pushBack(rpy[0]);
        pitch.pushBack(rpy[1]);

        //controller computes control signal FPD
        cs[0] = 0.2*(er > Cr);
        cs[1] = 0.2*(ep > Cp);

        //PLOT DE Control
//        rcs.pushBack(cs[0]);
//        pcs.pushBack(cs[1]);

//        if (!isnormal(cs[0])) cs[0] = 0;
//        if (!isnormal(cs[1])) cs[1] = 0;

        // Enviando velocidad motores
        rpy2mot(cs,mv);
        m1.SetVelocity(mv[0]);
        m2.SetVelocity(mv[1]);
        m3.SetVelocity(mv[2]);

//        m1.SetVelocity(0.01);
//        m2.SetVelocity(-0.01);
//        m3.SetVelocity(-0.01);

       Ts.WaitSamplingTime();
    }
    cout << "----TEST COMPLETED----" << endl;

    m1.SetupPositionMode();
    m2.SetupPositionMode();
    m3.SetupPositionMode();
    misensor.Reset();

    cout << "----HOMING MOTORS----" << endl;

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    sleep(5);

//    roll.Plot();
//    pitch.Plot();
//    rcs.Plot();
//    pcs.Plot();

}
