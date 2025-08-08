#include "Cia402device.h"
#include "imu3dmgx510.h"
#include <iostream>
#include "ToolsFControl.h"
//#include "SystemBlock.h"

int main(){
    //--sensor--
    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;
//    double *EulerAngles;

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();
//    cout << "Ping: " << misensor.Ping() << endl;

    sleep(4); //wait for sensor
//    SystemBlock filterSensor(0.09516,0,- 0.9048,1);


    for (double t=0;t<100;t+=dts)
        {
    //        misensor.GetPitchRollYaw(pitch,roll,yaw);
    //        cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;


            misensor.GetPitchRollYaw(pitch,roll,yaw);
            cout<<"Calibrando"<<endl;
            cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
        }

    cout << "Sensor started" << endl;


//    for (double t=0;t<1000;t+=dts){

//        if (tilt.estimateSensor(incSensor,oriSensor)<0)
////        if (tilt.readSensor(incSensor,oriSensor)<0)
//        {
//            cout << "Sensor read error !" << endl;
//        }
//        else
//        {
//            cout << "incli_sen: " <<  (incSensor > filterSensor) << " , orient_sen: " << oriSensor << endl;
//        }
//        cout << "Available time: " << tools.WaitSamplingTime() << endl;
//    }

}
