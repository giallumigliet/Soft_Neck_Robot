#include "Cia402device.h"
#include "imu3dmgx510.h"
#include <iostream>
#include "fcontrol.h"
#include "IPlot.h"

// MULTIMETER #include
#include "SerialComm.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

// CSV file creation
#include <fstream>
#include <sstream>


// MULTIMETER: for serial port configuration
SerialComm* setupSerialPort(const std::string& portName, long baudrate = 9600) {
    SerialComm* serial = new SerialComm(portName, baudrate);
    return serial;
}

// MULTIMETER: to send commands to multimeter
void sendCommand(SerialComm* serial, const std::string& command) {
    if (serial->WriteLine(command) <= 0) {
        std::cerr << "Error during command sending to multimeter" << std::endl;
    }
}

// MULTIMETER: to read data from multimeter
double readMultimeterData(SerialComm* serial) {
    sendCommand(serial, "MEAS:FRESistance?");
    std::string data = serial->GetLine();

    if (!data.empty()) {
        std::cout << "Multimeter Data: " << data << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
        try {
            return std::stod(data); // Conversion to double
        } catch (...) {
            std::cerr << "Error during data conversion to double." << std::endl;
            return 0.0;
        }
    } else {
        std::cerr << "No data received." << std::endl;
        return 0.0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(0)); // Pause between readings
    return 0.0;
}



vector<double> rpy = {0.01,0.01,0};
vector<double> tl(3);

// CONVERSION FUNCTION: rpyANGLES -> tendon length
long rpy2tendon(vector<double> rpy, vector<double>& tl)
{

    //Considering positive pitch bending the neck forward!!!
    tl[0] = rpy[1]/1.5;
    tl[1] = ( - (rpy[1] / 3) - (rpy[0] / 1.732) );
    tl[2] = ( (rpy[0] / 1.732) - (rpy[1] / 3) );
    //cout<<"tl[0]: "<<tl[0]<<", "<<"tl[1]: "<<tl[1]<<", "<<"tl[2]: "<<tl[2]<<endl;

    return 0;
}

// CONVERSION FUNCTION: rpyANGLES -> motor position
long rpy2mot(vector<double> rpy, vector<double>& tl)
{
    double rm=0.0075; //winch radius

    tl[0] = (rpy[1]/1.5)/rm;
    tl[1] = ( - (rpy[1] / 3) - (rpy[0] / 1.732) )/rm;
    tl[2] = ( (rpy[0] / 1.732) - (rpy[1] / 3) )/rm;
    //cout<<"tl[0]: "<<tl[0]<<", "<<"tl[1]: "<<tl[1]<<", "<<"tl[2]: "<<tl[2]<<endl;
    return 0;
}


// EXPORT CSV FILE FUNCTIONS
void exportVectorToCSV(const std::vector<double>& data, const std::string& filePath) {
    // Create and open the file at the specified path
    std::ofstream outFile(filePath);

    // Check if the file was opened successfully
    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filePath << std::endl;
        return;
    }

    // Write the data to the CSV file
    for (const auto& value : data) {
        outFile << value << "\n";  // Write each value in a new line
    }

    // Close the file
    outFile.close();

    std::cout << "Data has been written to " << filePath << std::endl;
}

void export2DVectorToCSV(const std::vector<std::vector<double>>& matrix, const std::string& filePath) {
    // Open the file to write
    std::ofstream outFile(filePath);

    // Check if the file is open
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }

    // Write each row of the matrix
    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            outFile << matrix[i][j];
            if (j != matrix[i].size() - 1) {
                outFile << ",";  // Separate values with commas
            }
        }
        outFile << std::endl;  // New line after each row
    }

    // Close the file after writing
    outFile.close();
    std::cout << "Data has been written to " << filePath << std::endl;
}





int main()
{
    //--Can port communications----------
    string can = "can0";
    SocketCanPort pm1(can);
    SocketCanPort pm2(can);
    SocketCanPort pm3(can);

    //CiA402SetupData takes as input the following arguments: (int new_encRes,float new_mlRatio, float new_SampSL,float motor_current_limit,float drive_current_limit)
    CiA402SetupData sd1(2048,24,0.001, 0.144,20); //max amp 20
    CiA402SetupData sd2(2048,24,0.001, 0.144,20); //max amp 20
    CiA402SetupData sd3(2048,24,0.001, 0.144,20); //max amp 20


    ///Create a joint and give a canopen id, and a 301port (by constructor)
    CiA402Device m1(1,&pm1, &sd1);
    CiA402Device m2(2,&pm2, &sd2);
    CiA402Device m3(3,&pm3, &sd3);


    //--Motors Setup-----------------------
    //NOTE: BE SURE THAT THE WIRES ARE ROLLED IN THE RIGHT DIRECTION BEFORE POWERING THE NECK (correct direction: wires being external in a vertical position)
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();
    //m33.DisablePDOs();

    //set velocity mode and aceleration (rads/s^2)
    m1.Setup_Velocity_Mode(20);
    m2.Setup_Velocity_Mode(20);
    m3.Setup_Velocity_Mode(20);



    //--IMU Sensor-----------------------------
    cout << "------------------------------------" <<  endl;
    double freq=50; //IMU sensor use values: 50,100,500... (better keep on 50 to avoid issues)
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);  // IMU port selection (portName,frequency)

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();
    //cout << "Ping: " << misensor.Ping() << endl;
    sleep(1); //wait for sensor


    //--MULTIMETER-----------------------------
    SerialComm* serial = setupSerialPort("/dev/ttyUSB1", 9600);   // MULTIMETER port selection (portName,baudrate)
    sendCommand(serial, "SYSTem:LOCal");
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause
    sendCommand(serial, "SYSTem:REMote");
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause


    //--Controlling Blocks---------------------
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);
    FPDBlock Cp(2.5773,3.2325,-0.85);
    FPDBlock Cr(2.6299,3.2395,-0.86);

    //--Plots----------------------------------
    IPlot roll(dts, "roll", "xLabel", "yLabel");
    IPlot pitch(dts, "pitch", "xLabel", "yLabel");
    IPlot rcs(dts, "rcs", "xLabel", "yLabel");
    IPlot pcs(dts, "pcs", "xLabel", "yLabel");

    //--IMU Calibration------------------------
    for (double t=0;t<5;t+=dts)
        {
        //misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;
        //cout<<"Calibrando"<<endl;

        //Do not comment the following two lines!!!
        double pitch,roll,yaw;
        misensor.GetPitchRollYaw(pitch,roll,yaw);

        cout << "Roll: " << roll << " Pitch: " << pitch  << " Yaw: " << yaw << " Time: " << t << endl;
        }

    cout << "Sensor started" << endl;


    //--Variables Creation---------------------
    vector<double> rpy = {0,0,0};
    vector<double> mv(3); //motor velocities
    vector<double> cs(3); //control signals
    double ep=0,er=0;


    //--Reference Signal Creation (rpy target)---------
    double interval=20; //in seconds        DURATION OF THE TEST

    std::vector<std::vector<double>> target_rpy(interval/dts, std::vector<double>(3));
    std::vector<std::vector<double>> data_rpy(interval/dts, std::vector<double>(3));
    std::vector<double> data_resistance(interval/dts, 0.0);
    std::vector<double> data_time(interval/dts, 0.0);


    double interval1=5;
    for (double t=0;t<interval1; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        target_rpy[index][0]=0.2;
        target_rpy[index][1]=0.2;
        target_rpy[index][2]=0;
        cout << "TARGET Roll: " << target_rpy[t][0] << " Pitch: " << target_rpy[t][1]  << " Yaw: " << target_rpy[t][2] << " Time: " << t << endl;
    }

    double interval2=10;
    for (double t=interval1;t<interval2; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        target_rpy[index][0]=0.2;
        target_rpy[index][1]=-0.2;
        target_rpy[index][2]=0;
        cout << "TARGET Roll: " << target_rpy[t][0] << " Pitch: " << target_rpy[t][1]  << " Yaw: " << target_rpy[t][2] << " Time: " << t << endl;
    }

    double interval3=15;
    for (double t=interval2;t<interval3; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        target_rpy[index][0]=-0.2;
        target_rpy[index][1]=-0.2;
        target_rpy[index][2]=0;
        cout << "TARGET Roll: " << target_rpy[t][0] << " Pitch: " << target_rpy[t][1]  << " Yaw: " << target_rpy[t][2] << " Time: " << t << endl;
    }

    double interval4=20;
    for (double t=interval3;t<interval4; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        target_rpy[index][0]=-0.2;
        target_rpy[index][1]=0.2;
        target_rpy[index][2]=0;
        cout << "TARGET Roll: " << target_rpy[t][0] << " Pitch: " << target_rpy[t][1]  << " Yaw: " << target_rpy[t][2] << " Time: " << t << endl;
    }

    for (double t=interval3;t<interval4; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        target_rpy[index][0]=-0.2;
        target_rpy[index][1]=0.2;
        target_rpy[index][2]=0;
        cout << "TARGET Roll: " << target_rpy[t][0] << " Pitch: " << target_rpy[t][1]  << " Yaw: " << target_rpy[t][2] << " Time: " << t << endl;
    }

//    cout << "--------" << endl;
//    for (int i=0; i<interval/dts; i++)
//    {
//        cout << "TARGET Roll: " << target_rpy[i][0] << " Pitch: " << target_rpy[i][1]  << " Yaw: " << target_rpy[i][2] << " Time: " << i*dts << endl;
//    }



    //--Control---------------------------------------
    cout << "----TEST STARTED----" << endl;

    for (double t=0;t<interval; t+=dts)
    {
        int index = static_cast<int>(t / dts);
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//THIS IS A PROBLEM WHEN UNCOMMENTED: THE COMMAND TO READ THE MULTIMETER TAKES TOO MUCH TIME, SO IT MESSES UP THE CONTROL---------------------------------------------

        double resistance = readMultimeterData(serial); //reading from multimeter
        data_resistance[index] = resistance;    //saving multimeter data
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

        misensor.GetPitchRollYaw(rpy[1],rpy[0],rpy[2]);  //reading from IMU
        rpy[1]=-rpy[1]; //Pitch angle opposite direction

        data_rpy[index][0]=rpy[1];  //saving IMU data
        data_rpy[index][1]=rpy[0];
        data_rpy[index][2]=rpy[2];

        data_time[index] = t;

        //computing roll and pitch errors
        er = target_rpy[index][0] - rpy[0];
        ep = target_rpy[index][1] - rpy[1];




        //plotting data
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

        //sending motor velocities
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

    //--Printing Saved Data-----------------------------
    for (double t=0;t<interval; t+=dts)
    {
        int index = static_cast<int>(t / dts);
        cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << data_time[index] << " Time t: " << t << " RES: " << data_resistance[index] << endl;

    }



    //--Export Data as CSV
    export2DVectorToCSV(data_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/rpy_data.csv");
    export2DVectorToCSV(target_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/target_rpy_input_data.csv");
    exportVectorToCSV(data_time, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/time_data.csv");
    exportVectorToCSV(data_resistance, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/resistance_data.csv");


}
