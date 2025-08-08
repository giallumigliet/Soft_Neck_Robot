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
#include <thread>

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

std::vector<double> timestamps_main;     
std::vector<double> timestamps_multimeter;

// MULTIMETER: thread function to read data from multimeter

void multimeterThreadFunction(SerialComm* serial, std::vector<double>& data_resistance, double dt, std::chrono::steady_clock::time_point start_time, bool& running) {
    int index = 0;
    while (running) {
        double resistance = readMultimeterData(serial); // reading from multimeter
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(now - start_time).count();

        timestamps_multimeter.push_back(elapsed_time);
        data_resistance[index] = resistance; // saving multimeter data
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000))); // Sleep for dt
        index++;
    }
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


// REFERENCE SIGNAL CREATOR FUNCTIONS
void referenceStep(std::vector<double>& angle, double dt, double amplitude, double duration){
    //STEP-------------------------------------------------------------------------
    int numSteps = static_cast<int>(duration / dt) + 1; // +1 to include t = 0
    std::vector<double> step(numSteps);

    for(int i = 0; i < numSteps; i++) {
        step[i] = amplitude;
    }

    angle = step;
}

void referenceRamp(std::vector<double>& angle, double dt, double amplitude, double rate){
     //RAMP--------------------------------------------------------------------------
     double period = 1/rate;

     int numSteps = static_cast<int>(period / dt) + 1; // +1 to include t = 0
     std::vector<double> ramp(numSteps);

     double t;

     for(int i = 0; i < numSteps; i++) {
         t = i * dt;
         ramp[i] = amplitude * rate * t;
     }

     angle = ramp;
}

void referenceDoubleRamp(std::vector<double>& angle, double dt, double amplitude, double rate){
     //DOUBLE RAMP--------------------------------------------------------------------------
     double period = 2 * 1/rate;

     int numSteps = static_cast<int>(period / dt) + 1; // +1 to include t = 0
     std::vector<double> ramp(numSteps);

     double t;
     int i;

     //increasing ramp
     for(i = 0; i < numSteps/2; i++) {
         t = i * dt;
         ramp[i] = amplitude * rate * t;
     }
     //decreasing ramp
     for(i = numSteps/2; i < numSteps; i++) {
         t = i * dt;
         ramp[i] = amplitude - amplitude * rate * (t-period/2);
     }

     angle = ramp;
}

void referenceCyclicRamp(std::vector<double>& angle, double dt, double amplitude, double rate, int nCycles){
    //DOUBLE RAMP--------------------------------------------------------------------------
    double period = 2 * 1/rate;

    int numSteps = static_cast<int>(period * nCycles/ dt) + 1; // +1 to include t = 0
    std::vector<double> ramp(numSteps);

    double t;
    int i;

    for(int cycle = 0; cycle < nCycles; cycle++) {
        //rampa (crescente)
        for(i = (0 + cycle * numSteps)/nCycles; i < (numSteps/2 + cycle * numSteps)/nCycles; i++) {
            t = (i-(cycle * numSteps/nCycles)) * dt;
            ramp[i] = amplitude * rate * t;   // y = A*(mx)
        }
        //rampa (decrescente)
        for(i = (numSteps/2 + cycle * numSteps)/nCycles; i < (numSteps + cycle * numSteps)/nCycles; i++) {
            t = (i-(cycle * numSteps/nCycles)) * dt;
            ramp[i] = amplitude - amplitude * rate * (t - period/2);   // y = A*(-mx) + q; q=A
        }
    }
    angle = ramp;

}

void referenceSinusoidal(std::vector<double>& angle, double dt, double amplitude, double freq, int nCycles){
    //SINUSOID--------------------------------------------------------------------------
    double period = 1/freq; //freq is in Hz, not in rad/s
    int numSteps = static_cast<int>(period * nCycles/ dt) + 1; // +1 to include t = 0
    std::vector<double> sinusoid(numSteps);

    double t;
    int i;

    for(int cycle = 0; cycle < nCycles; cycle++) {
        for(i = (0 + cycle * numSteps)/nCycles; i < (numSteps + cycle * numSteps)/nCycles; i++) {
            t = (i-(cycle * numSteps/nCycles)) * dt;
            sinusoid[i] = amplitude * sin(2 * M_PI * freq * t);   // y = A*sin(2*pi*f*t)
        }
    }
    angle = sinusoid;
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

    //set velocity mode and acceleration (rads/s^2)
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
    FPDBlock Cp(2.5773,3.2325,-0.85);   //original: Cp(2.5773,3.2325,-0.85);
    FPDBlock Cr(2.6299,3.2395,-0.86);   //original: Cr(2.6299,3.2395,-0.86);

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



    //--Reference Signal Creation (rpy target)---------
    std::vector<double> control_pitch;
    std::vector<double> control_roll;


    //****choose your reference signal***********
//    std::vector<double> target;
//    referenceRamp(target, dts, 0.5, 0.2); //RAMP->(std::vector<double>& pitch, double dt, double amplitude, double rate): target=vector in which it saves the trajectory, dt=time step
//    target_pitch.insert(target_pitch.end(), target.begin(), target.end());


//    std::vector<double> target1;
//    referenceStep(target1, dts, 0.0, 5.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());


//    std::vector<double> target2;
//    referenceDoubleRamp(target2, dts, 0.5, 0.2); //DOUBLE RAMP->(std::vector<double>& pitch, double dt, double amplitude, double rate)
//    target_pitch.insert(target_pitch.end(), target2.begin(), target2.end());


//    std::vector<double> target3;
//    referenceCyclicRamp(target3, dts, 0.5, 0.5, 3); //CYCLIC RAMPS->(std::vector<double>& pitch, double dt, double amplitude, double rate, int nCycles)
//    target_pitch.insert(target_pitch.end(), target3.begin(), target3.end());

    std::vector<double> target1;
    referenceStep(target1, dts, 0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target1.begin(), target1.end());
    std::vector<double> target2;
    referenceStep(target2, dts, -0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target2.begin(), target2.end());
    std::vector<double> target3;
    referenceStep(target3, dts, 0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target3.begin(), target3.end());
    std::vector<double> target4;
    referenceStep(target4, dts, -0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target4.begin(), target4.end());
    std::vector<double> target5;
    referenceStep(target5, dts, 0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target5.begin(), target5.end());
    std::vector<double> target6;
    referenceStep(target6, dts, -0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target6.begin(), target6.end());
    std::vector<double> target7;
    referenceStep(target7, dts, 0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target7.begin(), target7.end());
    std::vector<double> target8;
    referenceStep(target8, dts, -0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target8.begin(), target8.end());
    std::vector<double> target9;
    referenceStep(target9, dts, 0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target9.begin(), target9.end());
    std::vector<double> target10;
    referenceStep(target10, dts, -0.2, 2.5); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_pitch.insert(control_pitch.end(), target10.begin(), target10.end());


    std::vector<double> target1r;
    referenceStep(target1r, dts, 0.0, 25); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    control_roll.insert(control_roll.end(), target1r.begin(), target1r.end());

    //*****************************************

    int size = control_pitch.size();
    cout << "Size: " << size << endl;

    for(int index=0; index<size; index++){
        cout << "CONTROL Roll: " << control_roll[index]<< " Pitch: " << control_pitch[index] << " Time: " << index*dts << endl;
    }

    std::vector<std::vector<double>> target_rpy(size, std::vector<double>(3));
    std::vector<std::vector<double>> data_rpy(size, std::vector<double>(3));

    std::vector<double> data_resistance(size, 0.0);
    std::vector<double> data_time(size, 0.0);

    std::vector<double> data_position_motor1;
    std::vector<double> data_position_motor2;
    std::vector<double> data_position_motor3;
    std::vector<double> data_velocity_motor1;
    std::vector<double> data_velocity_motor2;
    std::vector<double> data_velocity_motor3;

    std::vector<double> data_control_signal_roll_velocity;
    std::vector<double> data_control_signal_pitch_velocity;



    //--Control---------------------------------------
    bool running = true;

    auto start_time = std::chrono::steady_clock::now();
    std::thread multimeterThread(multimeterThreadFunction, serial, std::ref(data_resistance), dts, start_time, std::ref(running)); //THE COMMAND TO READ THE MULTIMETER TAKES TOO MUCH TIME, SO IT MESSES UP THE CONTROL (this is why it is not inside the control loop)



    cout << "----TEST STARTED----" << endl;

    for (double t=0;t<size*dts; t+=dts)
    {
        int index = static_cast<int>(t / dts);

        misensor.GetPitchRollYaw(rpy[1],rpy[0],rpy[2]);  //reading from IMU

        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(now - start_time).count();
        timestamps_main.push_back(elapsed_time);


        rpy[1]=-rpy[1]; //Pitch angle opposite direction

        data_rpy[index][0]=rpy[1];  //saving IMU data
        data_rpy[index][1]=rpy[0];
        data_rpy[index][2]=rpy[2];

        data_time[index] = t;


        //controller computes control signal FPD
        cs[0] = control_roll[index];
        cs[1] = control_pitch[index];


        //Converting motor velocities
        rpy2mot(cs,mv);

        data_position_motor1.push_back(m1.GetPosition());
        data_position_motor2.push_back(m2.GetPosition());
        data_position_motor3.push_back(m3.GetPosition());

        data_control_signal_roll_velocity.push_back(cs[0]);
        data_control_signal_pitch_velocity.push_back(cs[1]);

        data_velocity_motor1.push_back(mv[0]);
        data_velocity_motor2.push_back(mv[1]);
        data_velocity_motor3.push_back(mv[2]);





        //MOTOR POSITION LIMIT: this is necessary to prevent the wire from loosening, otherwise it gets tangled
        //basically, if the motor reaches its upper limit, the positive velocities are zeroed out; if the motor reaches its lower limit, the negative velocities are zeroed out;

//        double maxPosition = 8;

//        if((m1.GetPosition()>=maxPosition && mv[0]>0) || (m1.GetPosition()<=-maxPosition && mv[0]<0)) m1.SetVelocity(0);
//        else m1.SetVelocity(mv[0]);


//        if((m2.GetPosition()>=maxPosition && mv[1]>0) || (m2.GetPosition()<=-maxPosition && mv[1]<0)) m2.SetVelocity(0);
//        else m2.SetVelocity(mv[1]);


//        if((m3.GetPosition()>=maxPosition && mv[2]>0) || (m3.GetPosition()<=-maxPosition && mv[2]<0)) m3.SetVelocity(0);
//        else m3.SetVelocity(mv[2]);



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

    running = false;
    multimeterThread.join();
    sleep(2);



//    roll.Plot();
//    pitch.Plot();
//    rcs.Plot();
//    pcs.Plot();

    //--Printing Saved Data-----------------------------

    // for(double index=0; index<size; index++)
    // {
    //     int index = static_cast<int>(t / dts);
    //     cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << data_time[index] << " Time t: " << index*dts << " RES: " << data_resistance[index] << endl;
    // }


    cout << "\nTimestamps del ciclo principale:\n";
    for(double index=0; index<size; index++) {
        cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << timestamps_main[index] << " s" <<endl;
    }

    cout << "\nTimestamps del thread del multimetro:\n";
    for(double index=0; index<size; index++) {
        cout << " RES: " << data_resistance[index] << " Time: " << timestamps_multimeter[index] << " s" <<endl;
    }



    //--Export Data as CSV
    export2DVectorToCSV(data_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/IMU_rpy_data.csv");
    export2DVectorToCSV(target_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/target_rpy_input_data.csv");
    exportVectorToCSV(timestamps_main, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/IMU_rpy_time_data.csv");

    exportVectorToCSV(data_resistance, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/resistance_data.csv");
    exportVectorToCSV(timestamps_multimeter, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/resistance_time_data.csv");


    exportVectorToCSV(data_control_signal_roll_velocity, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_control_signal_roll_velocity.csv");
    exportVectorToCSV(data_control_signal_pitch_velocity, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_control_signal_pitch_velocity.csv");

    exportVectorToCSV(data_position_motor1, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_position_motor1.csv");
    exportVectorToCSV(data_position_motor2, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_position_motor2.csv");
    exportVectorToCSV(data_position_motor3, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_position_motor3.csv");

    exportVectorToCSV(data_velocity_motor1, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_velocity_motor1.csv");
    exportVectorToCSV(data_velocity_motor2, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_velocity_motor2.csv");
    exportVectorToCSV(data_velocity_motor3, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_velocity_motor3.csv");


}
