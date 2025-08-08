#include "Cia402device.h"
#include "imu3dmgx510.h"
#include <iostream>
#include "fcontrol.h"
#include "IPlot.h"
#include "SoftCC3Tendon.h"  //for kinematics

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



//--Declaring global variables----------------------------------
std::vector<double> timestamps_main;
std::vector<double> timestamps_multimeter;
vector<double> rpy = {0.01,0.01,0};



//--Functions----------------------------------------------------
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




// MULTIMETER: thread function to read data from multimeter

void multimeterThreadFunction(SerialComm* serial, std::vector<double>& data_resistance, double dt, std::chrono::steady_clock::time_point start_time, bool& running) {
    int index = 0;
    while (running) {
        double resistance = readMultimeterData(serial); // reading from multimeter
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(now - start_time).count();

        timestamps_multimeter.push_back(elapsed_time);
        data_resistance.push_back(resistance); // saving multimeter data
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000))); // Sleep for dt
        index++;
    }
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

void referenceChirp(std::vector<double>& angle, double dt, double amplitude, double offset, double freq0, double freqF, double duration) {
    int numSteps = static_cast<int>(duration/dt);
    std::vector<double> chirp(numSteps);
    // Calcoliamo la variazione della frequenza per ogni campione
    for (int i = 0; i < numSteps; i++) {
        double t = i * dt;  // Tempo corrente (in secondi)

        // Frequenza al tempo t con variazione lineare (chirp)
        double freq_t = freq0 + (freqF - freq0) * (t / duration);

        // Calcoliamo il campione del segnale usando la funzione seno
        chirp[i] = amplitude* sin(2 * M_PI * freq_t * t) + offset;
    }
    angle = chirp;
}


// RESISTANCE TO ANGLE MODELS
double loadModel(double res) { //res is the normalized relative resistance
    double angle;
    double p1 = 4.5369e6;
    double p2 = 2.5358e4;
    double p3 = 133.73;
    double p4 = 0.5351;
    p4=0;
    angle = p1*res*res*res + p2*res*res +p3*res + p4;

    return angle;
}

double unloadModel(double res) { //res is the normalized relative resistance
    double angle;
    double p1 = 2.3512e7;
    double p2 = 2.2795e5;
    double p3 = 772.5816;
    double p4 = 1.1938;
    p4=0;
    angle = p1*res*res*res + p2*res*res + p3*res + p4;

    return angle;
}

double switchingModel(double alpha, vector<double> res, vector<double> res_time, double r0) {
    double angle, dR, dR_prev, slope, sigmoid;
    //Computing relative resistance
    dR = (res.back()-r0)/r0;
    dR_prev = (res[res.size()-2]-r0)/r0;
    //Computing angle
    slope = (dR - dR_prev)/(res_time.back() - res_time[res_time.size()-2]);
    sigmoid = 1 / (1 + exp(-alpha * slope));
    angle = sigmoid * loadModel(dR) + (1 - sigmoid) * unloadModel(dR);

    return angle;
}





int main()
{
    //--Can port communications--------------------------------
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


    //--Motors Setup---------------------------------------------
    //NOTE: BE SURE THAT THE WIRES ARE ROLLED IN THE RIGHT DIRECTION BEFORE POWERING THE NECK (correct direction: wires being external in a vertical position)
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();
    //m33.DisablePDOs();

    //set position mode (velocity (rads/s) and acceleration (rads/s^2))
    m1.SetupPositionMode(1,20);
    m2.SetupPositionMode(1,20);
    m3.SetupPositionMode(1,20);



    //--IMU Sensor---------------------------------------------------
    cout << "------------------------------------" <<  endl;
    //double freq=50; //IMU sensor use values: 50,100,500... (better keep on 50 to avoid issues)
    double freq=2; //Multimeter does around 2 measurements per seconds
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);  // IMU port selection (portName,frequency)

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();
    //cout << "Ping: " << misensor.Ping() << endl;
    sleep(1); //wait for sensor


    //--MULTIMETER---------------------------------------------------
    SerialComm* serial = setupSerialPort("/dev/ttyUSB1", 9600);   // MULTIMETER port selection (portName,baudrate)
    sendCommand(serial, "SYSTem:LOCal");
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause
    sendCommand(serial, "SYSTem:REMote");
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause

    //--Controlling Blocks---------------------
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);
    double gainP = 5/20;
    double gainI = 2;
    FPDBlock Cp(gainP*2.5773,gainI*3.2325,-0.85);   //original: Cp(2.5773,3.2325,-0.85);
    FPDBlock Cr(gainP*2.6299,gainI*3.2395,-0.86);   //original: Cr(2.6299,3.2395,-0.86);


    //--IMU Calibration-----------------------------------------------
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


    //--Variables Creation---------------------------------------------
    vector<double> rpy = {0,0,0};
    vector<double> cs(2); //control signals
    double ep=0,er=0;

    //--Plots----------------------------------------------------------
    IPlot roll(dts, "roll", "xLabel", "yLabel");
    IPlot pitch(dts, "pitch", "xLabel", "yLabel");
    IPlot resistance_plot(dts, "res", "xLabel", "yLabel");

    //--Reference Signal Creation (rpy target)-------------------------
    std::vector<double> target_pitch;
    std::vector<double> target_roll;

    std::vector<double> preload_pitch;
    std::vector<double> preload_roll;

    //****choose your reference signal*********************************

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

//    std::vector<double> target1;
//    referenceChirp(target1, dts, 0.174533, 0.174533, 0.01, 0.8, 100); //CHIRP ->(std::vector<double>& angle, double dt, double amplitude, double offset, double freq0, double freqF, double duration)
//    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());

    //0.785398 = 45 (rad=deg)
    //0.523599 = 30
    //0.349066 = 20
    //0.174533 = 10


    //20 degrees at 5deg/s
    std::vector<double> preload0;
    referenceCyclicRamp(preload0, dts, 0.349066, 0.087267, 2);
    preload_pitch.insert(preload_pitch.end(), preload0.begin(), preload0.end());

//    std::vector<double> preload1;
//    referenceStep(preload1, dts, 0.0, 2.0);
//    preload_pitch.insert(preload_pitch.end(), preload1.begin(), preload1.end());


    //20 degrees at 5deg/s
    std::vector<double> target0;
    referenceCyclicRamp(target0, dts, 0.349066, 0.087267, 1);
    target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());







    //*******************************************************************

    //--Creating variables to save data---------------------------------
    int size = target_pitch.size();
    int size_preload = preload_pitch.size();
    cout << "Size: " << size << endl;
    cout << "Preload Size: " << size_preload << endl;
    std::vector<std::vector<double>> target_rpy(size, std::vector<double>(3));
    std::vector<std::vector<double>> data_rpy(size+size_preload, std::vector<double>(3));
    std::vector<std::vector<double>> preload_rpy(size_preload, std::vector<double>(3));

    std::vector<double> data_resistance;
    std::vector<double> data_time;
    std::vector<double> preload_time;
    int index_preload;

    std::vector<double> data_position_motor1;
    std::vector<double> data_position_motor2;
    std::vector<double> data_position_motor3;
    std::vector<double> data_velocity_motor1;
    std::vector<double> data_velocity_motor2;
    std::vector<double> data_velocity_motor3;

    std::vector<double> data_control_signal_roll_velocity;
    std::vector<double> data_control_signal_pitch_velocity;

    std::vector<double> data_er;
    std::vector<double> data_ep;

    vector<double> dR, pitch_from_res;

    //--Assigning the roll pitch yaw values-------------------------------
    for(int index=0; index<size; index++){

        target_rpy[index][0]=0.0;
        target_rpy[index][1]=target_pitch[index];
        target_rpy[index][2]=0.0;

        //cout << "TARGET Roll: " << target_rpy[index][0] << " Pitch: " << target_rpy[index][1]  << " Yaw: " << target_rpy[index][2] << " Time: " << index*dts << endl;
    }

    cout << "--------" << endl;
    for(int i=0; i<size; i++){
        cout << "TARGET Roll: " << target_rpy[i][0] << " Pitch: " << target_rpy[i][1]  << " Yaw: " << target_rpy[i][2] << " Time: " << i*dts << endl;
    }

    for(int index=0; index<size_preload; index++){

        preload_rpy[index][0]=0.0;
        preload_rpy[index][1]=preload_pitch[index];
        preload_rpy[index][2]=0.0;

        //cout << "TARGET Roll: " << target_rpy[index][0] << " Pitch: " << target_rpy[index][1]  << " Yaw: " << target_rpy[index][2] << " Time: " << index*dts << endl;
    }




    //--Kinematics initialization---------------------------------------
    SoftCC3Tendon Soft_Neck(0.114,0.05);    //0.113m=11.3cm: height of the neck; 0.05m=5cm: distance from center to tendons in a limb cross section.

    double arc_length = 0.114;
    double rm = 0.0075;     //motors winch radius
    vector<double> currentTarget = {0.0, 0.0,arc_length};  // {roll,pitch,arc_length}
    Soft_Neck.SetRollPitchArcLen(currentTarget);

    vector<double> tendonLengths = Soft_Neck.GetTendons();
    cout << " tendon 1 : " <<tendonLengths[0] << " tendon 2 : " << tendonLengths[1] << " tendon 3 : " <<tendonLengths[2] << endl;




    //--Multimeter Thread-----------------------------------------------
    bool running = true;

    auto start_time = std::chrono::steady_clock::now();
    std::thread multimeterThread(multimeterThreadFunction, serial, std::ref(data_resistance), dts, start_time, std::ref(running)); //THE COMMAND TO READ THE MULTIMETER TAKES TOO MUCH TIME, SO IT MESSES UP THE CONTROL (this is why it is not inside the control loop)




    //--Preload---------------------------------------------------------
    cout << "----PRELOAD STARTED----" << endl;

    for (double t=0;t<size_preload*dts; t+=dts)
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

        //preload_time[index] = t;

        //Converting motor Position
        currentTarget = {0,preload_pitch[index],arc_length};
        Soft_Neck.SetRollPitchArcLen(currentTarget);
        tendonLengths = Soft_Neck.GetTendons();


        //Setting Position
        m1.SetPosition(-tendonLengths[0]/rm);
        m2.SetPosition(-tendonLengths[1]/rm);
        m3.SetPosition(-tendonLengths[2]/rm);

        //Saving data
        roll.pushBack(rpy[0]);
        pitch.pushBack(rpy[1]);

        data_position_motor1.push_back(m1.GetPosition());
        data_position_motor2.push_back(m2.GetPosition());
        data_position_motor3.push_back(m3.GetPosition());
        data_velocity_motor1.push_back(0);
        data_velocity_motor2.push_back(0);
        data_velocity_motor3.push_back(0);
        data_control_signal_roll_velocity.push_back(0);
        data_control_signal_pitch_velocity.push_back(0);
        data_er.push_back(0);
        data_ep.push_back(0);
        dR.push_back(0);
        pitch_from_res.push_back(0);

        cout << "***************" << endl;
        Ts.WaitSamplingTime();
        index_preload = index;
    }

    cout << "----HOMING MOTORS----" << endl;

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

    cout << "----PRELOAD COMPLETED----" << endl;




    //--Control---------------------------------------------------------
    cout << "----TEST STARTED----" << endl;
    //set velocity mode (acceleration (rads/s^2))
    m1.Setup_Velocity_Mode(20);
    m2.Setup_Velocity_Mode(20);
    m3.Setup_Velocity_Mode(20);

    double r0 = data_resistance.back();
    cout << "R0: " << r0 << endl;

    for (double t=0;t<(size)*dts; t+=dts)
    {
        int index = static_cast<int>(t / dts);

        misensor.GetPitchRollYaw(rpy[1],rpy[0],rpy[2]);  //reading from IMU

        dR.push_back((data_resistance.back()-r0)/r0);

        pitch_from_res.push_back(switchingModel(20, data_resistance, timestamps_multimeter, r0));

        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(now - start_time).count();
        timestamps_main.push_back(elapsed_time);


        rpy[1]=-rpy[1]; //Pitch angle opposite direction

        data_rpy[index+index_preload][0]=rpy[1];  //saving IMU data
        data_rpy[index+index_preload][1]=rpy[0];
        data_rpy[index+index_preload][2]=rpy[2];

        //data_time[index] = t;


        //Computing pitch error
        ep = target_rpy[index][1] - pitch_from_res.back();


        //Controller computes control signal FPD
        cs[0] = 0;
        cs[1] = 0.2*(ep > Cp);

        //Converting motor Velocity
        currentTarget = {cs[0],cs[1],arc_length};
        Soft_Neck.SetRollPitchArcLen(currentTarget);
        tendonLengths = Soft_Neck.GetTendons();

        //Setting Velocity
        m1.SetVelocity(-tendonLengths[0]/rm);
        m2.SetVelocity(-tendonLengths[1]/rm);
        m3.SetVelocity(-tendonLengths[2]/rm);


        //Saving data
        roll.pushBack(rpy[0]);
        pitch.pushBack(rpy[1]);

        data_er.push_back(er);
        data_ep.push_back(ep);
        data_position_motor1.push_back(m1.GetPosition());
        data_position_motor2.push_back(m2.GetPosition());
        data_position_motor3.push_back(m3.GetPosition());

        data_control_signal_roll_velocity.push_back(cs[0]);
        data_control_signal_pitch_velocity.push_back(cs[1]);

        data_velocity_motor1.push_back(-tendonLengths[0]/rm);
        data_velocity_motor2.push_back(-tendonLengths[1]/rm);
        data_velocity_motor3.push_back(-tendonLengths[2]/rm);


        cout << "^^^^^^^^^^^^^^^" << endl;
        Ts.WaitSamplingTime();
    }

    misensor.Reset();

    cout << "----TEST COMPLETED----" << endl;

    cout << "----HOMING MOTORS----" << endl;

    m1.SetupPositionMode();
    m2.SetupPositionMode();
    m3.SetupPositionMode();

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

    running = false;
    multimeterThread.join();
    sleep(2);


    //--Printing Saved Data-----------------------------------------
    for(double index=0; index<size_preload; index++) {
        resistance_plot.pushBack(data_resistance[index]);
    }

    roll.Plot();
    pitch.Plot();
    resistance_plot.Plot();

    // for(double index=0; index<size; index++)
    // {
    //     int index = static_cast<int>(t / dts);
    //     cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << data_time[index] << " Time t: " << index*dts << " RES: " << data_resistance[index] << endl;
    // }


    cout << endl << "\nTimestamps del ciclo principale:\n";
    for(double index=0; index<size_preload; index++) {
        cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << timestamps_main[index] << " s" <<endl;
    }

    cout << endl << "\nTimestamps del thread del multimetro:\n";
    for(double index=0; index<size_preload; index++) {
        cout << " RES: " << data_resistance[index] << " Time: " << timestamps_multimeter[index] << " s" <<endl;
    }



    //--Export Data as CSV
    export2DVectorToCSV(data_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/IMU_rpy_data.csv");

    export2DVectorToCSV(preload_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/preload_rpy_input_data.csv");
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

    exportVectorToCSV(data_er, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_er.csv");
    exportVectorToCSV(data_ep, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/data_ep.csv");

    exportVectorToCSV(pitch_from_res, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/pitch_from_res.csv");
    exportVectorToCSV(dR, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/dR.csv");

    sleep(2);
}
