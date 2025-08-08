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

//ARDUINO
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

// CSV file creation
#include <fstream>
#include <sstream>



//--Declaring global variables----------------------------------
std::vector<double> timestamps_main;
vector<double> rpy = {0.01,0.01,0};

//Functions-----------------------------------------------------
double gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    double res = a;
    return res;
}


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


// EXPORT CSV FILE FUNCTIONS------------------------------------
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

void referenceCyclicRampOffset(std::vector<double>& angle, double dt, double amplitude, double offset, double rate, int nCycles){
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
            ramp[i] = amplitude * rate * t + offset;   // y = A*(mx)
        }
        //rampa (decrescente)
        for(i = (numSteps/2 + cycle * numSteps)/nCycles; i < (numSteps + cycle * numSteps)/nCycles; i++) {
            t = (i-(cycle * numSteps/nCycles)) * dt;
            ramp[i] = amplitude - amplitude * rate * (t - period/2) + offset;   // y = A*(-mx) + q; q=A
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

bool is_valid_number(const std::string& str) {
    try {
        std::stod(str);  // try convert string to double
        return true;  // completed conversion
    } catch (const std::invalid_argument& e) {
        return false;  // failed conversion
    } catch (const std::out_of_range& e) {
        return false;  // number outside of valid range for double type
    }
}

double read_arduino(int fd){
    char string[255];  //buffer for memorizing read data from Arduino
    std::string string_res;
    int n;
    double data;

    n = read(fd, string, sizeof(string));

    string_res = string;

    //find first and second \n
    size_t first_nl = string_res.find('\n');
    size_t second_nl = string_res.find('\n', first_nl + 1);

    //extract the part of the string between \n
    string_res = string_res.substr(first_nl + 1, second_nl - first_nl - 1);

    if (n > 0) {
        if (is_valid_number(string_res)) {
                data = std::stod(string_res); // string to double conversion
                return data;
        }
    }
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


    //--ARDUINO---------------------------------------------------
    std::cout << "Connecting Arduino..." << std::endl;

    const char* port = "/dev/ttyACM0"; // La porta seriale su Linux (verifica che sia giusta)
    int baud_rate = B115200; // Assicurati che il baud rate sia lo stesso di quello impostato su Arduino

    // Apre la porta seriale
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Errore nell'aprire la porta seriale!" << std::endl;
        return 1;
    }

    // Imposta i parametri della porta seriale
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);  // Abilita la lettura e ignora il controllo del modem
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;  // Imposta 8 bit per byte
    options.c_cflag &= ~PARENB;  // Nessuna parità
    options.c_cflag &= ~CSTOPB;  // Un solo bit di stop
    tcsetattr(fd, TCSANOW, &options);

    std::cout << "Arduino Connected" << std::endl;


    //--Controlling Blocks---------------------
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);
    double gainP = 5;
    double gainI = 2;
    FPDBlock Cp(gainP*2.5773,gainI*3.2325,-0.85);   //original: Cp(2.5773,3.2325,-0.85);
    FPDBlock Cr(gainP*2.6299,gainI*3.2395,-0.86);   //original: Cr(2.6299,3.2395,-0.86);


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
    vector<double> cs(3); //control signals
    double ep=0,er=0;

    //--Plots----------------------------------
    IPlot roll(dts, "roll", "xLabel", "yLabel");
    IPlot pitch(dts, "pitch", "xLabel", "yLabel");
    IPlot res_plot(dts, "res", "xLabel", "yLabel");
    IPlot pitch_IN(dts, "input", "xLabel", "yLabel");

    //--Reference Signal Creation (rpy target)---------
    std::vector<double> target_pitch;
    std::vector<double> target_roll;


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



//    std::vector<double> target0;
//    referenceStep(target0, dts, -0.174533, 4.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());


////CHIRP.....................................
//    std::vector<double> target0;
//    referenceStep(target0, dts, 0.174533, 1.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());
//    std::vector<double> target1;
//    referenceChirp(target1, dts, 0.174533, 0, 0.05, 0.2, 50); //CHIRP ->(std::vector<double>& angle, double dt, double amplitude, double offset, double freq0, double freqF, double duration)
//    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());

//RAMPS.....................................
    std::vector<double> target1;
    referenceStep(target1, dts, 0.174533, 2.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());
    std::vector<double> target0;
    referenceCyclicRampOffset(target0, dts, 0.349066, -0.174533, 0.174533, 6);
    target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());

////STEPS.....................................
//    std::vector<double> target0;
//    referenceStep(target0, dts, 0.0, 2.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());
//    std::vector<double> target1;
//    referenceStep(target1, dts, 0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());
//    std::vector<double> target2;
//    referenceStep(target2, dts, -0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target2.begin(), target2.end());
//    std::vector<double> target3;
//    referenceStep(target3, dts, 0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target3.begin(), target3.end());
//    std::vector<double> target4;
//    referenceStep(target4, dts, -0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target4.begin(), target4.end());
//    std::vector<double> target5;
//    referenceStep(target5, dts, 0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target5.begin(), target5.end());
//    std::vector<double> target6;
//    referenceStep(target6, dts, -0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target6.begin(), target6.end());
//    std::vector<double> target7;
//    referenceStep(target7, dts, 0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target7.begin(), target7.end());
//    std::vector<double> target8;
//    referenceStep(target8, dts, -0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target8.begin(), target8.end());
//    std::vector<double> target9;
//    referenceStep(target9, dts, 0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target9.begin(), target9.end());
//    std::vector<double> target10;
//    referenceStep(target10, dts, -0.174533, 3.0); //STEP->(std::vector<double>& pitch, double dt, double amplitude, double duration)
//    target_pitch.insert(target_pitch.end(), target10.begin(), target10.end());


//    //20 degrees at 5deg/s
//    std::vector<double> target3;
//    referenceCyclicRamp(target3, dts, 0.523599, 0.087267, 10);
//    target_pitch.insert(target_pitch.end(), target3.begin(), target3.end());



    //0.785398 = 45 (rad=deg)
    //0.523599 = 30
    //0.349066 = 20
    //0.174533 = 10

//    std::vector<double> target;
//    referenceCyclicRamp(target, dts, 0.785398, 0.05, 3);
//    target_pitch.insert(target_pitch.end(), target.begin(), target.end());

//    std::vector<double> target1;
//    referenceCyclicRamp(target1, dts, 0.785398, 0.1, 3);
//    target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());


//    std::vector<double> target2;
//    referenceCyclicRamp(target2, dts, 0.785398, 0.2, 3);
//    target_pitch.insert(target_pitch.end(), target2.begin(), target2.end());


//    std::vector<double> target3;
//    referenceCyclicRamp(target3, dts, 0.785398, 0.4, 3);
//    target_pitch.insert(target_pitch.end(), target3.begin(), target3.end());

    //*****************************************

    //Creating variables to save data
    int size = target_pitch.size();
    cout << "Size: " << size << endl;

    std::vector<std::vector<double>> target_rpy(size, std::vector<double>(3));
    std::vector<std::vector<double>> data_rpy(size, std::vector<double>(3));



    double data_res = 0;
    std::vector<double> data_resistance;
    std::vector<double> data_time;

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


    //Assigning the roll pitch yaw values
    for(int index=0; index<size; index++){

        target_rpy[index][0]=0 ;
        target_rpy[index][1]=target_pitch[index];
        target_rpy[index][2]=0.0;

        cout << "TARGET Roll: " << target_rpy[index][0] << " Pitch: " << target_rpy[index][1]  << " Yaw: " << target_rpy[index][2] << " Time: " << index*dts << endl;
    }


    cout << "--------" << endl;

    for(int i=0; i<size; i++){
        cout << "TARGET Roll: " << target_rpy[i][0] << " Pitch: " << target_rpy[i][1]  << " Yaw: " << target_rpy[i][2] << " Time: " << i*dts << endl;
    }


    //--Kinematics initialization---------------------------------------
    SoftCC3Tendon Soft_Neck(0.114,0.05);    //0.113m=11.3cm: height of the neck; 0.05m=5cm: distance from center to tendons in a limb cross section.

    double arc_length = 0.114;
    double rm = 0.0075;     //motors winch radius
    vector<double> currentTarget = {0.0, 0.0,arc_length};  // {roll,pitch,arc_length}
    Soft_Neck.SetRollPitchArcLen(currentTarget);

    vector<double> tendonLengths = Soft_Neck.GetTendons();
    cout << " tendon 1 : " <<tendonLengths[0] << " tendon 2 : " << tendonLengths[1] << " tendon 3 : " <<tendonLengths[2] << endl;



    //--TIMES-----------------------------------------------
    auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto then = std::chrono::steady_clock::now();
    double elapsed_time = 0;
    cout << "----TEST STARTED----" << endl;

    //--Control---------------------------------------
    for (int index=0;index<size; index++)
    {
        misensor.GetPitchRollYaw(rpy[1],rpy[0],rpy[2]);  //reading from IMU

        data_res = read_arduino(fd);    //reading from Arduino
        data_resistance.push_back(data_res);
        cout << "RES: " << data_res <<endl;

        now = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration<double>(now - start_time).count();
        timestamps_main.push_back(elapsed_time);


        rpy[1]=-rpy[1]; //Pitch angle opposite direction

        data_rpy[index][0]=rpy[0];  //saving IMU data
        data_rpy[index][1]=rpy[1];
        data_rpy[index][2]=rpy[2];


        //controller computes control signal FPD
        cs[0] = target_rpy[index][0];
        cs[1] = target_rpy[index][1];


        //plotting data
        roll.pushBack(rpy[0]);
        pitch.pushBack(rpy[1]);
        res_plot.pushBack(data_res);

        //Converting motor Velocity
        currentTarget = {cs[0],cs[1],arc_length};
        Soft_Neck.SetRollPitchArcLen(currentTarget);
        tendonLengths = Soft_Neck.GetTendons();

        //Setting Velocity
        m1.SetVelocity(-tendonLengths[0]/rm);
        m2.SetVelocity(-tendonLengths[1]/rm);
        m3.SetVelocity(-tendonLengths[2]/rm);



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

    sleep(2);

    //--Printing Saved Data-----------------------------
    roll.Plot();
    pitch.Plot();
    res_plot.Plot();

    // for(double index=0; index<size; index++)
    // {
    //     int index = static_cast<int>(t / dts);
    //     cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << data_time[index] << " Time t: " << index*dts << " RES: " << data_resistance[index] << endl;
    // }


    cout << endl << "\nTimestamps del ciclo principale:\n";
    for(double index=0; index<size; index++) {
        cout << "Roll: " << data_rpy[index][0] << " Pitch: " << data_rpy[index][1]  << " Yaw: " << data_rpy[index][2] << " Time: " << timestamps_main[index] << " s" <<endl;
    }

    cout << endl << "\nTimestamps del thread del multimetro:\n";
    for(double index=0; index<size; index++) {
        cout << " RES: " << data_resistance[index] << " Time: " << timestamps_main[index] << " s" <<endl;
    }



    //--Export Data as CSV
    export2DVectorToCSV(data_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/IMU_rpy_data.csv");
    export2DVectorToCSV(target_rpy, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/target_rpy_input_data.csv");
    exportVectorToCSV(timestamps_main, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/IMU_rpy_time_data.csv");

    exportVectorToCSV(data_resistance, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/resistance_data.csv");
    exportVectorToCSV(timestamps_main, "/home/humasoft/Soft_neck_Soft_sensor/CSVdata/resistance_time_data.csv");


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


    sleep(2);
}
