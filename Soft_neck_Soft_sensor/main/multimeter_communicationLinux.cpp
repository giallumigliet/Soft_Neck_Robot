#include "SerialComm.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

// Funzione per configurare la porta seriale usando la classe SerialComm
SerialComm* setupSerialPort(const std::string& portName, long baudrate = 9600) {
    SerialComm* serial = new SerialComm(portName, baudrate);
    return serial;
}

// Funzione per inviare un comando
void sendCommand(SerialComm* serial, const std::string& command) {
    if (serial->WriteLine(command) <= 0) {
        std::cerr << "Error during command sending to multimeter" << std::endl;
    }
}

// Funzione per leggere i dati dal multimetro
double readMultimeterData(SerialComm* serial) {
    sendCommand(serial, "MEAS:FRESistance?");
    std::string data = serial->GetLine();
    
    if (!data.empty()) {
        std::cout << "Multimeter Data: " << data << std::endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
//    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Pause between readings
    return 0.0;
}



double readDataTaker(SerialComm* serial) {
    sendCommand(serial, "1R");
    std::string data = serial->GetLine();

    if (!data.empty()) {
        std::cout << "Datataker Data: " << data << std::endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
//    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Pause between readings
    return 0.0;
}



int main() {
    SerialComm* serial = setupSerialPort("/dev/ttyUSB1", 9600); // Port selection

    if (serial) {
        sendCommand(serial, "SYSTem:LOCal");
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause
        sendCommand(serial, "SYSTem:REMote");
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 1s pause
        readMultimeterData(serial); // Reads and prints data from multimeter
//        readDataTaker(serial);

        // Data collection with timing
        int n_datos = 10;  // Adjust this based on your data requirements
        std::vector<double> data_r(n_datos, 0.0);
        std::vector<double> T(n_datos, 0.0);

        for (int i = 0; i < n_datos; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            
            double resistance = readMultimeterData(serial);
//            double resistance = readDataTaker(serial);

            data_r[i] = resistance;

//            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Pause between reads

            auto end = std::chrono::high_resolution_clock::now();
            T[i] = std::chrono::duration<double>(end - start).count();
        }

        // Sum of all times
        double tFor = 0.0;
        for (const auto& time : T) {
            tFor += time;
        }
        std::cout << "Total time (tFor): " << tFor << " seconds" << std::endl;

        // Accumulated time calculation
        std::vector<double> time(T.size(), 0.0);
        time[0] = T[0];
        for (size_t i = 1; i < T.size(); ++i) {
            time[i] = T[i] + time[i - 1];
        }

        // Print resistance vector and time vector
        for (size_t i = 0; i < time.size(); ++i) {
            std::cout << "Resistance reading " << i + 1 << ": " << data_r[i] << " ohm" << std::endl;
        }  

        for (size_t i = 0; i < time.size(); ++i) {
            std::cout << "Accumulated time at index " << i + 1 << ": " << time[i] << " s" << std::endl;
        }  

        sendCommand(serial, "SYSTem:LOCal");
        delete serial; // Closes serial connection
    }
    return 0;
}
