#include <iostream>
#include <vector>


int main() {
    
// //RAMPA--------------------------------------------------------------------------
//     int interval1 = 0;       // initial time
//     double amplitude = 0.8;  // Pitch final value in rad (0 to finalValue)
//     double rate = 0.3;   // freq rad/s
//     double dt = 0.1;    // time step

//     double period = 1/rate;
    
//     int numSteps = static_cast<int>(period / dt) + 1; // +1 to include t = 0
//     std::vector<double> Pitch(numSteps); 
    
//     double t;

//     for(int i = 0; i < numSteps; i++) {
//         t = i * dt;
//         Pitch[i] = amplitude * rate * t;
//     }




 //DOPPIA RAMPA------------------------------------------------------------------
//     int interval1 = 0;       // initial time
     double amplitude = 0.8;  // Pitch final value in rad (0 to finalValue)
     double rate = 0.3;   // freq rad/s
     double dt = 0.1;    // time step

     double period = 2 * 1/rate;
    
     int numSteps = static_cast<int>(period / dt) + 1; // +1 to include t = 0
     std::vector<double> Pitch(numSteps);
    
     double t;
     int i;
     //rampa (crescente)
     for(i = 0; i < numSteps/2; i++) {
         t = i * dt;
         Pitch[i] = amplitude * rate * t;
     }
     //rampa (decrescente)
     for(i = numSteps/2; i < numSteps; i++) {
         t = i * dt;
         Pitch[i] = amplitude - amplitude * rate * (t-period/2);
     }



////CHIRP 5 CYCLES------------------------------------------------------------------
//    double dt = 0.1;    // time step
//    int nCycles = 5;
//    int interval1 = 0;       // initial time
//    double amplitude = 0.8;  // Pitch final value in rad (0 to finalValue)
//    double rate = 0.4;   // freq rad/s

//    double period = 2 * 1/rate;
    
//    int numSteps = static_cast<int>(period * nCycles/ dt) + 1; // +1 to include t = 0
//    std::vector<double> Pitch(numSteps);
//    double t;
//    int i;
//    for(int cycle = 0; cycle < nCycles; cycle++) {
//        //rampa (crescente)
//        for(i = (0 + cycle * numSteps)/nCycles; i < (numSteps/2 + cycle * numSteps)/nCycles; i++) {
//            t = (i-(cycle * numSteps/nCycles)) * dt;
//            Pitch[i] = amplitude * rate * t;   // y = A*(mx)
//        }
//        //rampa (decrescente)
//        for(i = (numSteps/2 + cycle * numSteps)/nCycles; i < (numSteps + cycle * numSteps)/nCycles; i++) {
//            t = (i-(cycle * numSteps/nCycles)) * dt;
//            Pitch[i] = amplitude - amplitude * rate * (t - period/2);   // y = A*(-mx) + q; q=A
//        }
//    }



//PRINT---------------------------------------------------------------------------
    std::string printing = {"-----------"}; 
    std::cout << "steps:" << numSteps << std::endl;
    for (int i = 0; i < numSteps; ++i) {
        printing = "-----------";
        printing[static_cast<int>(Pitch[i]*10)] = '0';
        std::cout << printing << std::endl;
        
    }


    for (int i = 0; i < numSteps; ++i) {
        std::cout << Pitch[i] << std::endl;
        
    }
//--------------------------------------------------------------------------------







}
