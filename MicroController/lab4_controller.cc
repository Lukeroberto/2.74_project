#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"

#define PI 3.14159
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))

#define NUM_INPUTS 8
#define NUM_OUTPUTS 5

AnalogIn current_in(A0);

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
PwmOut motorPWM(D5);        // Motor PWM output
DigitalOut motorFwd(D6);    // Motor forward enable
DigitalOut motorRev(D7);    // Motor backward enable
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoder(D3,D4, NC, 1200 , QEI::X4_ENCODING); // Pins D0, D1, no index, 1200 counts/rev, Quadrature encoding

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();

    // PWM period should nominally be a multiple of our control loop
    motorPWM.period_us(100);

    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];

    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            float K_p           = input_params[0]; // Proportional Gain
            float time_loop   = input_params[1]; // Time for the loop
            float R           = input_params[2]; // Resistance ohms
            float k_b         = input_params[3]; // Back EMF constant
            float K           = input_params[4];
            float b           = input_params[5]; // damping factor
            float pos_d       = input_params[6];
            float optional    = input_params[7];
            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            motorFwd = 1;
            motorRev = 0;
            motorPWM.write(0);

            //float ierr = 0;

            // Run experiment
            while( t.read() <  time_loop) {
                // Perform control loop logic

                // Calculate Values
                float current = 36.7*current_in - 18.3;
                float vel = encoder.getVelocity()*(360.0f/1200.0f)*(PI/180);
                float pos = encoder.getPulses()*(360.0f/1200.0f)*(PI/180);

                float err_pos = pos_d - pos;

                // Desired torque = -K*theta + b*dtheta
                float current_d = (1/k_b)*(-K*err_pos - b*vel);

                // Calculate Errors
                float err_i = current_d - current;

                // Calculate output
                float u = R*current_d + k_b*vel + K_p*err_i + optional;  // + K_d*derr + K_i*ierr;
                float v_out = min(abs(u)/12, 1.0f);

                int direction = u/abs(u);
                if (direction == 1){
                    motorFwd = 0;
                    motorRev = 1;
                    }
                else{
                    motorFwd = 1;
                    motorRev = 0;
                }
                // Set voltage

                motorPWM.write(v_out);

                // Form output to send to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = pos;
                output_data[2] = vel;
                output_data[3] = 12*v_out;
                output_data[4] = current;
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001);
            }
            // Cleanup after experiment
            server.setExperimentComplete();
            motorPWM.write(0);
        } // end if
    } // end while
} // end main
