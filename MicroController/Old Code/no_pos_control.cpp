#include <vector>

#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "spline.h"

#define PI 3.14159
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))

#define NUM_INPUTS 8
#define NUM_OUTPUTS 6

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
            
            float R            = input_params[0]; // Resistance ohms
            float k_b          = input_params[1]; // Back EMF constant           
            float profile_size = input_params[2]; // Length of profile vec 
            float ctrl_tf      = input_params[3]; // Control Time length
            int   toggle       = input_params[4]; // Pos control toggle
            
            // Length of control loop
            float dt = 0.001;
            
            // Generate torque spline
            std::vector<double> X(4), Y(4);
            
            float intvl = ctrl_tf/3.0;
            printf("Interval size: %f", intvl);
            
            Y[0]=input_params[4]; Y[1] = input_params[5]; 
            Y[2]=input_params[6]; Y[3] = input_params[7];
            
            for(int i = 0; i < 4; i++)
            {    
                X[i] = intvl*i;
                printf("(X,Y): (%f, %f)\n\r", X[i], Y[i]);
            };
            
            
            tk::spline s;
            s.set_points(X,Y);    // currently it is required that X is already sorted
            
            printf("\nDebugging...\n\r");
            printf("Looping through spline:\n\n\r");
            
            for(int i = 0; i < X.size(); i++)
            {
                printf("Value of (s,y) at %f: (%f, %f)\n\r", X[i], s(X[i]), Y[i]);
            };
            
            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            motorFwd = 1;
            motorRev = 0;
            motorPWM.write(0);
            
            float dt_ctrl = X[3]/profile_size; // time length of control step
            float n_loops = dt_ctrl/dt;
            
            //float ierr = 0;
            
            // Run experiment
            int torque_increment = 0; // Torque profile increment
            int time_increment   = 0; // Time profile increment
            
            while( torque_increment < profile_size) { 
                // Perform control loop logic
                
                // Calculate Values
                float K_p = 1;
                float current = 36.7*current_in - 18.3;
                float vel = encoder.getVelocity()*(360.0f/1200.0f)*(PI/180);
                float pos = encoder.getPulses()*(360.0f/1200.0f)*(PI/180);
                                
                // float err_pos = pos_d - pos;
        
                // Desired torque = -K*theta + b*dtheta
                float desired_torque = s(torque_increment*dt_ctrl);
                
                float current_d = desired_torque/k_b;
                //printf("Desired Current at time step %f: %f \n\r", torque_increment*dt_ctrl, s(torque_increment*dt_ctrl)/k_b);
                
                // Calculate Errors
                float err = current_d - current;
                
                // Calculate output
                float u = R*current_d + k_b*vel;// + K_p*err;
                //printf("Commanded Outpur: %f\n\r", u);
                
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
                output_data[1] = 12*v_out;
                output_data[2] = desired_torque;                
                output_data[3] = current;
                output_data[4] = pos;
                output_data[5] = vel;
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                
                // Next torque in profile
                //printf("Condition for incremet: %f\n\r", (float)(time_increment % (int)n_loops));
                if((time_increment % (int)n_loops) == 0){torque_increment++;};
                time_increment++;
                
                //printf("(N loops, dt_ctrl): %f, %f\n\r", n_loops, dt_ctrl);
                //printf("Time increment: %i\n\r", time_increment);
                //printf("Torque increment: %i\n\r", torque_increment);
                
                wait(dt); 
            }     
            // Cleanup after experiment
            server.setExperimentComplete();
            motorPWM.write(0);
        } // end if
    } // end while
} // end main
