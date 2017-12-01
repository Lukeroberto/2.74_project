#include <cstdio>
#include <cstdlib>
#include <vector>
#include "Spline/spline.h"
#include <unistd.h>

int main(int argc, char** argv) {
    
    int profile_size = 3;

    std::vector<double> X(4), Y(4);
    X[0] = 0.0; X[1]= 0.125;    X[2]= 0.25;
    X[3]= 0.325;    X[4]= 0.5;

    Y[0] =  0.0; Y[1] = -0.1; Y[1] = 0.1;
    Y[2] = -0.1; Y[3] =  0.1;

    tk::spline s;
    s.set_points(X,Y);    // currently it is required that X is already sorted
   

    float dt = 0.001; 
    float dt_ctrl = X[4]/profile_size; // time length of control step
    int n_loops = dt_ctrl/dt;
    
    int torque_increment = 0;
    int time_increment = 0;  
 
    while(torque_increment < profile_size)
    {
        
        printf("spline at %f is %f\n", torque_increment*dt_ctrl, s(torque_increment*dt_ctrl));
        usleep(3000);

        if((time_increment % n_loops) == 0){torque_increment++;};
        time_increment++;
        
        printf("Torque increment: %i\n", torque_increment);
        printf("Time increment: %i\n\n" , time_increment);

    }
    printf("Length of control step for %i steps: %f\n", profile_size, dt_ctrl);
    printf("Number of loops: %i\n", n_loops);

    return EXIT_SUCCESS;
}
