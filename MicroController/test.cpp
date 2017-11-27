#include <cstdio>
#include <cstdlib>
#include <vector>
#include "Spline/spline.h"

int main(int argc, char** argv) {

    std::vector<double> X(4), Y(4);
    X[0] = 0.0; X[1]= 0.125;    X[2]= 0.25;
    X[3]= 0.325;    X[4]= 0.5;

    Y[0] =  0.0; Y[1] = -0.1; Y[1] = 0.1;
    Y[2] = -0.1; Y[3] =  0.1;

    tk::spline s;
    s.set_points(X,Y);    // currently it is required that X is already sorted
   double x=0.001;

   printf("spline at %f is %f\n", x, s(x));

   return EXIT_SUCCESS;
}
