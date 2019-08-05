#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

//
// define a struct/class for the cost
//
struct CURVE_FITTING_COST {
    const double _x, _y;

    // init
    CURVE_FITTING_COST (double x, double y) : _x(x), _y(y){}

    // cost function
    template <typename T>
    bool operator() (
        const T* const abc,     // parameters of model (array)
        T* residual ) const     // residual
    {
        T predict = ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]); // exp(ax^2+bx+c)
        residual[0] = T(_y) - predict;
        return true;
    }    
};

int main ( int argc, char** argv ) {
    //
    // set the parameters of the equation and try to fit it 
    //
    double a = 1.0, b = 2.0, c = 1.0;   // parameters
    int N = 100;                        // data num
    double w_sigma = 1.0;               // noise
    cv::RNG rng;                        // OpenCV random number generator
    double abc[3] = {0, 0, 0};          // estimation of abc

    // data = curve + noise
    vector<double> x_data, y_data;
    cout << "generating data: " << endl;
    for (int i=0; i<N; i++) {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(
            exp(a*x*x + b*x + c) + rng.gaussian(w_sigma)
        );
        printf("x: %lf, y: %lf\n", x_data[i], y_data[i]);
    }

    //
    // set the problem
    //
    ceres::Problem problem;
    const int INPUT_DIM = 1;  // dim of residual[0]
    const int OUTPUT_DIM = 3; // dim of abc
    for (int i=0; i<N; i++) {
        problem.AddResidualBlock (
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, INPUT_DIM, OUTPUT_DIM> ( 
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,   // kernel function
            abc        // estimation of abc
        );
    }

    //
    // set the solver and solve the problem
    //
    ceres::Solver::Options options;                // additional options
    // options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;  // method
    options.minimizer_progress_to_stdout = true;   // display the log during optimization

    ceres::Solver::Summary summary;    
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);     // start to optimize
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // display the result
    cout << summary.BriefReport() << endl;
    printf("Real      a: %lf, b: %lf, c: %lf\n", a, b, c);
    printf("Estimated a: %lf, b: %lf, c: %lf\n", abc[0], abc[1], abc[2]);
    
    return 0;
}
