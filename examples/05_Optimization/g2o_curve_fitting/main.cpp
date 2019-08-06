#include<iostream>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>

using namespace std; 

//
// the template of the vertex
//
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() {
        _estimate << 0,0,0;
    }
    
    virtual void oplusImpl(const double* update) {
        _estimate += Eigen::Vector3d(update);
    }
    
    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
};

//
// the template of the edge
//
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double _x;  // _x => x， _measurement => y

    CurveFittingEdge(double x): BaseUnaryEdge(), _x(x) {}

    void computeError() {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double prediction = std::exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
        _error(0,0) = _measurement - prediction;
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
};

int main(int argc, char** argv) {
    //
    // set the parameters of the equation and try to fit it 
    //
    double a = 1.0, b = 2.0, c = 1.0;   // parameters
    int N = 100;                        // data num
    double w_sigma = 1.0;               // noise
    cv::RNG rng;                        // OpenV random number generator
    double abc[3] = {0,0,0};            // estimation of abc

    // data = curve + noise
    vector<double> x_data, y_data;
    cout << "generating data: " << endl;
    for (int i=0; i<N; i++) {
        double x = i/100.0;
        x_data.push_back (x);
        y_data.push_back (
            exp(a*x*x + b*x + c) + rng.gaussian(w_sigma)
        );
        printf("x: %lf, y: %lf\n", x_data[i], y_data[i]);
    }
    
    //
    // construct the computational graph which is 
    // similar to the graph in TensorFlow
    //
    const int POSE_DIM = 3;
    const int LANDMARK_DIM = 1;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<POSE_DIM, LANDMARK_DIM>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));

    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(unique_ptr<Block>(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    
    //
    // add vertex (veriable) and edge (error) to the graph
    //
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0,0,0));
    v->setId(0);
    optimizer.addVertex(v);
    
    for (int i=0; i<N; i++) {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma)); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge(edge);
    }
    
    //
    // start optimization
    //
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds.\n" << endl;
    
    // display the result
    Eigen::Vector3d abc_estimate = v->estimate();
    printf("Real      a: %lf, b: %lf, c: %lf\n", a, b, c);
    printf("Estimated a: %lf, b: %lf, c: %lf\n", 
            abc_estimate.transpose()[0], 
            abc_estimate.transpose()[1], 
            abc_estimate.transpose()[2]
        );
    
    return 0;
}