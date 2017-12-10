#ifndef MPC_H
#define MPC_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using CppAD::AD;
using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // Motion model calculations, return state
  Eigen::VectorXd MotionModel(double x, double y, double psi, double v, double cte, double epsi, double delta, double a, double dt);

  // Evaluate a polynomial.
  double Polyeval(VectorXd coeffs, double x);

  // Evaluate a polynomial.
  AD<double> Polyeval(VectorXd coeffs, AD<double> x);

  // Evaluate derivative of a polynomial.
  double Polyderiv(VectorXd coeffs, double x);

  // Evaluate derivative of a polynomial.
  AD<double> Polyderiv(VectorXd coeffs, AD<double> x);

  // Fit a polynomial.
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  VectorXd Polyfit(VectorXd xvals, VectorXd yvals,int order);

  // Transform waypoints to vehicle coordinates
  void TransformWayPointsToVehicleCoordinates(VectorXd &ptsx, VectorXd &ptsy, double px, double py, double psi);

  // Convert std::vector<double> to Eigen::VectorXd
  VectorXd ConvertVector(vector<double> &vector);

  // Convert Eigen::VectorXd to std::vector<double>
  vector<double> ConvertVector(VectorXd &vector);
};

#endif /* MPC_H */
