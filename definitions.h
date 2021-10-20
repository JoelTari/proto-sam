#ifndef DEFINITIONS_SAM_SLAM_BACK_END_H_
#define DEFINITIONS_SAM_SLAM_BACK_END_H_

#include <eigen3/Eigen/Dense> // TODO inclure des librairies sparses
#include <memory>
#include <string>
#include <vector>

struct Pose2D {
  double x  = 0;
  double y  = 0;
  double th = 0;

  Pose2D(double x, double y, double th) : x(x), y(y), th(th) {}

  Eigen::Vector3d vector() const
  {
    Eigen::Vector3d vect;
    vect << x, y, th;
    return vect;
  }
};

// struct BaseMeasure
// {
// };

// struct ControlMeasure : public BaseMeasure
// {
//   double v  ;
//   double w  ;
//   double dt ; // delta time in seconds
//   const std::string var;

//   ControlMeasure(const std::string & var,double v, double w, double dt) :
//   v(v), w(w), dt(dt), var(var)
//   {
//   }

//   Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// struct RangeBearingMeasure : public BaseMeasure
// {
//   double r = 0;
//   double phi = 0; // angle from robot direction to the object of interest
//   int id;         // identifiant
//   std::string var;

//   RangeBearingMeasure(double r, double phi, int id, const std::string& var) :
//   r(r), phi(phi), id(id), var(var)
//   {
//   }

//   Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

template <typename T>
inline T EcPi(const T & a)
{
  return atan2(sin(a), cos(a));
}

#endif
