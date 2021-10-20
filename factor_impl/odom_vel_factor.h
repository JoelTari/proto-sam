#ifndef SAM_VELODOM_H_
#define SAM_VELODOM_H_

#include "sam_slam_back_end/factor.h"
#include "sam_slam_back_end/definitions.h"

namespace Factor
{

#define VELODOM_MES_SIZE 3U
#define VELODOM_STATE_SIZE 6U
#define VELODOM_STATE_CARD 2U   // number of (grouped) variables
#define VELODOM_IDX_STATES {0U,3U}    // TODO  compile time ASSERT pour verifier si ces lignes sont coherentes
#define VELODOM_SIZE_SUBSTATES {3U, 3U} // size of substates

typedef Eigen::Matrix<double, VELODOM_MES_SIZE, 1> MesVelOdom_t ;
typedef Eigen::Matrix<double, VELODOM_MES_SIZE, VELODOM_MES_SIZE> CovMesVelOdom_t ;
typedef Eigen::Matrix<double, VELODOM_STATE_SIZE, 1> StateVelOdom_t ;
typedef Eigen::Matrix<double, VELODOM_MES_SIZE, VELODOM_STATE_SIZE> JacobiVelOdom_t;
typedef std::array<std::string, VELODOM_STATE_CARD> VelOdomVarsStr_t;


// this is a class now, not a class template
// variable ordering in factor: [Xi, X{i-1}]
//                             = [xi,yi,thi,x{i-1},y{i-1},th{i-1}] 
class FactorOdom : public FactorBase<FactorOdom, MesVelOdom_t,CovMesVelOdom_t,StateVelOdom_t,JacobiVelOdom_t, VelOdomVarsStr_t>
{
public:
typedef FactorBase<FactorOdom, MesVelOdom_t,CovMesVelOdom_t,StateVelOdom_t,JacobiVelOdom_t, VelOdomVarsStr_t>  FactorVelOdomBase_t;

    FactorOdom(const VelOdomVarsStr_t & vars, const MesVelOdom_t &measure, const CovMesVelOdom_t &measure_covariance, const double & dt)
    : FactorVelOdomBase_t(vars, measure,measure_covariance)
      , dt_(dt)
    {
        // TODO: runtime assert entre le nombre de variables et les membres meta
    }

    // some constants/meta info about the factor class in order help the bookkeeping
    const int16_t size_mes = VELODOM_MES_SIZE ;
    const int16_t size_state = VELODOM_STATE_SIZE ;
    const int16_t idx_states[VELODOM_STATE_CARD] = VELODOM_IDX_STATES ;
    const int16_t size_sub_states[VELODOM_STATE_CARD] = VELODOM_SIZE_SUBSTATES;

private:
    const double dt_;
    MesVelOdom_t compute_expected_hvalue_at_point(const StateVelOdom_t & linearization_pt) const
    {
        MesVelOdom_t hX0;  // h(x^0)
        
        
        return hX0;
    }

    JacobiVelOdom_t compute_normalized_jacobian(const StateVelOdom_t & linearization_pt) const
    {
            // TODO
    }
};

};

#endif