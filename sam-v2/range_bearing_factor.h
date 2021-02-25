#ifndef SAM_RANGE_BEARING_FACTOR_H_
#define SAM_RANGE_BEARING_FACTOR_H_

#include "sam_slam_back_end/factor.h"


// TODO: Disambiguate Variable ordering convention: 
//       Measure : r, phi
//       State: d_x,d_y,d_th,d_lx,d_ly     


namespace Factor
{

#define RANGEBEARING_MES_SIZE 2U
#define RANGEBEARING_STATE_SIZE 5U
#define RANGEBEARING_STATE_CARD 2U   // number of (grouped) variables
#define RANGEBEARING_IDX_STATES {0U,2U}    // TODO  compile time ASSERT pour verifier si ces lignes sont coherentes
#define RANGEBEARING_SIZE_SUBSTATES {2U, 3U} // size of substates

// TODO Tag each occurence of an eigen-involving line for later
typedef Eigen::Matrix<double, RANGEBEARING_MES_SIZE, 1> MesRangeBearing_t ;
typedef Eigen::Matrix<double, RANGEBEARING_MES_SIZE,RANGEBEARING_MES_SIZE> CovMesRangeBearing_t ;
typedef Eigen::Matrix<double, RANGEBEARING_STATE_SIZE, 1> StateRangeBearing_t ;
typedef Eigen::Matrix<double, RANGEBEARING_MES_SIZE, RANGEBEARING_STATE_SIZE> JacobiRangeBearing_t;
typedef std::array<std::string, RANGEBEARING_STATE_CARD> RangeBearingVarsStr_t;



class FactorRangeBearing : public FactorBase<FactorRangeBearing
                                            , MesRangeBearing_t
                                            , CovMesRangeBearing_t
                                            , StateRangeBearing_t
                                            , JacobiRangeBearing_t
                                            , RangeBearingVarsStr_t> 
{
public:
    typedef FactorBase  <FactorRangeBearing
                        , MesRangeBearing_t
                        , CovMesRangeBearing_t
                        , StateRangeBearing_t
                        , JacobiRangeBearing_t
                        , RangeBearingVarsStr_t> FactorRangeBearingBase_t;
                    
    FactorRangeBearing(const RangeBearingVarsStr_t & vars, const Eigen::Vector2d &measure, const Eigen::Matrix2d &measure_covariance)
    :   FactorRangeBearingBase_t(vars,measure,measure_covariance)
    {

    }

    // some constants/meta info about the factor class in order help the bookkeeping
    const int16_t size_mes = RANGEBEARING_MES_SIZE ;
    const int16_t size_state = RANGEBEARING_STATE_SIZE ;
    const int16_t idx_states[RANGEBEARING_STATE_CARD] = RANGEBEARING_IDX_STATES ;
    const int16_t size_sub_states[RANGEBEARING_STATE_CARD] = RANGEBEARING_SIZE_SUBSTATES;

private:
    // h(x^0)   WARNING != bi               IF linear =>  H*Xi0
    MesRangeBearing_t compute_expected_measure(const StateRangeBearing_t & linearization_pt) const
    {
            double x = linearization_pt(0);  // TODO: do it more elegantly
            double y = linearization_pt(1);
            double th = linearization_pt(2);
            double lx = linearization_pt(3);
            double ly = linearization_pt(4);

            //r, phi  
            MesRangeBearing_t expected_mes;
            expected_mes << sqrt( (lx-x)*(lx-x)  + (ly-y)*(ly-y) ), EcPi(atan2(ly-y,lx-x)-th) ; 

            return expected_mes;
             
    }

    // Ai = rho*Jacobian(h0)                IF linear =>  rho*H
    JacobiRangeBearing_t compute_normalized_jacobian(const StateRangeBearing_t & linearization_pt) const
    {
            // TODO
            //1 compute the jacobian
            double z_r = measure_(0);
            double z_phi = measure_(1);
            double x = linearization_pt(0);
            double y = linearization_pt(1);
            double th = linearization_pt(2);
            double lx = linearization_pt(3);
            double ly = linearization_pt(4);
            double q = (x - lx)*(x - lx) + (y - ly)*(y - ly);
            double sq = sqrt(q);

            JacobiRangeBearing_t jacobian;
            jacobian <<
            (x-lx)/sq,  (y-ly)/sq,  0   ,   (lx-x)/sq   ,   (ly-y)/sq,
            (ly-y)/q ,  (x-lx)/q , -1   ,   (y-ly)/q    ,   (lx-x)/q    ;  // TAG EIGEN

            // 2 apply sqrt measure pre multiplication and return
            return sqrt_measure_covariance_*jacobian;


    }
};

};

#endif