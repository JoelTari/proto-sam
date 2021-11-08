#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "factor.h"

//------------------------------------------------------------------//
//                        Anchor (2d) factor                        //
//------------------------------------------------------------------//
/**
 * @brief anchor (2d) factor implementation. Typically used for prior or gps on
 * 2d (R^2) variables
 */
static constexpr char                   kAnchorCategoryName[] = "anchor";
constexpr int                           kAnchorNbVar          = 1;
constexpr int                           kAnchorXDimTot        = 2;
constexpr std::array<int, kAnchorNbVar> kAnchorVarSizes       = {2};
constexpr int                           kAnchorMesDim         = 2;
using AnchorMeta_t = FactorMetaInfo<kAnchorNbVar,
                                    kAnchorXDimTot,
                                    kAnchorVarSizes,
                                    kAnchorMesDim>;

/**
 * @brief Anchor factor class
 */
class AnchorFactor
    : public BaseFactor<AnchorFactor, AnchorMeta_t, kAnchorCategoryName>
{
  public:
  AnchorFactor(const std::string&              factor_id,
               const AnchorFactor::var_keys_t& var_names)
      : BaseFactor<AnchorFactor, AnchorMeta_t, kAnchorCategoryName>(factor_id,
                                                                    var_names)
  {
  }

  // void performActionImpl()
  // {
  //   std::cout << "hello factor";
  // }
};

#endif
