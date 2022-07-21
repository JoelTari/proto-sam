#ifndef FACTOR_H_
#define FACTOR_H_

#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <array>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <iostream>
#include <map>
#include <string_view>
#include <tuple>
#include <utility>

template <typename DerivedKCC, typename KEYMETA,size_t DimMes, const char* ContextRole,bool LinearModel=false> 
struct KeyContextualConduct : KEYMETA
{
  using KeyMeta_t = KEYMETA;
  using Key_t = typename KeyMeta_t::key_t;
  static constexpr const char*       kRole {ContextRole};
  static constexpr const std::size_t kM {DimMes};
  static constexpr const bool kLinear {LinearModel};
  // non static but const
  const std::string key_id;
  // non static, not const
  using process_matrix_t  = Eigen::Matrix<double, DimMes, KEYMETA::kN>;
  // using measure_vect_t    = Eigen::Matrix<double, DimMes, 1>;
  using measure_cov_t     = Eigen::Matrix<double, DimMes, DimMes>;
  using part_state_vect_t = Eigen::Matrix<double, KEYMETA::kN, 1>;
  // measure_vect_t   b;
  const measure_cov_t& rho;

  // FIX: ACTION: key mean view should be mean distribution
  std::shared_ptr<part_state_vect_t> key_mean_view; // TODO: make it a const ?

  process_matrix_t compute_part_A() const
  {
    // NOTE: if NL, the compute_part_A_impl must compute rho*((d part_h/ dx)|_x0) 
    // NOTE: (need the key linpoint part_x0 : key_mean_view)
    // NOTE: if Linear, it is just a constant returned value (rho*partH, where partH is static)

    // OPTIMIZE: store in a member to save some cycles (RAM vs CPU), but less readability
    return static_cast<const DerivedKCC*>(this)->compute_part_A_impl(); 
  }

    // ctor for linear systems
  KeyContextualConduct(const std::string& key_id, const measure_cov_t& rho)
      : key_id(key_id)
      , rho(rho)
  {
    // necessary (but not sufficient) condition for this ctor: the context model must be linear.
    // sufficient condition would be that the wider system be linear (enforceable at higher level)
    static_assert(kLinear);
  }

    // ctor called for non linear systems
  KeyContextualConduct
    (const std::string& key_id
    , const measure_cov_t& rho
    , std::shared_ptr<part_state_vect_t> init_point_view)
      : key_id(key_id)
      , rho(rho)
      , key_mean_view(init_point_view)
  {
    // NOTE: this can be called even if the context model is not linear 
    // NOTE: (one nl model in the wider system implies that everything is nl)
  }
};

//------------------------------------------------------------------//
//                         Factor Template                          //
//------------------------------------------------------------------//

template <typename DerivedFactor,
          const char* FactorLabel,
          typename MEASURE_META,
          typename... KeyConducts>
class BaseFactor
{
  public:

  using measure_meta_t = MEASURE_META;
  using measure_t = typename measure_meta_t::measure_t;
  static constexpr size_t      kN      = (KeyConducts::kN + ...);
  static constexpr size_t      kM      = MEASURE_META::kM;
  static constexpr size_t      kNbKeys = sizeof...(KeyConducts);
  using criterion_t = Eigen::Vector<double, kM>;
  using measure_cov_t = Eigen::Matrix<double,kM,kM>;
  using state_vector_t                 = Eigen::Matrix<double, kN, 1>;  // { dXk , ... }
  // the next 2 are probably unnecessary at Base
  using tuple_of_part_state_ptr_t          
    = std::tuple<std::shared_ptr<typename KeyConducts::part_state_vect_t> ...>;
  using tuple_of_opt_part_state_ptr_t      
    = std::tuple<std::optional<std::shared_ptr<typename KeyConducts::part_state_vect_t>> ...>;
  // remove the last two
  using composite_state_ptr_t = std::tuple<std::shared_ptr<typename KeyConducts::Key_t> ...>; // {*Xk ...}
  using composite_of_opt_state_ptr_t = std::tuple<std::optional<std::shared_ptr<typename KeyConducts::Key_t>>...>;
                                                              //  NOTE: Xk same type as dXk in euclidian factors
  using matrix_Ai_t = Eigen::Matrix<double, kM, kN>;
  using matrices_Aik_t = std::tuple<typename KeyConducts::process_matrix_t...>;
  using KeysSet_t = std::tuple<KeyConducts...>;

  static constexpr const char*                 kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName = MEASURE_META::components;
  const std::string                            factor_id;   // fill at ctor
  const measure_t                                z;           // fill at ctor
  const measure_cov_t                          z_cov;       // fill at ctor
  const measure_cov_t                          rho;         // fill at ctor
  KeysSet_t keys_set; // fill at ctor, mutable

  // methods
  static constexpr bool isLinear = (KeyConducts::kLinear && ...);

  // double norm_at_lin_point = 0; // NOT used

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // init helper
  static
  std::optional< composite_state_ptr_t >
  guess_init_key_points(composite_of_opt_state_ptr_t x_init_ptr_optional_tup, const criterion_t & z)
  {
    return DerivedFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);      
  }

  std::array<std::string, kNbKeys> get_array_keys_id() const
  {
    return sam_tuples::reduce_array_variadically(this->keys_set,
        [this]<std::size_t ...J>(const auto & keyset, std::index_sequence<J...>)
        {
          return std::array<std::string,kNbKeys>{std::get<J>(keyset).key_id ...};
        });
  }

  template <bool isSystFullyLinear>
  std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi() const 
  // TODO: pass the lin point in argument ? there could be a use case
  {
    return static_cast<const DerivedFactor*>(this)->compute_Ai_bi_impl();
  }

  criterion_t compute_r_of_x_at(const composite_state_ptr_t & X ) const
  {
    return static_cast<const DerivedFactor*>(this)->compute_r_of_x_at_impl(X);
  }

  criterion_t compute_r_of_x_at_current_lin_point() const
  {
      // build back the tup of stored lin point
      auto lin_point_tup =  sam_tuples::reduce_tuple_variadically(this->keys_set,[this]<std::size_t...J>
          (const auto & kset, std::index_sequence<J...>)
        {  
            // TODO: assert (key_mean_view != nullptr && ...);
            return std::make_tuple( *(std::get<J>(kset).key_mean_view) ... ) ;
        });
      // TODO: check if ok
      return compute_r_of_x_at(lin_point_tup);
  }

  private:

  // ctor helper
  std::map<std::string, std::size_t>
      map_keyid(const std::array<std::string, kNbKeys>& keys_id) const
  {
    std::map<std::string, std::size_t> result;
    for (int i = 0; i < keys_id.size(); i++) result[keys_id[i]] = i;
    return result;
  }

  //  func that gets the tup of lin points contained in kcm into a tuple of Key_t
  std::tuple<typename KeyConducts::Key_t ...> get_key_points() const
  {
    std::tuple<typename KeyConducts::Key_t ...> tup_mean;
    std::apply([this,&tup_mean](const auto & ...kcc)
    {
      // TODO: assert( *(kcc.key_mean_view) != nullptr && ... );
      tup_mean = {*(kcc.key_mean_view) ... };
    }
    ,this->keys_set);
    return tup_mean;
  }
  
};

// template <typename DerivedEuclidianFactor,
//           const char* FactorLabel,
//           typename MEASURE_META,
//           typename... KeyConducts>
// class EuclidianFactor : public BaseFactor<EuclidianFactor, FactorLabel, MEASURE_META, KeyConducts>
// {
//
// };

template <typename DerivedFactor,
          const char* FactorLabel,
          typename MEASURE_META,
          typename... KeyConducts>
class Factor
{
  public:
  using measure_meta_t = MEASURE_META;
  using measure_t = typename measure_meta_t::measure_t;
  using criterion_t = Eigen::Matrix<double, MEASURE_META::kM, 1>;
  using measure_cov_t  = Eigen::Matrix<double, MEASURE_META::kM, MEASURE_META::kM>;
  static constexpr const char* kFactorLabel {FactorLabel};
  static constexpr size_t      kN      = (KeyConducts::kN + ...);
  static constexpr size_t      kM      = MEASURE_META::kM;
  static constexpr size_t      kNbKeys = sizeof...(KeyConducts);
  using state_vector_t                 = Eigen::Matrix<double, kN, 1>; 
  using tuple_of_part_state_ptr_t          
    = std::tuple<std::shared_ptr<typename KeyConducts::part_state_vect_t> ...>;
  using tuple_of_opt_part_state_ptr_t      
    = std::tuple<std::optional<std::shared_ptr<typename KeyConducts::part_state_vect_t>> ...>;
  // NOTE: On state vector (or init point, or map) : no explicit state vect is kept at factor level:
  // NOTE:  we do keep it at the keys level, and offer the get_state_vector_from_tuple() 
  // NOTE:  method to query it if necessary
  using process_matrix_t               = Eigen::Matrix<double, kM, kN>;
  // make a tuple of KeySet.  Michelin *** vaut le détour.
  using KeysSet_t = std::tuple<KeyConducts...>;
  using matrices_Aik_t = std::tuple<typename KeyConducts::process_matrix_t...>;

  static constexpr const char*                 kMeasureName {MEASURE_META::kMeasureName};
  static constexpr std::array<const char*, kM> kMeasureComponentsName = MEASURE_META::components;
  const std::string                            factor_id;   // fill at ctor
  const measure_t                                z;           // fill at ctor
  const measure_cov_t                          z_cov;       // fill at ctor
  const measure_cov_t                          rho;         // fill at ctor
  const criterion_t                         rosie = rho*z; // FIX: ACTION: euclidian only
  KeysSet_t keys_set;   // a tuple of the structures of each keys (dim, id,
                        // process matrix), fill at ctor, modifiable

  static constexpr bool isLinear = (KeyConducts::kLinear && ...);

  // TODO: ACTION: static constexpr bool isEuclidianFactor ; // -> false if the factor has at least 1 non-euclidian key 

  double norm_at_lin_point = 0;

  std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

  // NOTE: tuple of optional (input)  =>  optional of tuple (output)
  // nullopt returned value indicates that the init point cannot be 
  // defined for all keys (e.g. bearing observation of a new landmark)
  static
  std::optional< tuple_of_part_state_ptr_t >
  guess_init_key_points(tuple_of_opt_part_state_ptr_t x_init_ptr_optional_tup, const criterion_t & z)
  {
    return DerivedFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);      
  }

  // ctor helper
  std::map<std::string, std::size_t>
      map_keyid(const std::array<std::string, kNbKeys>& keys_id) const
  {
    std::map<std::string, std::size_t> result;
    for (int i = 0; i < keys_id.size(); i++) result[keys_id[i]] = i;
    return result;
  }

  static constexpr bool isEuclidianFactor = true;  // TODO: ACTION:
  
  // WARNING: this is the new method !!
  template <bool isSystFullyLinear>
  std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi() const // TODO: pass the lin point in argument ? probably not
  {
    if constexpr (isEuclidianFactor)
    {
      criterion_t bi;
      if constexpr (isSystFullyLinear) {
        bi= this->rosie;
      }
      else{
        bi = this->compute_bi_nl();
      }
      matrices_Aik_t all_Aik = 
        sam_tuples::reduce_array_variadically(this->keys_set,
            [this]<std::size_t ...J>(const auto & keyset, std::index_sequence<J...>)
            {
              return std::make_tuple<matrices_Aik_t> (std::get<J>(keyset).compute_part_A() ... ) ;
              // return std::array<std::string,kNbKeys>{std::get<J>(keyset).key_id ...};
            });
    }
    else // factor involves non trivial lie group
    {
  //      // WARNING: I assume that no nonEuclidian factor is Linear, Im not sure, but couldn't find a counter example
  //      // WARNING: nontheless, let's have a static assertion here, in case Im wrong
       static_assert( !(isLinear && !isEuclidianFactor ));
       // there we can't go more in details in class template because we want to take advantage of the fact
       // that the manif library computes some elementary Jacobians that participate in computing of Aik
       return static_cast<DerivedFactor*>(this)->compute_Ai_bi_simultaneous_impl();
    }
  }

  // this uses the internally stored key_mean
  // FIX: ACTION: euclidian only
  criterion_t compute_bi_nl() const
  {
    auto  tuple_of_means = this->get_key_points();
    return this->rosie - this->rho*this->compute_h_of_x(tuple_of_means);
  }

  //  func that gets the tup of lin points contained in kcm into state_vector_t
  std::tuple<typename KeyConducts::part_state_vect_t ...> get_key_points() const // FIX: ACTION: part_state_vect is not necessarily a vect ?
  {
    // FIX: ACTION: I think I just need to remove the _vect (in KCC also)
    std::tuple<typename KeyConducts::part_state_vect_t ...> tup_mean;
    std::apply([this,&tup_mean](const auto & ...kcc) //-> std::tuple<typename KeyConducts::state_vector_t ...>
    {
      // TODO: assert( *(kcc.key_mean_view) != nullptr && ... );
      // FIX: ACTION: needs mean distribution ? I think yes !
      tup_mean = {*(kcc.key_mean_view) ... };
    }
    ,this->keys_set);
    return tup_mean;
  }

  //  func that transforms a tup of lin points contained in kcm into state_vector_t
  // template <typename... PARTIAL_STATE_VECTORS_T>
    // TODO: consider it protected
    // NOTE: not used currently ??
    // FIX: ACTION: when used, think carefully whether or not part_state_(vect)_t should be a vector or a lie manif element
  state_vector_t 
    get_state_vector_from_tuple(const std::tuple<typename KeyConducts::part_state_vect_t ...> & x_tup) const
  {
    static_assert(std::tuple_size_v<std::tuple<typename KeyConducts::part_state_vect_t ...>> == kNbKeys);
    state_vector_t x;

    std::apply([&x](auto... partx)
    {
      ((x << partx ),  ...);
        // x << (partx, ...) ;        
    }, x_tup); // TODO: check, too good to be true ??

    return x;
  }


  // FIX: ACTION: only for euclidian
  criterion_t compute_h_of_x(const state_vector_t & x) const
  {
    // class instantiation dependent
    return static_cast<const DerivedFactor*>(this)->compute_h_of_x_impl(x);
  }

  // the overloaded ctor (used in NL with init points)
  Factor(const std::string&                      factor_id,
         const criterion_t&                   z,
         const measure_cov_t&                    z_cov,
         const std::array<std::string, kNbKeys>& keys_id,
         const std::tuple<std::shared_ptr<typename KeyConducts::part_state_vect_t> ...> & tup_init_points_ptr)
      : z(z)
      , z_cov(z_cov)
      , factor_id(factor_id)
      , rho(Eigen::LLT<measure_cov_t>(z_cov.inverse()).matrixU())
      , keys_set(sam_tuples::reduce_array_variadically(
            keys_id,
            []<std::size_t... I>(const auto& my_keys_id,
                                 const auto& rho,
                                 const auto& tup_init_points_ptr,
                                 std::index_sequence<I...>) -> KeysSet_t {
              // return std::make_tuple(KeyConducts(my_keys_id[I], rho)...); // original
              // might be possible to use perfect forwarding, by declaring an empty tuple
              // and next line expanding tuple_cat with an intermediary function that has perfect forwarding ( TODO:)
              return  { KeyConducts(my_keys_id[I], rho,std::get<I>(tup_init_points_ptr)) ... } ;
            },
            rho,tup_init_points_ptr))
      , keyIdToTupleIdx(map_keyid(keys_id))
  {
    std::cout << "rho : " << this->rho
      << '\n';
    std::cout << "rosie : " << this->rosie
      << '\n';
    
  }

  std::array<std::string, kNbKeys> get_array_keys_id() const
  {
    // NOTE: reduce_tuple_variadically didn't work
    // TODO: move this logic at the factor level
    return sam_tuples::reduce_array_variadically(this->keys_set,
        [this]<std::size_t ...J>(const auto & keyset, std::index_sequence<J...>)
        {
          return std::array<std::string,kNbKeys>{std::get<J>(keyset).key_id ...};
        });
  }


  // not used for now, TODO: do the same for x_tuple input
  // FIX: ACTION: only for euclidian
  double compute_factor_norm(const state_vector_t& x) const
  {
      return (this->rho * this->compute_h_of_x(x) - this->rosie).norm();
  }

  // FIX: ACTION: only for euclidians. Maybe this one doesnt exists
  // compute the norm
  double compute_factor_norm_at_lin_point() const
  {
    if constexpr (isLinear)
    {
      // Ax := A_1 \mu_1 + A_2 \mu_2 + ... ;
      criterion_t Ax = criterion_t::Zero(); // init to 0
      sam_tuples::for_each_in_const_tuple(
          this->keys_set, [&Ax](const auto & kset,auto NIET)
          {
            // FIX: ACTION: mean distribution here
            Ax += kset.compute_part_A() * *(kset.key_mean_view);
          }
      );
      std::cout << "Ax : \n" << Ax
        << '\n';
      return (Ax - this->rosie).norm(); // FIX: ACTION:
    }
    else
    {
      // build back the tup of stored lin point
      auto lin_point_tup =  sam_tuples::reduce_tuple_variadically(this->keys_set,[this]<std::size_t...J>
          (const auto & kset, std::index_sequence<J...>)
        {  
            // TODO: assert (key_mean_view != nullptr && ...);
            // FIX: ACTION: mean distribution here
            return std::make_tuple( *(std::get<J>(kset).key_mean_view) ... ) ;
        });
      return (this->rho * this->compute_h_of_x(lin_point_tup) - this->rosie).norm(); // FIX: ACTION:
    }
  }
};


//------------------------------------------------------------------//
//            Factor History & Factors History Container            //
//------------------------------------------------------------------//
template <typename FT>
struct FactorHistory
{
  using Factor_t = FT;
  using measure_t = typename FT::measure_t;
  const std::string factor_id = "NaS";
  static constexpr const char* kFactorLabel {FT::kFactorLabel};
  static constexpr const char* kMeasureName {FT::kMeasureName};
  const std::array<std::string,FT::kNbKeys> vars_id = {};
  std::vector<double> norms;
  const measure_t z;

  // ctor
  FactorHistory(const std::string & factor_id, const std::array<std::string, FT::kNbKeys> & vars_id, const measure_t & z)
    : factor_id(factor_id), vars_id(vars_id), z(z)
  {}

  // using measure_t = Factor_t::measure_t;
  // const measure_t z;

  void push_norm_value(double factor_norm)
  {
    norms.push_back( factor_norm );
  }
};

template <typename FT,typename ... FTs>
struct FactorsHistoriesContainer
{
    using factors_histories_t
        = std::tuple<std::unordered_map<std::string, FactorHistory<FT>>,
                     std::unordered_map<std::string, FactorHistory<FTs>>...>; 

  // the data : maps of 
  // Its wrapped in a tuple because the 
  factors_histories_t factors_histories_container;

  static constexpr const std::size_t kNbFactorTypes { std::tuple_size_v<factors_histories_t>};


  template <typename Q_FT>
  void insert_new_factor_history(const std::string & factor_id, const Q_FT & factor, double factor_norm)
  {
      // static_assert(std::is_same_v<FT,Q_FT> || (std::is_same_v<FTs,Q_FT> || ...)  );
      // constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();
      //
      // auto [it, hasBeenPlaced] = std::get<I>(this->factors_histories_container)
      //   .try_emplace(factor_id, factor_id, factor.get_array_keys_id(),factor.z);
      // assert(hasBeenPlaced); // TODO: consider this a consistency check
      auto it = insert_new_factor_history<Q_FT>(factor_id,factor);

      // push the norm
      it->second.push_norm_value(factor_norm);
  }

  // overloard where 
  // TODO: no overload have the one with 3 args call the one with 2 args
  template <typename Q_FT>
  auto insert_new_factor_history(const std::string & factor_id, const Q_FT & factor)
  {
      static_assert(std::is_same_v<FT,Q_FT> || (std::is_same_v<FTs,Q_FT> || ...)  );
      constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();

      auto [it, hasBeenPlaced] = std::get<I>(this->factors_histories_container)
        .try_emplace(factor_id, factor_id, factor.get_array_keys_id(),factor.z);
      assert(hasBeenPlaced); // TODO: consider this a consistency check
      return it;
  }

  // convenience overload
  template <typename Q_FT>
  void insert_new_factor_history(const Q_FT & factor)
  {
    this->insert_new_factor_history(factor.factor_id, factor);
  }
  

  template <typename Q_FT>
  void push_data_in_factor_history(const std::string & factor_id, double factor_norm)
  {
      static_assert(std::is_same_v<FT,Q_FT> || (std::is_same_v<FTs,Q_FT> || ...)  );
      constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();
      // get marginal history ref
      auto factor_history_it = std::get<I>(this->factors_histories_container).find(factor_id);
      // TODO: assert(marginal_history_it != std::get<I>(this->marginal_history_tuple).end() );
      // push new norm
      factor_history_it->second.push_norm_value( factor_norm );
  }


  template <typename Q_FT, std::size_t I = 0>
  static constexpr std::size_t get_correct_tuple_idx_of_factor_type()
  {
    static_assert(I < kNbFactorTypes);
    if constexpr (std::is_same_v<typename std::tuple_element_t<I, factors_histories_t>::mapped_type::Factor_t, Q_FT>)
    { return I; }
    else
    {
      return get_correct_tuple_idx_of_factor_type<Q_FT,I + 1>();
    }
  }


};


//------------------------------------------------------------------//
//                      Helper print functions                      //
//------------------------------------------------------------------//
// TODO: use tuple_patterns rather
// static traverse of the KeySet tuple
template <typename TUP, size_t I = 0>
constexpr void traverse_tup()
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>();
  }
}

// runtime traverse of the KeySet tuple
template <typename TUP, size_t I = 0>
void traverse_tup(const TUP& tup)
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole
              << ". Id: " << std::get<I>(tup).key_id << '\n';
    // std::cout << "\t\t\t A:\n" << std::get<I>(tup).A << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>(tup);
  }
}

template <typename KC>
void print_KeyContextConduct(const KC& kcc)
{
  std::cout << "\t\t+ Key Nattupelemure: " << KC::kKeyName << ".  Role: " << KC::kRole
            << ". Id: " << kcc.key_id << '\n';
}


// print static information of a factor label
template <typename FT>
constexpr void factor_print()
{
  std::cout << FT::kFactorLabel << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse statically the ttupelemuple of keys data
  traverse_tup<typename FT::KeysSet_t>();

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " { ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "{\n";


  std::cout << "\t----- \n";
}

// This is the non static version
template <typename FT>
void factor_print(const FT& fact)
{
  Eigen::IOFormat
      CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "  ", ";");

  std::cout << FT::kFactorLabel << " - id : " << fact.factor_id << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse_tup(fact.keys_set);
  // for_each_in_tuple(fact.keys_set, [](const auto& kcc){
  //   // using KT = ;
  //   std::cout << "\t\t+ Key Nature: " << decltype(kcc)::kKeyName << ".  Role: " <<
  //   decltype(kcc)::kRole
  //             << ". Id: " << kcc.key_id << '\n';
  // });
  // for_each_in_tuple(fact.keys_set, &printtupelem);
  std::apply([](auto... kcc) { ((print_KeyContextConduct(kcc)), ...); }, fact.keys_set);


  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << "\n\t\t { ";
  for (int i = 0; i < FT::kMeasureComponentsName.size(); i++)
    std::cout << FT::kMeasureComponentsName[i] << ": " << fact.z[i] << "  ";
  std::cout << "}\n\t\t Cov: [ " << fact.z_cov.format(CommaInitFmt) << " ] \n";


  std::cout << "\t----- \n";
}

#endif
