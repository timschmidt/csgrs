/***************************************************************************
 *            algebra/sweeper.hpp
 *
 *  Copyright  2010-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

/*! \file algebra/sweeper.hpp
 *  \brief Classes for simplifying the representation of a polynomial expansion.
 */

#ifndef ARIADNE_SWEEPER_HPP
#define ARIADNE_SWEEPER_HPP

#include "utility/macros.hpp"
#include "utility/attribute.hpp"
#include "numeric/float.decl.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"

namespace Ariadne {

template<class F> class Sweeper;
template<class F> class SweeperBase;
template<class I, class X> class Expansion;


template<class F> class UnknownError;

template<class F> class SweeperInterface {
    friend class Sweeper<F>;
    friend class SweeperBase<F>;
  protected:
    typedef typename F::PrecisionType PR;
  public:
    virtual ~SweeperInterface() = default;
    inline Void sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const { this->_sweep(p,e); }
    inline Void sweep(Expansion<MultiIndex,FloatBounds<PR>>& p, FloatError<PR>& e) const { this->_sweep(p,e); }
    inline Void sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p) const { this->_sweep(p); }
    inline Void sweep(Expansion<MultiIndex,FloatUpperInterval<PR>>& p, FloatError<PR>& e) const { this->_sweep(p,e); }
    inline PR precision() const { return this->_precision(); }
  private:
    virtual SweeperInterface* _clone() const = 0;
    virtual PR _precision() const = 0;
    virtual Void _sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const = 0;
    virtual Void _sweep(Expansion<MultiIndex,FloatBounds<PR>>& p, FloatError<PR>& e) const = 0;
    virtual Void _sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p) const = 0;
    virtual Void _sweep(Expansion<MultiIndex,FloatUpperInterval<PR>>& p, FloatError<PR>& e) const = 0;
    virtual Void _write(OutputStream& os) const = 0;
  public:
    inline SweeperInterface* _copy() const { return this->_clone(); }
    friend OutputStream& operator<<(OutputStream& os, const SweeperInterface& swp) { swp._write(os); return os; }
};

/*! \brief A class for removing terms in a polynomial expansion which fall below some significance level, putting them in a uniform error.
 *
 *  \internal The virtual methods in SweeperInterface are private so that they can be implemented in terms of public inline methods for efficiency.
 *  In particular, the concrete sweeper classes have an inline discard(...) method which is used in the sweep(...) method since the exact type is known.
 *  This is a major efficiency improvement.
 */
template<class F> class Sweeper
    : public Handle<const SweeperInterface<F>>
{
    typedef typename F::PrecisionType PR;
  public:
    typedef SweeperInterface<F> Interface;
    typedef F RawFloatType;
    typedef PR PrecisionType;
    typedef MultiIndex IndexType;
  public:
    using Handle<const Interface>::Handle;

    Sweeper();

    //! \brief The precision to which terms should be built.
    inline PrecisionType precision() const { return this->_ptr->_precision(); }
    //! \brief Discard terms in the expansion, adding the absolute value of the coefficient to the uniform error.
    inline Void sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const { this->_ptr->_sweep(p,e); }
    //! \brief Discard terms in the expansion, adding the absolute value of the coefficient to the uniform error.
    inline Void sweep(Expansion<MultiIndex,FloatBounds<PR>>& p, FloatError<PR>& e) const { this->_ptr->_sweep(p,e); }
    //! \brief Discard terms in the expansion, without keeping track of discarded terms.
    inline Void sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p, UnknownError<F>&) const { this->_ptr->_sweep(p); }
    //! \brief Discard terms in the expansion, adding the absolute value of the coefficient to the uniform error.
    inline Void sweep(Expansion<MultiIndex,FloatUpperInterval<PR>>& p, FloatError<PR>& e) const { this->_ptr->_sweep(p,e); }
    friend OutputStream& operator<<(OutputStream& os, const Sweeper<F>& swp) { return os << *swp._ptr; }
};

template<class F> class SweeperBase
    : public virtual SweeperInterface<F>
{
    typedef typename F::PrecisionType PR;
    virtual Void _sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatBounds<PR>>& p, FloatError<PR>& e) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatUpperInterval<PR>>& p, FloatError<PR>& e) const override;
    virtual Bool _discard(const MultiIndex& a, const F& x) const = 0;
};


template<class SWP, class F> class SweeperMixin
    : public virtual SweeperBase<F>
{
    typedef typename F::PrecisionType PR;
    virtual SweeperInterface<F>* _clone() const override final;
    virtual PR _precision() const override final;
    virtual Bool _discard(const MultiIndex& a, const F& x) const override final;
};

template<class SWP, class F>
auto SweeperMixin<SWP,F>::_clone() const -> SweeperInterface<F>*
{
    return new SWP(static_cast<const SWP&>(*this));
}

template<class SWP, class F>
auto SweeperMixin<SWP,F>::_precision() const -> PR
{
    return static_cast<const SWP*>(this)->SWP::precision();
}

template<class SWP, class F>
auto SweeperMixin<SWP,F>::_discard(const MultiIndex& a, const F& x) const -> Bool
{
    return static_cast<const SWP*>(this)->SWP::discard(a,x);
}


template<class F> class RelativeSweeperBase
    : public virtual SweeperInterface<F>
{
    typedef typename F::PrecisionType PR;
    virtual Void _sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatBounds<PR>>& p, FloatError<PR>& e) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p) const override;
    virtual Void _sweep(Expansion<MultiIndex,FloatUpperInterval<PR>>& p, FloatError<PR>& e) const override;
    virtual Bool _discard(const F& x, const F& nrm) const = 0;
};


template<class SWP, class F> class RelativeSweeperMixin
    : public virtual RelativeSweeperBase<F>
{
    typedef typename F::PrecisionType PR;
    virtual SweeperInterface<F>* _clone() const override final;
    virtual PR _precision() const override final;
    virtual Bool _discard(const F& x, const F& nrm) const override final;
};

template<class SWP, class F>
auto RelativeSweeperMixin<SWP,F>::_clone() const -> SweeperInterface<F>*
{
    return new SWP(static_cast<const SWP&>(*this));
}

template<class SWP, class F>
auto RelativeSweeperMixin<SWP,F>::_precision() const -> PR
{
    return static_cast<const SWP*>(this)->SWP::precision();
}

template<class SWP, class F>
auto RelativeSweeperMixin<SWP,F>::_discard(const F& x, const F& nrm) const -> Bool
{
    return static_cast<const SWP*>(this)->SWP::discard(x,nrm);
}


//! \brief A sweeper class which discards terms whose absolute value is smaller than a threshold.
template<class F> class ThresholdSweeper : public SweeperMixin<ThresholdSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
    F _sweep_threshold;
  public:
    ThresholdSweeper(PR precision, F sweep_threshold)
        : _coefficient_precision(precision), _sweep_threshold(sweep_threshold) { ARIADNE_ASSERT(sweep_threshold>=0); }
    ThresholdSweeper(PR precision, ApproximateDouble sweep_threshold)
        : _coefficient_precision(precision), _sweep_threshold(cast_exact(sweep_threshold),precision) { ARIADNE_ASSERT(cast_exact(sweep_threshold)>=0); }
    inline PR precision() const { return _coefficient_precision; }
    inline F sweep_threshold() const { return _sweep_threshold; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return abs(x) < this->_sweep_threshold; }
  private:
    virtual Void _write(OutputStream& os) const { os << "ThresholdSweeper( sweep_threshold="<<this->_sweep_threshold<<" )"; };
};

//! \brief A sweeper class which does not discard any terms at all.
template<class F> class RelativeThresholdSweeper : public RelativeSweeperMixin<RelativeThresholdSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
    F _relative_sweep_threshold;
  public:
    RelativeThresholdSweeper(PR precision, F relative_sweep_threshold)
        : _coefficient_precision(precision), _relative_sweep_threshold(relative_sweep_threshold) { ARIADNE_ASSERT(relative_sweep_threshold>0); }
    inline PR precision() const { return _coefficient_precision; }
    inline F relative_sweep_threshold() const { return _relative_sweep_threshold; }
    inline Bool discard(const F& x, const F& nrm) const { return abs(x) < mul(approx,this->_relative_sweep_threshold,nrm); }
  private:
    virtual Void _write(OutputStream& os) const { os << "RelativeThresholdSweeper( relative_sweep_threshold="<<this->_relative_sweep_threshold<<" )"; };
};

//! \brief A sweeper class which does not discard any terms at all.
template<class F> class TrivialSweeper : public SweeperMixin<TrivialSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
  public:
    TrivialSweeper(PR precision) : _coefficient_precision(precision) { }
    inline PR precision() const { return _coefficient_precision; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return false; }
  private:
    virtual Void _sweep(Expansion<MultiIndex,Float<PR>>& p, FloatError<PR>& e) const final { }
    virtual Void _sweep(Expansion<MultiIndex,FloatApproximation<PR>>& p) const final { }
    virtual Void _write(OutputStream& os) const { os << "TrivialSweeper"; }
};

//! \brief A sweeper class which only discards the zero term.
template<class F> class NullSweeper : public SweeperMixin<NullSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
  public:
    NullSweeper(PR precision) : _coefficient_precision(precision) { }
    inline PR precision() const { return _coefficient_precision; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return x==0.0; }
  private:
    virtual Void _write(OutputStream& os) const { os << "NullSweeper"; }
};

//! \brief A sweeper class which discards non-affine terms.
template<class F> class AffineSweeper : public SweeperMixin<AffineSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
  public:
    AffineSweeper(PR precision) : _coefficient_precision(precision) { }
    inline PR precision() const { return _coefficient_precision; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return a.degree()>1; }
  private:
    virtual Void _write(OutputStream& os) const { os << "AffineSweeper"; }
};

//! \brief A sweeper class which discards terms whose total degree is above some threshold.
template<class F> class GradedSweeper : public SweeperMixin<GradedSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
  public:
    GradedSweeper(PR precision, DegreeType degree) : _coefficient_precision(precision), _degree(degree) { }
    DegreeType degree() const { return this->_degree; }
    inline PR precision() const { return _coefficient_precision; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return a.degree()>this->_degree; }
  private:
    virtual Void _write(OutputStream& os) const { os << "GradedSweeper( degree="<<this->_degree<<" )"; }
  private:
    DegreeType _degree;
};

//! \brief A sweeper class which discards terms whose total degree is above some threshold or whose absolute value is smaller than a threshold.
template<class F> class GradedThresholdSweeper : public SweeperMixin<GradedThresholdSweeper<F>,F> {
    typedef PrecisionType<F> PR;
    PR _coefficient_precision;
    F _sweep_threshold;
public:
    GradedThresholdSweeper(PR precision, DegreeType degree, F sweep_threshold)
            : _coefficient_precision(precision), _sweep_threshold(sweep_threshold), _degree(degree) { ARIADNE_ASSERT(sweep_threshold>=0); }
    DegreeType degree() const { return this->_degree; }
    inline F sweep_threshold() const { return _sweep_threshold; }
    inline PR precision() const { return _coefficient_precision; }
    inline Bool discard(const MultiIndex& a, const F& x) const { return a.degree()>this->_degree || abs(x) < this->_sweep_threshold; }
private:
    virtual Void _write(OutputStream& os) const { os << "GradedThresholdSweeper( degree="<<this->_degree<<", sweep_threshold="<<this->_sweep_threshold<<" )"; }
private:
    DegreeType _degree;
};

template<> inline Sweeper<FloatDP>::Sweeper()
    : Sweeper(new ThresholdSweeper<FloatDP>(dp,std::numeric_limits<float>::epsilon())) { }
template<> inline Sweeper<FloatMP>::Sweeper()
    : Sweeper(new ThresholdSweeper<FloatMP>(MultiplePrecision(64_bits),std::numeric_limits<double>::epsilon())) { }

using SweeperDP=Sweeper<FloatDP>;
using SweeperMP=Sweeper<FloatMP>;

template<class PR> ThresholdSweeper(PR,RawFloatType<PR>) -> ThresholdSweeper<RawFloatType<PR>>;
template<class PR> RelativeThresholdSweeper(PR,RawFloatType<PR>) -> RelativeThresholdSweeper<RawFloatType<PR>>;
template<class PR> TrivialSweeper(PR) -> TrivialSweeper<RawFloatType<PR>>;
template<class PR> NullSweeper(PR) -> NullSweeper<RawFloatType<PR>>;
template<class PR> AffineSweeper(PR) -> AffineSweeper<RawFloatType<PR>>;
template<class PR> GradedSweeper(PR,DegreeType) -> GradedSweeper<RawFloatType<PR>>;
template<class PR> GradedThresholdSweeper(PR,DegreeType,RawFloatType<PR>) -> GradedThresholdSweeper<RawFloatType<PR>>;

} // namespace Ariadne

#endif // ARIADNE_SWEEPER_HPP
