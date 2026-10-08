/***************************************************************************
 *            scaled_function_patch.tpl.hpp
 *
 *  Copyright  2008-20  Pieter Collins
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

#ifndef FUNCTION_PATCH_TCC
#define FUNCTION_PATCH_TCC

#include "function/functional.hpp"

#include <iostream>
#include <iomanip>

#include "utility/macros.hpp"
#include "utility/exceptions.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "function/polynomial.hpp"
#include "algebra/differential.hpp"

#include "function/function.hpp"
#include "function/function_mixin.hpp"

#include "algebra/evaluate.hpp"

#define VOLATILE ;

namespace Ariadne {

namespace {

Box<Interval<FloatMP>> convert_box(BoxDomainType const& bx, MultiplePrecision pr) {
    Box<Interval<FloatMP>> r(bx.dimension(),Interval<FloatMP>(FloatMP(pr),FloatMP(pr)));
    for(SizeType i=0; i!=r.dimension(); ++i) { r[i]=convert_interval(bx[i],pr); }
    return r;
}

} // namespace

inline decltype(auto) contains(IntervalDomainType const& bx, Scalar<FloatMPBounds> const& x) { return contains(convert_interval(bx,x.precision()),x); }
inline decltype(auto) contains(IntervalDomainType const& bx, Scalar<FloatMPApproximation> const& x) { return contains(convert_interval(bx,x.precision()),x); }
inline decltype(auto) contains(BoxDomainType const& bx, Vector<FloatMPBounds> const& x) { return contains(convert_box(bx,x.zero_element().precision()),x); }
inline decltype(auto) contains(BoxDomainType const& bx, Vector<FloatMPApproximation> const& x) { return contains(convert_box(bx,x.zero_element().precision()),x); }


template<class M> Void _set_scaling(ScaledFunctionPatch<M>& x, const IntervalDomainType& ivl, SizeType j)
{
    // A scaling of [-1,+1] into [a,b] has the form s->rx+c where c is centre and r radius of ivl
    const FloatDP& l=ivl.lower_bound();
    const FloatDP& u=ivl.upper_bound();
    FloatDPBall c{hlf(l+u)};
    FloatDPBall r{hlf(u-l)};
    FloatDPError e=c.error()+r.error();
    MultiIndex a(x.argument_size());
    x.expansion().append(a,c.value());
    ++a[j];
    x.expansion().append(a,r.value());
    x.set_error(e);
}



inline OutputStream& operator<<(OutputStream& os, const Representation<FloatDP>& flt_repr)
{
    const FloatDP& flt=*flt_repr.pointer;
    Int precision=os.precision(); std::ios_base::fmtflags flags = os.flags();
    os.precision(17); os.setf(std::ios_base::showpoint);
    os << "FloatDP(" << flt << ")";
    os.precision(precision); os.flags(flags);
    return os;
}

inline OutputStream& operator<<(OutputStream& os, const Representation<FloatMP>& flt_repr)
{
    return os << *flt_repr.pointer;
}

template<class PR> inline OutputStream& operator<<(OutputStream& os, const Representation<PositiveFloatUpperBound<PR>>& flt_repr)
{
    return os << reinterpret_cast<Representation<RawFloat<PR>>const&>(flt_repr);
}

template<class PR> inline OutputStream& operator<<(OutputStream& os, const Representation<FloatError<PR>>& flt_repr)
{
    return os << reinterpret_cast<Representation<RawFloat<PR>>const&>(flt_repr);
}

template<class PR> inline OutputStream& operator<<(OutputStream& os, const Representation<IntervalDomainType>& ivl_repr)
{
    const IntervalDomainType& ivl=*ivl_repr.pointer;
    Int precision=os.precision(); std::ios_base::fmtflags flags = os.flags();
    os.precision(17); os.setf(std::ios_base::showpoint);
    os << "IntervalDomainType("<<ivl.lower_bound()<<","<<ivl.upper_bound()<<")";
    os.precision(precision); os.flags(flags);
    return os;
}

template<class PR> inline OutputStream& operator<<(OutputStream& os, const Representation< Expansion<MultiIndex,RawFloat<PR>> >& exp_repr)
{
    const Expansion<MultiIndex,RawFloat<PR>>& exp=*exp_repr.pointer;
    Int precision=os.precision(); std::ios_base::fmtflags flags = os.flags();
    os.precision(17); os.setf(std::ios_base::showpoint);
    os << "Expansion<MultiIndex,Float<PR>>(" << exp.argument_size() << "," << exp.number_of_nonzeros();
    for(typename Expansion<MultiIndex,RawFloat<PR>>::ConstIterator iter=exp.begin(); iter!=exp.end(); ++iter) {
        for(SizeType j=0; j!=iter->index().size(); ++j) {
            os << "," << Nat(iter->index()[j]);
        }
        os << "," << iter->coefficient();
    }
    os << ")";
    os.precision(precision); os.flags(flags);
    return os;
}

template<class PR> inline OutputStream& operator<<(OutputStream& os, const Representation< Expansion<MultiIndex,FloatApproximation<PR>> >& exp_repr) {
    return os << reinterpret_cast<Expansion<MultiIndex,RawFloat<PR>>const&>(exp_repr);
}

template<class X> inline OutputStream& operator<<(OutputStream& os, const Representation< Vector<X> >& vec_repr)
{
    const Vector<X>& vec=*vec_repr.pointer;
    ARIADNE_ASSERT(vec.size()!=0);
    os << "(";
    for(SizeType i=0; i!=vec.size(); ++i) {
        if(i!=0) { os << ","; }
        os << representation(vec[i]);
    }
    os << ")";
    return os;
}

inline OutputStream& operator<<(OutputStream& os, const Representation< BoxDomainType >& box_repr)
{
    const BoxDomainType& vec=*box_repr.pointer;
    ARIADNE_ASSERT(vec.size()!=0);
    os << "(";
    for(SizeType i=0; i!=vec.size(); ++i) {
        if(i!=0) { os << ","; }
        os << representation(vec[i]);
    }
    os << ")";
    return os;
}

template<class F> inline OutputStream& operator<<(OutputStream& os, const Representation< Sweeper<F> >& swp_repr)
{
    const Sweeper<F>& swp=*swp_repr.pointer;
    Int precision=os.precision(); std::ios_base::fmtflags flags = os.flags();
    os.precision(17); os.setf(std::ios_base::showpoint);
    os << swp;
    os.precision(precision); os.flags(flags);
    return os;
}

template<class T> inline OutputStream& operator<<(OutputStream& os, const Representation< List<T> >& lst_repr)
{
    const List<T>& lst=*lst_repr.pointer;
    ARIADNE_ASSERT(lst.size()!=0);
    os << "(";
    for(SizeType i=0; i!=lst.size(); ++i) {
        if(i!=0) { os << ","; }
        os << representation(lst[i]);
    }
    os << ")";
    return os;
}



template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, PropertiesType prp)
    : _domain(d), _model(d.size(),prp)
{
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, const Expansion<MultiIndex,ExactDouble>& p, const ExactDouble& e, const Sweeper<RawFloat<PR>>& prp)
    : _domain(d), _model(p,e,prp)
{
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, const Expansion<MultiIndex,RawFloat<PR>>& p, const RawFloat<PR>& e, const Sweeper<RawFloat<PR>>& prp)
    : _domain(d), _model(p,e,prp)
{
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, const Expansion<MultiIndex,CoefficientType>& p, const ErrorType& e, const Sweeper<RawFloat<PR>>& prp)
    : _domain(d), _model(p,e,prp)
{
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, const ModelType& m)
    : _domain(d), _model(m)
{
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const BoxDomainType& d, const ScalarFunctionType<M>& f, PropertiesType prp)
    : _domain(d), _model(f.argument_size(),prp)
{
    if constexpr (AnInterval<typename M::NumericType>) {
        ARIADNE_FAIL_MSG("Cannot convert multivalued IntervalTaylorFunctionModel "<<*this<<" to a single-valued function.");
    } else {
        ARIADNE_ASSERT_MSG(d.size()==f.argument_size(),"d="<<d<<" f="<<f);
        Vector<ModelType> x=ModelType::scalings(d,prp);
        this->_model=f.evaluate(x);
        this->_model.simplify();
    }
}

template<class M> ScaledFunctionPatch<M>::ScaledFunctionPatch(const ScalarFunctionModelType<M>& f)
    : ScaledFunctionPatch(dynamic_cast<const ScaledFunctionPatch<M>&>(f.reference()))
{
    ARIADNE_ASSERT_MSG(dynamic_cast<const ScaledFunctionPatch<M>*>(f.raw_pointer())," f="<<f);
}

template<class M> ScaledFunctionPatch<M>& ScaledFunctionPatch<M>::operator=(const ScalarFunctionModelType<M>& f)
{
    return (*this)=ScaledFunctionPatch<M>(this->domain(),f,this->properties());
}


template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::zero(const BoxDomainType& d, PropertiesType prp)
{
    return ScaledFunctionPatch<M>(d,ModelType::zero(d.size(),prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::constant(const BoxDomainType& d, const NumericType& c, PropertiesType prp)
{
    return ScaledFunctionPatch<M>(d,ModelType::constant(d.size(),c,prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::constant(const BoxDomainType& d, const GenericNumericType& c, PropertiesType prp)
{
    return ScaledFunctionPatch<M>(d,ModelType::constant(d.size(),c,prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::unit_ball(const BoxDomainType& d, PropertiesType prp)
{
    return ScaledFunctionPatch<M>(d,ModelType::unit_ball(d.size(),prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::coordinate(const BoxDomainType& d, SizeType j, PropertiesType prp)
{
    ARIADNE_ASSERT(j<d.size());
    return ScaledFunctionPatch<M>(d,ModelType::scaling(d.size(),j,d[j],prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::projection(const BoxDomainType& d, SizeType j, PropertiesType prp)
{
    return ScaledFunctionPatch<M>::coordinate(d,j,prp);
}

template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatch<M>::projection(const BoxDomainType& d, Range js, PropertiesType prp)
{
    VectorScaledFunctionPatch<M> x(js.size(),d,prp);
    for(SizeType i=0; i!=x.size(); ++i) {
        x[i]=ScaledFunctionPatch<M>::coordinate(d,js.start()+i,prp);
    }
    return x;
}

template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatch<M>::identity(const BoxDomainType& d, PropertiesType prp)
{
    return VectorScaledFunctionPatch<M>(d,ModelType::scalings(d,prp));
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::create_zero() const
{
    return ScaledFunctionPatch<M>(this->domain(),this->properties());
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::create_constant(NumericType const& c) const
{
    return ScaledFunctionPatch<M>::constant(this->domain(),c,this->properties());
}

template<class M> ScaledFunctionPatch<M> ScaledFunctionPatch<M>::create_constant(GenericNumericType const& c) const
{
    return this->create_constant(NumericType(c,this->precision()));
}

//FIXME: Should allow this in code file
/*
template<class M> ScaledFunctionPatch<M>* ScaledFunctionPatch<M>::_clone() const
{
    return new ScaledFunctionPatch<M>(*this);
}
*/

template<class M> ScaledFunctionPatchFactory<M>* ScaledFunctionPatch<M>::_factory() const
{
    return new ScaledFunctionPatchFactory<M>(this->_model.properties());
}

template<class M> ScaledFunctionPatch<M>* ScaledFunctionPatch<M>::_create() const
{
    return new ScaledFunctionPatch<M>(this->domain(),this->_model.properties());
}


template<class M> Void VectorScaledFunctionPatch<M>::adjoin(const ScaledFunctionPatch<M>& sf)
{
    ARIADNE_ASSERT_MSG(sf.domain()==this->domain(),"sf="<<sf);
    this->_models=join(this->_models,sf.model());
}


template<class M> Void ScaledFunctionPatch<M>::restrict(const BoxDomainType& dom) {
    (*this)=restriction(*this,dom);
}



inline Bool operator==(FloatDPBounds x1, Int n2) { return x1.upper_raw()==FloatDP(n2,dp) && x1.lower_raw()==FloatDP(n2,dp); }
inline Bool operator==(FloatDPApproximation x1, Int n2) { return x1.raw()==FloatDP(n2,dp); }

inline Bool operator!=(FloatDPBounds x1, Int n2) { return x1.upper_raw()!=FloatDP(n2,dp) || x1.lower_raw()!=FloatDP(n2,dp); }
inline Bool operator!=(FloatDPApproximation x1, Int n2) { return x1.raw()!=FloatDP(n2,dp); }

inline Bool operator> (FloatDPBounds x1, Int n2) { return x1.lower_raw()> FloatDP(n2,dp); }
inline Bool operator> (FloatDPApproximation x1, Int n2) { return x1.raw()> FloatDP(n2,dp); }


template<class M> auto ScaledFunctionPatch<M>::polynomial() const -> MultivariatePolynomial<NumericType>
{
    NumericType zero(0,this->model().precision());

    Vector<MultivariatePolynomial<NumericType> > pid=MultivariatePolynomial<NumericType>::coordinates(this->argument_size(),zero);
    return horner_evaluate(this->expansion(),unscale(pid,this->domain()))+NumericType(-this->error(),+this->error());

    MultivariatePolynomial<NumericType> z(this->argument_size(),zero);
    MultivariatePolynomial<NumericType> p=z;

    Vector<MultivariatePolynomial<NumericType> > s(this->argument_size(),z);
    for(SizeType j=0; j!=this->argument_size(); ++j) {
        auto domj=convert_interval(this->domain()[j],this->precision());
        if(domj.lower_bound()>=domj.upper_bound()) {
            ARIADNE_ASSERT(this->domain()[j].is_singleton());
            s[j]=MultivariatePolynomial<NumericType>::constant(this->argument_size(),zero);
        } else {
            //s[j]=Ariadne::polynomial(ModelType::unscaling(this->argument_size(),j,this->domain()[j],this->properties()));
            s[j]=(MultivariatePolynomial<NumericType>::coordinate(this->argument_size(),j,zero)-domj.midpoint())/static_cast<NumericType>(domj.radius());
        }
    }

    return compose(p,s);
}

template<class M> ScalarFunctionType<M> ScaledFunctionPatch<M>::function() const
{
    if constexpr (AnInterval<CoefficientType>) {
        ARIADNE_FAIL_MSG("IntervalTaylorModel object "<<*this<<" cannot be converted to a function.");
        abort();
    } else {
        return ScalarFunctionType<M>(new ScaledFunctionPatch<M>(*this));
    }
}

template<class M> ScalarFunctionType<M> ScaledFunctionPatch<M>::generic() const
{
    return this->function();
}

/*
template<class M> Bool ScaledFunctionPatch<M>::operator==(const ScaledFunctionPatch<M>& tv) const
{
    ARIADNE_DEPRECATED("operator==(ScaledFunctionPatch<M>,ScaledFunctionPatch<M>)","Use same(...) instead.");
    return same(*this,tv);
}
*/


template<class M> auto ScaledFunctionPatch<M>::operator() (const Vector<FloatApproximation<PR>>& x) const
    -> ArithmeticType<CoefficientType,FloatApproximation<PR>>
{
    const ScaledFunctionPatch<M>& f=*this;
    if(!decide(contains(f.domain(),x))) {
        ARIADNE_THROW(DomainException,"tf.evaluate(ax) with tf="<<f<<", ax="<<x," ax is not an element of tf.domain()="<<f.domain());
    }
    Vector<FloatApproximation<PR>> sx=Ariadne::unscale(x,f._domain);
    return evaluate(this->_model,sx);
}

template<class M> auto ScaledFunctionPatch<M>::operator()(const Vector<FloatBounds<PR>>& x) const
    -> ArithmeticType<CoefficientType,FloatBounds<PR>>
{
    const ScaledFunctionPatch<M>& f=*this;
    if(!definitely(contains(f.domain(),x))) {
        ARIADNE_THROW(DomainException,"evaluate(f,x) with f="<<f<<", x="<<x,"x is not definitely an element of f.domain()="<<f.domain());
    }
    return unchecked_evaluate(f,x);
}

template<class M> auto ScaledFunctionPatch<M>::operator()(const Vector<Float<PR>>& x) const -> ArithmeticType<CoefficientType,FloatBounds<PR>>
{
    return evaluate(*this,Vector<FloatBounds<PR>>(x));
}

template<class M> auto ScaledFunctionPatch<M>::operator()(const Vector<ValidatedNumber>& y) const -> ValidatedNumber
{
    if constexpr (AnInterval<CoefficientType>) {
        ARIADNE_FAIL_MSG("Cannot evaluate an IntervalTaylorModel on a generic Number");
    } else {
        return this->operator()(Vector<FloatBounds<PR>>(y,this->precision()));
    }
}

template<class M> ScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::_unchecked_compose(const ScaledFunctionPatch<M>& g, const VectorScaledFunctionPatch<M>& f) {
    return ScaledFunctionPatch<M>(f.domain(),compose(g.model(),unscale(f.models(),g.domain())));
}

template<class M> VectorScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::_unchecked_compose(const VectorScaledFunctionPatch<M>& g, const VectorScaledFunctionPatch<M>& f) {
    return VectorScaledFunctionPatch<M>(f.domain(),compose(g.models(),unscale(f.models(),g.domain())));
}


/*
template<class M, class V> decltype(auto) model_gradient(const M& f, const V& s) {
    return gradient(f,s); }

template<class M> auto ScaledFunctionPatch<M>::gradient(const Vector<NumericType>& x) const -> Covector<NumericType>
{
    Vector<NumericType> s=unscale(x,this->_domain);
    Covector<NumericType> g=model_gradient(this->_model,s);
//    Covector<NumericType> g=this->_model.gradient(s);
    for(SizeType j=0; j!=g.size(); ++j) {
        NumericType rad=convert_interval(this->_domain[j],this->model().precision()).radius();
        g[j]/=rad;
    }
    return g;
}
*/


template<class M> OutputStream& write_polynomial(OutputStream& os, ScaledFunctionPatch<M> const& fp) {
    typedef typename ScaledFunctionPatch<M>::CoefficientType CoefficientType;
    typedef typename ScaledFunctionPatch<M>::PrecisionType PR;

    os << "{";
    if constexpr (AnInterval<CoefficientType>) {
        os << fp.polynomial();
    } else {
        os << MultivariatePolynomial<FloatApproximation<PR>>(fp.polynomial());
    }

    if(fp.error().raw()>0.0_x) { os << "+/-" << fp.error(); }

    os << "}";
    return os;
}

template<class M> OutputStream& ScaledFunctionPatch<M>::_write(OutputStream& os) const {
    os << "ScaledFunctionPatch( dom=" << this->domain() << ", rng=" << this->range() << ", f=";
    write_polynomial(os,*this);
    return os << " )";
}

template<class M> OutputStream& ScaledFunctionPatch<M>::repr(OutputStream& os) const
{
    return os << "ScaledFunctionPatch<M>(" << representation(this->domain()) << ", " << representation(this->model())<<")";
}

/*
static double TAYLOR_FUNCTION_WRITING_ACCURACY = 1e-8;

template<class M> OutputStream& operator<<(OutputStream& os, const Representation<ScaledFunctionPatch<M>>& frepr)
{
    ScaledFunctionPatch<M> const& function=*frepr.pointer;
    ScaledFunctionPatch<M> truncated_function=function;
    truncated_function.set_error(0.0);
    truncated_function.simplify(ThresholdSweeper(TAYLOR_FUNCTION_WRITING_ACCURACY));

    os << midpoint(truncated_function.polynomial());
        if(truncated_function.error()>0.0) { os << "+/-" << truncated_function.error(); }
    if(function.error()>0.0) { os << "+/-" << function.error(); }
    // TODO: Use Unicode +/- literal when this becomes avaialable in C++0x
    return os;
}
*/
/*
template<class M> OutputStream& operator<<(OutputStream& os, const ModelRepresentation<ScaledFunctionPatch<M>>& frepr)
{
    ScaledFunctionPatch<M> const& f=*frepr.pointer;
    FloatDP truncatation_error = 0.0;
    os << "<"<<f.domain()<<"\n";
    for(ModelType::ConstIterator iter=f.begin(); iter!=f.end(); ++iter) {
        if(abs(iter->coefficient())>frepr.threshold) { truncatation_error+=abs(iter->coefficient()); }
        else { os << iter->index() << ":" << iter->coefficient() << ","; }
    }
    os << "+/-" << truncatation_error << "+/-" << f.error();
    return os;
}
*/

template<class M> OutputStream& operator<<(OutputStream& os, const ModelRepresentation<ScaledFunctionPatch<M>>& frepr)
{
    typedef typename M::PropertiesType Prp;
    ScaledFunctionPatch<M> const& f=*frepr.pointer;
    ScaledFunctionPatch<M> tf=f;
    tf.clobber();
    tf.simplify(Prp(frepr.precision(),frepr.threshold));
    os << "("<<tf.model()<<"+/-"<<f.error();
    return os;
}

template<class M> OutputStream& operator<<(OutputStream& os, const PolynomialRepresentation<ScaledFunctionPatch<M>>& frepr)
{
    typedef typename M::RawFloatType F;
    typedef typename F::PrecisionType PR;
    ScaledFunctionPatch<M> const& function=*frepr.pointer;
    ScaledFunctionPatch<M> truncated_function = function;
    truncated_function.clobber();
    truncated_function.simplify(ThresholdSweeper<F>(frepr.precision(),frepr.threshold));
    FloatError<PR> truncatation_error = truncated_function.error();
    truncated_function.clobber();
    MultivariatePolynomial<FloatBounds<PR>> validated_polynomial_function=polynomial(truncated_function);
    MultivariatePolynomial<Float<PR>> polynomial_function = midpoint(validated_polynomial_function);
    if(frepr.names.empty()) { os << polynomial_function; }
    else { os << named_argument_repr(polynomial_function,frepr.names); }
    os << "+/-" << truncatation_error << "+/-" << function.error();
    return os;
}








template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(SizeType m, const BoxDomainType& d, PropertiesType prp)
    : _domain(d), _models(m,ModelType(d.size(),prp))
{
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(SizeType k, const ScaledFunctionPatch<M>& f)
    : _domain(f.domain()), _models(k,f.model())
{
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const VectorFunctionModelType<M>& f)
    : _domain(), _models()
{
    ARIADNE_ASSERT(dynamic_cast<const VectorScaledFunctionPatch<M>*>(&f.reference()));
    *this = dynamic_cast<const VectorScaledFunctionPatch<M>&>(f.reference());
}

template<class M> VectorScaledFunctionPatch<M>& VectorScaledFunctionPatch<M>::operator=(const VectorFunctionModelType<M>& f)
{
    ARIADNE_ASSERT(dynamic_cast<const VectorScaledFunctionPatch<M>*>(&f.reference()));
    *this = dynamic_cast<const VectorScaledFunctionPatch<M>&>(f.reference());
    return *this;
}




template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const BoxDomainType& d,
                                           const Vector<ModelType>& f)
    : _domain(d), _models(f)
{
    for(SizeType i=0; i!=f.size(); ++i) {
        ARIADNE_ASSERT_MSG(d.size()==f[i].argument_size(),"d="<<d<<", f="<<f<<", i="<<i<<", f[i]="<<f[i]<<", f[i].argument_size()="<<f[i].argument_size());
    }
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const BoxDomainType& d,
                                           const Vector<Expansion<MultiIndex,CoefficientType>>& f,
                                           const Vector<ErrorType>& e,
                                           PropertiesType prp)
    : _domain(d), _models(f.size(),ModelType(d.size(),prp))
{
    ARIADNE_ASSERT(f.size()==e.size());
    for(SizeType i=0; i!=f.size(); ++i) {
        ARIADNE_ASSERT(d.size()==f[i].argument_size());
        _models[i]=ModelType(f[i],e[i],prp);
    }
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const BoxDomainType& d,
                                           const Vector<Expansion<MultiIndex,CoefficientType>>& f,
                                           PropertiesType prp)
    : VectorScaledFunctionPatch<M>(d,f,Vector<ErrorType>(f.size(),prp.precision()),prp)
{
}



template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const BoxDomainType& d,
                                           const Vector<Expansion<MultiIndex,RawFloat<PR>>>& f,
                                           const Vector<RawFloat<PR>>& e,
                                           PropertiesType prp)
    : VectorScaledFunctionPatch<M>(d,Vector<M>(f.size(),[&](SizeType i){return M(f[i],e[i],prp);}))
{
}



template<class C, class F> decltype(auto) common(C const& c, F const& f) {
    assert(c.size()>0);
    auto r=f(c[0]);
    for(SizeType i=1; i!=c.size(); ++i) { ARIADNE_ASSERT(f(c[i])==r); }
    return r;
}


template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const BoxDomainType& d,
                                           const VectorFunctionType<M>& f,
                                           const PropertiesType& prp)
    : _domain(d), _models()
{
    //ARIADNE_ASSERT_MSG(f.result_size()>0, "d="<<d<<", f="<<f<<", prp="<<prp);
    ARIADNE_ASSERT(d.size()==f.argument_size());
    Vector<ModelType> x=ModelType::scalings(d,prp);
    this->_models=f(x);
    ARIADNE_DEBUG_ASSERT(this->argument_size()==f.argument_size());
    ARIADNE_DEBUG_ASSERT_MSG(this->result_size()==f.result_size(),"  f="<<f<<"\n  r="<<*this);
    this->simplify();
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const Vector<ScaledFunctionPatch<M>>& v)
    : _domain(v.zero_element().domain()),_models(v.size(),[&v](SizeType i){return v[i].model();})
{
    for(SizeType i=0; i!=v.size(); ++i) { ARIADNE_ASSERT(v[i].domain()==v.zero_element().domain()); }
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(const List<ScaledFunctionPatch<M>>& v)
    : _domain(common(v,[](auto sf){return sf.domain();})), _models(v.size(),[&v](SizeType i){return v[i].model();})
{
}

template<class M> VectorScaledFunctionPatch<M>::VectorScaledFunctionPatch(InitializerList<ScaledFunctionPatch<M>> lst)
    : VectorScaledFunctionPatch<M>(List<ScaledFunctionPatch<M>>(lst))
{ }


// FIXME: Should be possible to put in code file
/*
template<class M> VectorScaledFunctionPatch<M>* VectorScaledFunctionPatch<M>::_clone() const
{
    return new VectorScaledFunctionPatch<M>(*this);
}
*/

template<class M> ScaledFunctionPatchFactory<M>* VectorScaledFunctionPatch<M>::_factory() const
{
    return new ScaledFunctionPatchFactory<M>(this->_models.zero_element().properties());
}

template<class M> VectorScaledFunctionPatch<M>* VectorScaledFunctionPatch<M>::_create() const
{
    return new VectorScaledFunctionPatch<M>(this->result_size(), ScaledFunctionPatch<M>(this->domain(),this->properties()));
}



template<class M> VectorScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::constant(const BoxDomainType& d, const Vector<NumericType>& c, PropertiesType prp)
{
    return VectorScaledFunctionPatch<M>(d,ModelType::constants(d.size(),c,prp));
}

template<class M> VectorScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::projection(const BoxDomainType& d, Range js, PropertiesType prp)
{
    return ScaledFunctionPatch<M>::projection(d,js,prp);
}

template<class M> VectorScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::identity(const BoxDomainType& d, PropertiesType prp)
{
    return VectorScaledFunctionPatch<M>(d,ModelType::scalings(d,prp));
}


template<class M> auto VectorScaledFunctionPatch<M>::polynomials() const -> Vector<MultivariatePolynomial<NumericType>>
{
    Vector<MultivariatePolynomial<NumericType> > p(this->result_size(),MultivariatePolynomial<NumericType>(this->argument_size(),this->precision()));
    for(SizeType i=0; i!=this->result_size(); ++i) {
        p[i]=static_cast<ScaledFunctionPatch<M>>((*this)[i]).polynomial();
    }
    return p;
}

template<class M> auto VectorScaledFunctionPatch<M>::expansions() const -> Vector<Expansion<MultiIndex,CoefficientType>> const
{
    Vector<Expansion<MultiIndex,CoefficientType>> e(this->result_size(),Expansion<MultiIndex,CoefficientType>(this->argument_size(),this->precision()));
    for(SizeType i=0; i!=this->result_size(); ++i) {
        e[i]=this->models()[i].expansion();
    }
    return e;
}

template<class M> auto VectorScaledFunctionPatch<M>::values() const -> Vector<ValueType> const
{
    return elementwise([&](auto x){return x.value();},this->models());
    Vector<ValueType> e(this->result_size(),this->precision());
    for(SizeType i=0; i!=this->result_size(); ++i) {
        e[i]=this->models()[i].value();
    }
    return e;
}

template<class M> auto VectorScaledFunctionPatch<M>::errors() const -> Vector<ErrorType> const
{
    return Vector<FloatError<PR>>(this->result_size(),[&](SizeType i){return this->models()[i].error();},this->zero_element().error().precision());
}

template<class M> auto VectorScaledFunctionPatch<M>::error() const -> ErrorType const
{
    FloatError<PR> e(0u,this->precision());
    for(SizeType i=0; i!=this->result_size(); ++i) {
        e=max(e,this->models()[i].error());
    }
    return e;
}

template<class M> VectorFunctionType<M> VectorScaledFunctionPatch<M>::function() const
{
    if constexpr (AnInterval<CoefficientType>) {
        ARIADNE_FAIL_MSG("Cannot convert multivalued IntervalTaylorFunctionModel "<<*this<<" to a single-valued function.");
    } else {
        return VectorFunctionType<M>(new VectorScaledFunctionPatch<M>(*this));
    }
}

template<class M> VectorFunctionType<M> VectorScaledFunctionPatch<M>::generic() const
{
    return this->function();
}


template<class M> auto VectorScaledFunctionPatch<M>::properties() const -> PropertiesType
{
    return this->_models.zero_element().properties();
}

template<class M> auto VectorScaledFunctionPatch<M>::precision() const -> PrecisionType
{
    return this->_models.zero_element().precision();
}


template<class M> Void VectorScaledFunctionPatch<M>::set_properties(PropertiesType prp)
{
    for(SizeType i=0; i!=this->result_size(); ++i) {
        this->_models[i].set_properties(prp);
    }
}

template<class M> const BoxDomainType VectorScaledFunctionPatch<M>::domain() const
{
    return this->_domain;
}

template<class M> const BoxDomainType VectorScaledFunctionPatch<M>::codomain() const
{
    BoxDomainType result(this->result_size());
    for(SizeType i=0; i!=result.size(); ++i) {
        result[i]=this->_models[i].codomain();
    }
    return result;
}


template<class M> const typename VectorScaledFunctionPatch<M>::RangeType VectorScaledFunctionPatch<M>::range() const
{
    RangeType result(this->result_size(),this->_models.zero_element().range());
    for(SizeType i=0; i!=result.size(); ++i) {
        result[i]=this->_models[i].range();
    }
    return result;
}


template<class M> const Vector<typename VectorScaledFunctionPatch<M>::CoefficientType> VectorScaledFunctionPatch<M>::centre() const
{
    Vector<CoefficientType> result(this->result_size(),this->precision());
    for(SizeType i=0; i!=result.size(); ++i) {
        result[i]=this->_models[i].value();
    }
    return result;
}


template<class M> const Vector<typename VectorScaledFunctionPatch<M>::ModelType>& VectorScaledFunctionPatch<M>::models() const
{
    return this->_models;
}

template<class M> Vector<typename VectorScaledFunctionPatch<M>::ModelType>& VectorScaledFunctionPatch<M>::models()
{
    return this->_models;
}

template<class M> const typename VectorScaledFunctionPatch<M>::ModelType& VectorScaledFunctionPatch<M>::model(SizeType i) const
{
    return this->_models[i];
}

template<class M> typename VectorScaledFunctionPatch<M>::ModelType& VectorScaledFunctionPatch<M>::model(SizeType i)
{
    return this->_models[i];
}




template<class M> SizeType VectorScaledFunctionPatch<M>::argument_size() const
{
    return this->_domain.size();
}


template<class M> SizeType VectorScaledFunctionPatch<M>::result_size() const
{
    return this->_models.size();
}


template<class M> SizeType VectorScaledFunctionPatch<M>::size() const
{
    return this->_models.size();
}

template<class M> ScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::zero_element() const
{
    return ScaledFunctionPatch<M>(this->_domain,this->_models.zero_element());
}

template<class M> ScaledFunctionPatch<M> const VectorScaledFunctionPatch<M>::operator[](SizeType i) const
{
    return this->get(i);
}

template<class M> VectorScaledFunctionPatchElementReference<M> VectorScaledFunctionPatch<M>::operator[](SizeType i)
{
    return VectorScaledFunctionPatchElementReference<M>(*this,i);
}

template<class M> ScaledFunctionPatch<M> VectorScaledFunctionPatch<M>::get(SizeType i) const
{
    return ScaledFunctionPatch<M>(this->_domain,this->_models[i]);
}

template<class M> Void VectorScaledFunctionPatch<M>::set(SizeType i, const ScaledFunctionPatch<M>& e)
{
    ARIADNE_ASSERT_MSG(this->size()>i,"Cannot set "<<i<<"th element of VectorScaledFunctionPatch<M> "<<(*this));
    if(this->domain().size()!=0) {
        ARIADNE_ASSERT_MSG(e.domain()==this->domain(),"Domain of "<<e<<" conflicts with existing domain "<<this->domain());
    } else {
        this->_domain=e.domain();
    }
    this->_models[i]=e.model();
}



template<class M> VectorScaledFunctionPatch<M>& VectorScaledFunctionPatch<M>::simplify()
{
    for(SizeType i=0; i!=this->size(); ++i) {
        this->_models[i].simplify();
    }
    return *this;
}

template<class M> VectorScaledFunctionPatch<M>& VectorScaledFunctionPatch<M>::simplify(const PropertiesType& properties)
{
    for(SizeType i=0; i!=this->size(); ++i) {
        this->_models[i].simplify(properties);
    }
    return *this;
}


template<class M> Void VectorScaledFunctionPatch<M>::clobber()
{
    for(SizeType i=0; i!=this->size(); ++i) {
        this->_models[i].clobber();
    }
}





template<class M> auto VectorScaledFunctionPatch<M>::operator()(const Vector<FloatApproximation<PR>>& x) const
    -> Vector<ArithmeticType<CoefficientType,FloatApproximation<PR>>>
{
    const VectorScaledFunctionPatch<M>& f=*this;
    if(!decide(contains(f.domain(),x))) {
        ARIADNE_THROW(DomainException,"tf.evaluate(ax) with tf="<<f<<", ax="<<x,"ax is not an element of tf.domain()="<<f.domain());
    }
    Vector<FloatApproximation<PR>> sx=Ariadne::unscale(x,f._domain);
    Vector<ArithmeticType<CoefficientType,FloatApproximation<PR>>> r(this->result_size(),x.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=Ariadne::evaluate(this->_models[i].expansion(),sx);
    }
    return r;
}

template<class M> auto VectorScaledFunctionPatch<M>::operator()(const Vector<FloatBounds<PR>>& x) const
    -> Vector<ArithmeticType<CoefficientType,FloatBounds<PR>>>
{
    const VectorScaledFunctionPatch<M>& f=*this;
    if(!definitely(contains(f.domain(),x))) {
        ARIADNE_THROW(DomainException,"tf.evaluate(vx) with tf="<<f<<", x="<<x,"vx is not a definitely an element of tf.domain()="<<f.domain());
    }
    Vector<FloatBounds<PR>> sx=Ariadne::unscale(x,f._domain);
    return Ariadne::evaluate(f._models,sx);
}

template<class M> auto VectorScaledFunctionPatch<M>::operator()(const Vector<ValidatedNumber>& y) const
    -> Vector<ArithmeticType<CoefficientType,ValidatedNumber>>
{
    if constexpr (AnInterval<CoefficientType>) {
        ARIADNE_FAIL_MSG("Cannot evaluate an IntervalTaylorModel on a generic Number");
    } else {
        return this->operator()(Vector<FloatBounds<PR>>(y,this->precision()));
    }
}

template<class M> auto VectorScaledFunctionPatch<M>::jacobian(const Vector<NumericType>& x) const -> Matrix<NumericType>
{
    Vector<NumericType> y=unscale(x,this->_domain);
    Matrix<NumericType> J(this->size(),x.size(),x.zero_element());
    for(SizeType i=0; i!=J.row_size(); ++i) {
        J[i]=gradient(this->_models[i],y);
    }
    for(SizeType j=0; j!=J.column_size(); ++j) {
        auto r=rad(this->_domain[j]);
        for(SizeType i=0; i!=J.row_size(); ++i) {
            J[i][j]/=r;
        }
    }
    return J;
}

template<class M> Void VectorScaledFunctionPatch<M>::restrict(const BoxDomainType& x)
{
    *this=restriction(*this,x);
}



template<class M> OutputStream& VectorScaledFunctionPatch<M>::_write(OutputStream& os) const
{
    os << "VectorFunctionPatch";
    os << "( result_size="<<this->result_size()<<", dom=" << this->domain() << ", rng=" << this->range() << ", f=";
    os << "[ ";
    for(SizeType i=0; i!=this->result_size(); ++i) {
        if(i!=0) { os << ", "; }
        write_polynomial(os,(*this)[i]);
    }
    os << " ] )";
    return os;
}

template<class M> OutputStream& VectorScaledFunctionPatch<M>::repr(OutputStream& os) const
{
    return os << "VectorScaledFunctionPatch<M>("
              << representation(this->domain()) << ", " << representation(this->expansions()) << ", "
              << representation(this->errors()) << ", " << representation(this->properties()) << ")";
}





template<class M> auto ScaledFunctionPatchFactory<M>::create(const Number<P>& number) const -> CanonicalNumericType<P,PR,PRE> {
    return CanonicalNumericType<P,PR>(number,this->_properties.precision());
}
template<class M> ScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create(const DomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const {
    return ScaledFunctionPatch<M>(domain,function,this->_properties);
}
template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create(const DomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const {
    return VectorScaledFunctionPatch<M>(domain,function,this->_properties);
}
template<class M> ScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_zero(const DomainType& domain) const {
    return ScaledFunctionPatch<M>(domain,this->_properties);
}
template<class M> ScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_constant(const DomainType& domain, GenericNumericType const& value) const {
    auto concrete_value=this->create(value);
    return ScaledFunctionPatch<M>::constant(domain,concrete_value,this->_properties);
}
template<class M> ScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_coordinate(const DomainType& domain, SizeType index) const {
    return ScaledFunctionPatch<M>::coordinate(domain,index,this->_properties);
}
template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_zeros(SizeType rsize, const DomainType& domain) const {
    return VectorScaledFunctionPatch<M>(rsize,domain,this->_properties);
}
template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_constants(const DomainType& domain, Vector<Number<P>> const& values) const {
    Vector<NumericType> concrete_values(values.size(),this->_properties.precision());
    for(SizeType i=0; i!=values.size(); ++i) { concrete_values[i]=values[i]; }
    return VectorScaledFunctionPatch<M>::constant(domain,concrete_values,this->_properties);
}
template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_projection(const DomainType& domain, Range indices) const {
    return VectorScaledFunctionPatch<M>::projection(domain,indices,this->_properties);
}
template<class M> VectorScaledFunctionPatch<M> ScaledFunctionPatchFactory<M>::create_identity(const DomainType& domain) const {
    return VectorScaledFunctionPatch<M>::identity(domain,this->_properties);
}





} // namespace Ariadne

#endif
