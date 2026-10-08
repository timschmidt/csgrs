/***************************************************************************
 *            algebra/differential.hpp
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

/*! \file algebra/differential.hpp
 *  \brief Differential algebra variables with a sparse representation.
 */

#ifndef ARIADNE_DIFFERENTIAL_HPP
#define ARIADNE_DIFFERENTIAL_HPP

#include <map>

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/series.hpp"
#include "algebra/expansion.hpp"
#include "algebra/operations.hpp"

#include "differential.decl.hpp"
#include "univariate_differential.hpp"

namespace Ariadne {

template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Series;

template<class I, class X> class Expansion;
template<class X> class Differential;
template<class X> class UnivariateDifferential;
template<class X> class NonAssignableDifferential;

template<class X> class DifferentialFactory {
    typedef PrecisionType<X> PR;
    PR _pr;
  public:
    DifferentialFactory(PR const& pr) : _pr(pr) { }

    template<class Y> requires Constructible<X,Y,PR> X create(Y const& y) { return X(y,_pr); }
    template<class Y> UnivariateDifferential<X> create(UnivariateDifferential<Y> const&);
    template<class Y> Differential<X> create(Differential<Y> const&);
    Differential<X>const& create(NonAssignableDifferential<X> const& dx) { return dx; }
};
template<class X> inline DifferentialFactory<X> differential_factory(X const& x) { return DifferentialFactory<X>(x.precision()); }
template<class X> inline DifferentialFactory<X> factory(UnivariateDifferential<X> const& dx) {
    return DifferentialFactory<X>(dx.value().precision()); }
template<class X> inline DifferentialFactory<X> factory(Differential<X> const& dx) {
    return DifferentialFactory<X>(dx.value().precision()); }





//! \ingroup DifferentiationSubModule
//! \brief A class representing the partial derivatives of a scalar quantity
//! depending on multiple arguments.
template<class X> class DifferentialCharacteristics
    : public Tuple<SizeType,DegreeType,CharacteristicsType<X>>
{
  public:
    DifferentialCharacteristics(SizeType as, DegreeType deg, CharacteristicsType<X> prs)
        : Tuple<SizeType,DegreeType,CharacteristicsType<X>>(as,deg,prs) { }
    DifferentialCharacteristics(SizeType as, DegreeType deg, X const& z)
        : DifferentialCharacteristics(as,deg,characteristics(z)) { }
    template<class... PRS> requires Constructible<X,Nat,PRS...>
    DifferentialCharacteristics(SizeType as, DegreeType deg, PRS... prs)
        : DifferentialCharacteristics(as,deg,X(0,prs...)) { }
    DifferentialCharacteristics(const Differential<X>& d)
        : DifferentialCharacteristics(d.argument_size(),d.degree(),d.coefficient_characteristics()) { }
    template<class XX> DifferentialCharacteristics(DifferentialCharacteristics<XX> chrs)
        : DifferentialCharacteristics(chrs.argument_size(),chrs.degree(), precision(chrs.zero_coefficient())) { }
    SizeType argument_size() const { return std::get<0>(*this); }
    DegreeType degree() const { return std::get<1>(*this); }
    CharacteristicsType<X> coefficient_characteristics() const { return std::get<2>(*this); }
    X zero_coefficient() const { return X(std::get<2>(*this)); }
};

//!
//! Based on a power series Expansion, centred on the point at which the partial derivatives are
//! being evaluated.
//!
//! \invariant The expansion is sorted using graded_sort(). In particular, the
//! total degree of the terms is increasing, and the linear terms appear in coordinate order.
template<class X>
class Differential
    : public DispatchTranscendentalAlgebraOperations<Differential<X>,X>
    , public DispatchLatticeAlgebraOperations<Differential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<Differential<X>>
{
    typedef Differential<X> SelfType;

    static const DegreeType MAX_DEGREE=65535;

    SortedExpansion<MultiIndex,X,GradedIndexLess> _expansion;
    DegreeType _degree;
  public:
    //! \brief The type used for a power series.
    typedef UnivariateDifferential<X> SeriesType;
    //! \brief The comparison used to sort the coefficients.
    typedef GradedLess IndexComparisonType;
    //! \brief The type of used to represent numbers in the coefficient.
    typedef SortedExpansion<MultiIndex,X,GradedIndexLess> ExpansionType;
    //! \brief The kind of information provided by the concrete values.
    typedef typename X::Paradigm Paradigm;
    //! \brief The type of used to represent numbers in the coefficient.
    typedef typename X::NumericType NumericType;
    //! \brief The type of used to represent the coefficients.
    typedef X ValueType;
    //! \brief The type of an Iterator through (index,coefficient) pairs..
    typedef typename ExpansionType::Iterator Iterator;
    //! \brief The type of a constant Iterator through (index,coefficient) pairs..
    typedef typename ExpansionType::ConstIterator ConstIterator;

    //! \brief Default constructor constructs a differential with degree zero in no variables.
    Differential() = delete;
    //! \brief Constructs a differential with degree \a deg in \a as variables, and zero element based on \a prs.
    template<class... PRS> requires Constructible<X,Nat,PRS...> explicit Differential(SizeType as, DegreeType deg, PRS... prs)
        : Differential(as,deg,X(0u,prs...)) { }
    explicit Differential(SizeType as, DegreeType deg, X const& z);
    template<class XX> explicit Differential(DifferentialCharacteristics<XX> chrs);
    //! \brief Construct a differential from a mapping giving a coefficient for a finite number of multi-indices.
    explicit Differential(const Map<MultiIndex,X>& map, DegreeType deg);
    //! \brief Construct a differential of degree \a deg from the power-series expansion \a e.
    //! Terms in \a e of degree higher than \a deg are truncated
    explicit Differential(const Expansion<MultiIndex,X>& e, DegreeType deg);
    //! \brief Construct a differential of degree \a deg from an initializer list of (index,coefficient) pairs.
    explicit Differential(SizeType as, DegreeType deg, InitializerList< Pair<InitializerList<DegreeType>,X> > lst);
    //! \brief Construct a differential of degree \a deg from an initializer list of (index,coefficient) pairs.
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        explicit Differential(SizeType as, DegreeType deg, InitializerList< Pair<InitializerList<DegreeType>,ExactDouble> > lst, PRS... pr);
    //! \brief Construct a dense differential of degree \a deg from an initializer list of coefficient arranged in graded lexicographic order of the index.
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        explicit Differential(SizeType as, DegreeType deg, InitializerList<ExactDouble> lst, PRS... prs);

    //! \brief Conversion constructor from a different numerical type.
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...> explicit Differential(const Differential<Y>& dy, PRS... prs);

    //! \brief Set the differential equal to a constant, without changing the degree or number of arguments.
    Differential<X>& operator=(const X& c);
    template<AssignableTo<X> W>
        Differential<X>& operator=(const W& c) { X xc=nul(this->value()); xc=c; return (*this)=xc; }

    //! \brief A constant differential of degree \a deg in \a as arguments with value \a c.
    static Differential<X> constant(SizeType as, DegreeType deg, const X& c);
    //! \brief A differential of degree \a deg in \a as arguments representing the quantity \f$v+x_j\f$.
    static Differential<X> variable(SizeType as, DegreeType deg, const X& v, SizeType j);
    //! \brief A differential of degree \a deg in \a as arguments representing a quantity with value \a v and gradient \a g.
    static Differential<X> affine(SizeType as, DegreeType d, const X& v, const Covector<X>& g);
    static Differential<X> affine(DegreeType d, const X& v, const Covector<X>& g);

    //! \brief A vector of \a rs constant differentials of degree \a deg in \a as arguments with values \a c[i].
    static Vector< Differential<X> > constants(SizeType rs, SizeType as, DegreeType deg, const Vector<X>& c);
    static Vector< Differential<X> > constants(SizeType as, DegreeType deg, const Vector<X>& c);
    //! \brief A vector of \a rs differentials of degree \a deg in \a as arguments with values \f$v_i+dx_i\f$.
    //! \pre \a rs == \a as == c.size().
    static Vector< Differential<X> > variables(SizeType rs, SizeType as, DegreeType deg, const Vector<X>& v);
    static Vector< Differential<X> > variables(DegreeType deg, const Vector<X>& v);
    //! \brief A vector of \a rs affine differentials of degree \a deg in \a as arguments with values \a v[i] and gradients \a G[i][j].
    static Vector<Differential<X>> affine(SizeType rs, SizeType as, DegreeType deg, const Vector<X>& v, const Matrix<X>& G);
    static Vector<Differential<X>> affine(DegreeType deg, const Vector<X>& v, const Matrix<X>& G);

    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static Differential<X> constant(SizeType as, DegreeType deg, const Y& c, const PRS&... prs) {
        return constant(as,deg,X(c,prs...)); }
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static Differential<X> variable(SizeType as, DegreeType deg, const Y& v, SizeType j, const PRS&... prs) {
        return variable(as,deg,X(v,prs...),j); }
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static Vector< Differential<X> > constants(SizeType as, DegreeType deg, const Vector<Y>& c, const PRS&... prs) {
        return constants(as,deg,Vector<X>(c,prs...)); }
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static Vector< Differential<X> > variables(DegreeType deg, const Vector<Y>& v, const PRS&... prs) {
        return variables(deg,Vector<X>(v,prs...)); }

//    static UnivariateDifferential<X> identity(DegreeType deg, const Vector<X>& v);
    static Differential<X> identity(DegreeType deg, const X& v); // FIXME: Should use UnivariateDifferentia; required for derivative of UnivariateFunction.
    static Vector<Differential<X>> identity(DegreeType deg, const Vector<X>& v);

    //! \brief Equality operator. Tests equality of representation, so comparing two differentials which are mathematically equal may return false if the structural zeros are different.
    EqualityType<X> operator==(const Differential<X>& other) const;
    //! \brief Inequality operator.
    InequalityType<X> operator!=(const Differential<X>& other) const;


    //! \brief The number of independent variables.
    SizeType argument_size() const;
    //! \brief The maximum degree of the stored terms.
    DegreeType degree() const;
    //! \brief The precision argument of the numbers.
    Ariadne::CharacteristicsType<X> coefficient_characteristics() const;
    //! \brief A zero with the precision argument of the numbers.
    X zero_coefficient() const;
    //! \brief The internal representation of the polynomial expansion.
    const Expansion<MultiIndex,X>& expansion() const;
    //! \brief A reference to the internal representation of the polynomial expansion.
    Expansion<MultiIndex,X>& expansion();
    //! \brief The value of the differential i.e. the coefficient of \f$1\f$.
    const X& value() const;
    //! \brief The coefficient of \f$x_j\f$.
    const X& gradient(SizeType j) const;
    //! \brief The vector of coefficients of \f$x_j\f$.
    Covector<X> gradient() const;
    //! \brief The Hessian matrix.
    //! \note Note the the components of the Hessian matrix are \em half those of the values indexed by the differential.
    //! This is because the differential stores the coefficients of the Taylor expansion, rather than the derivatives themselves.
    Matrix<X> hessian() const;
    Matrix<X> half_hessian() const;

    //! \brief A reference to the coefficient of \f$x_j\f$.
    X& operator[](const SizeType& j);
    //! \brief The coefficient of \f$x_j\f$.
    const X& operator[](const SizeType& j) const;
    //! \brief The coefficient of \f$x^a\f$ i.e. \f$\prod x_j^{a_j}\f$.
    X& operator[](const MultiIndex& a);
    //! \brief A reference to the coefficient of \f$x^a\f$.
    const X& operator[](const MultiIndex& a) const;

    //! \brief Set the degree to be equal to \a d.
    Void set_degree(DegreeType d);
    //! \brief Set the coefficient of the constant term to \a c.
    Void set_value(const X& c);
    //! \brief Set the coefficient of the term in \f$x_j\f$ to \a d.
    Void set_gradient(SizeType j, const X& d);

    //! \brief An Iterator to the first structural nonzero.
    Iterator begin();
    //! \brief An Iterator to past-the-end structural nonzero.
    Iterator end();
    //! \brief A constant Iterator to the first structural nonzero.
    ConstIterator begin() const;
    //! \brief A constant Iterator to past-the-end structural nonzero.
    ConstIterator end() const;

    //! \brief The zero element of the differential algebra.
    Differential<X> create() const;
    Differential<X> create_zero() const;
    Differential<X> create_constant(const NumericType& c) const;
    Differential<X> create_constant(Int c) const;
    //! \brief Set all coefficients to zero.
    Void clear();
    //! \brief Remove all terms with coefficient \f$0\f$.
    Void cleanup();
    //! \brief Check that differential is sorted and all terms have degree less than maximum degree.
    Void check() const;

  public:
    //! \brief The derivatives of the composition of functions.
    template<class XX> friend Differential<XX> compose(UnivariateDifferential<XX> const& dx, Differential<XX> const& dy);
    //! \brief The derivatives of the composition of functions.
    template<class XX> friend Differential<XX> compose(Differential<XX> const& dx, Vector<Differential<XX>> const& dy);
    //! \brief The derivatives of the composition of functions.
    template<class XX> friend Vector<Differential<X>> compose(Vector<Differential<X>> const& dx, Vector<Differential<X>> const& dy);
    //! \brief Compute the differential of the derivative.
    template<class XX> friend Differential<XX> derivative(Differential<XX> const& dx, SizeType k);
    //! \brief Compute an antiderivative with respect to the variable \a i.
    template<class XX> friend Differential<XX> antiderivative(Differential<XX> const& dx, SizeType k);

    friend X value(const Differential<X>& x) { return x.value(); }
    friend Covector<X> gradient(const Differential<X>& x) { return x.gradient(); }
    friend Matrix<X> hessian(const Differential<X>& x) { return x.hessian(); }
    friend Matrix<X> half_hessian(const Differential<X>& x) { return x.half_hessian(); }

    friend OutputStream& operator<<(OutputStream& os, Differential<X> const& dx) { return dx._write(os); }
  public:
    friend DifferentialCharacteristics<X> characteristics(Differential<X> const& dx) {
        return DifferentialCharacteristics<X>(dx.argument_size(),dx.degree(),nul(dx.value())); }
/*
    friend Differential<X> min(const Differential<X>& x1, const Differential<X>& x2) {
        return AlgebraOperations<Differential<X>>::min(x1,x2); }
    friend Differential<X> max(const Differential<X>& x1, const Differential<X>& x2) {
        return AlgebraOperations<Differential<X>>::max(x1,x2); }
    friend Differential<X> abs(const Differential<X>& x) {
        return AlgebraOperations<Differential<X>>::abs(x); }
*/
    friend Differential<X> compose(UnivariateDifferential<X> const& x, Differential<X> const& y) {
        return Differential<X>::_compose(x,y); }
    friend Differential<X> compose(Differential<X> const& x, Vector<Differential<X>> const& y) {
        return Differential<X>::_compose(x,y); }
    friend Differential<X> derivative(Differential<X> const& x, SizeType k) { return
        Differential<X>::_derivative(x,k); }
    friend Differential<X> antiderivative(Differential<X> const& x, SizeType k) { return
        Differential<X>::_antiderivative(x,k); }

  public:
    static Differential<X> _derivative(Differential<X> const& dx, SizeType k);
    static Differential<X> _antiderivative(Differential<X> const& dx, SizeType k);
    static Differential<X> _compose(Series<X> const& f, Differential<X> const& dx) {
        return _compose(UnivariateDifferential<X>(dx.degree(),f),dx); }
    static Differential<X> _compose(UnivariateDifferential<X> const&, Differential<X> const&);
    static Differential<X> _compose(Differential<X> const&, Vector<Differential<X>> const&);
    static Vector<Differential<X>> _compose(Vector<Differential<X>> const&, Vector<Differential<X>> const&);

    static Matrix<X> _jacobian(Vector<Differential<X>> const&);
    OutputStream& _write(OutputStream& os) const;
};

template<class X, class Y> requires ConcreteNumber<X> and GenericNumber<Y>
decltype(auto) operator+(Differential<X> const& x, Y const& y) { return x+factory(x).create(y); }

template<class X> struct AlgebraOperations<Differential<X>> : GradedAlgebraOperations<Differential<X>> {
    static Differential<X> apply(UnaryElementaryOperator op, Differential<X> dx) {
        UnivariateDifferential<X> dop = op.accept([&dx](auto o){return UnivariateDifferential<X>(o,dx.degree(),dx.value());});
        return compose(dop,dx); }

    template<class OP> static Differential<X> apply(OP op, Differential<X> dx) {
        return compose(UnivariateDifferential<X>(op,dx.degree(),dx.value()),dx); }
    static Differential<X> apply(Pos op, Differential<X> dx);
    static Differential<X> apply(Neg op, Differential<X> dx);
    static Differential<X> apply(Add op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Sub op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Mul op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Div op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Pow op, Differential<X> const& dx, Int n);
    static Differential<X> apply(Add op, Differential<X> dx, X const& c);
    static Differential<X> apply(Mul op, Differential<X> dx, X const& c);
    static Differential<X> apply(Min op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Max op, Differential<X> const& dx1, Differential<X> const& dx2);
    static Differential<X> apply(Abs op, Differential<X> const& dx);
};


template<class X> template<class XX> Differential<X>::Differential(DifferentialCharacteristics<XX> c)
    : Differential(c.argument_size(),c.degree(),c.zero_coefficient()) { }

template<class X> template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
Differential<X>::Differential(const Differential<Y>& x, PRS... prs)
    : _expansion(x.expansion(),prs...), _degree(x.degree())
{
}

template<class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Differential<X>::Differential(SizeType as, DegreeType deg, InitializerList< Pair<InitializerList<DegreeType>,ExactDouble> > lst, PRS... prs)
    : Differential<X>(Expansion<MultiIndex,X>(lst,prs...),deg)
{ }

template<class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Differential<X>::Differential(SizeType as, DegreeType deg, InitializerList<ExactDouble> lst, PRS... prs) : _expansion(as,X(prs...)), _degree(deg) {
    auto iter = lst.begin();
    for(MultiIndex j(as); j.degree()<=deg; ++j) {
        ExactDouble const& y=*iter; if(y!=0.0_x) { _expansion.append(j,X(y,prs...)); } ++iter; }
    this->cleanup();
}


template<class X> Vector<Differential<X>> compose(Vector<Differential<X>> const&, Vector<Differential<X>> const&);
template<class X> Vector<Differential<X>> derivative(Vector<Differential<X>> const&, SizeType k);
template<class X> Vector<Differential<X>> antiderivative(Vector<Differential<X>> const&, SizeType k);
template<class X> Vector<X> value(const Vector<Differential<X>>& x);
template<class X> Matrix<X> jacobian(Vector<Differential<X>> const&);

template<class X> Vector<Differential<X>> lie_derivative(const Vector<Differential<X> >& df, const Vector<Differential<X> >& dg);




template<class X>
class NonAssignableDifferential
    : public Differential<X>
{
  public:
    NonAssignableDifferential<X>& operator=(const Differential<X>& other) {
        ARIADNE_PRECONDITION_MSG( this->argument_size()==other.argument_size(),
                                  "\n  self="<<*this<<"\n  other="<<other<<"\n"<<std::flush  );
        ARIADNE_PRECONDITION( this->degree()==other.degree() );
        this->Differential<X>::operator=(other); return *this; }
    NonAssignableDifferential<X>& operator=(const X& c) {
        this->Differential<X>::operator=(c); return *this; }
    NonAssignableDifferential<X>& operator+=(const Differential<X>& x) {
        static_cast<Differential<X>&>(*this) += x; return *this; }
    // FIXME: Operators below should not be necessary
    friend Differential<X> operator+(const NonAssignableDifferential<X>& dx1, const NonAssignableDifferential<X>& dx2) {
        return static_cast<Differential<X>const&>(dx1) + static_cast<Differential<X>const&>(dx2); }
    friend Differential<X> operator*(const NonAssignableDifferential<X>& dx1, const NonAssignableDifferential<X>& dx2) {
        return static_cast<Differential<X>const&>(dx1) * static_cast<Differential<X>const&>(dx2); }
};

//! \brief A class representing the derivatives of a vector quantity depending on multiple arguments.
template<class X>
class Vector< Differential<X> >
    : public VectorExpression< Vector< Differential<X> > >
    , ProvideVectorOperations
{
    //BOOST_CONCEPT_ASSERT((DifferentialVectorConcept<DifferentialVector<X> >));
  public:
    UniformArray< Differential<X> > _ary;
  public:
    // The type of the class
    typedef Vector< Differential<X> > SelfType;
    // The type used for accessing elements
    typedef SizeType IndexType;
    // The value stored in the vector.
    typedef Differential<X> ValueType;
    // The type used for scalars.
    typedef Differential<X> ScalarType;
    // The type used for scalars.
    typedef X NumericType;

    Vector() = delete;
    Vector(SizeType rs) = delete;
    template<class... PRS> requires Constructible<X,Nat,PRS...> explicit Vector(SizeType rs, SizeType as, DegreeType d, PRS... prs)
        : Vector<Differential<X>>(rs,as,d,X(0u,prs...)) { }
    Vector(SizeType rs, SizeType as, DegreeType d, X const& z);
    Vector(SizeType rs, const Differential<X>& sd);
    Vector(InitializerList<Differential<X>> const& lst);
    Vector(Array<Differential<X>> ary);
    template<class E> Vector(const VectorExpression<E>& ve);
    template<class E> Vector< Differential<X> >& operator=(const VectorExpression<E>& ve);
    template<class G> requires InvocableReturning<Differential<X>,G,SizeType> Vector(SizeType n, G const& g);
    template<class G, class... PRS> requires InvocableReturning<Differential<X>,G,SizeType> and Constructible<Differential<X>,PRS...>
        Vector(SizeType n, G const& g, PRS... prs);
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...> explicit Vector(const Vector<Differential<Y>>& dv, PRS... prs);

    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        Vector(SizeType rs, SizeType as, DegreeType d, InitializerList<InitializerList<Pair<InitializerList<DegreeType>,ExactDouble>>> lst, PRS... prs);
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        explicit Vector(SizeType rs, SizeType as, DegreeType deg, InitializerList<InitializerList<ExactDouble>> lst, PRS... prs);

    const Differential<X>& operator[](SizeType i) const { return this->_ary[i]; }
    NonAssignableDifferential<X>& operator[](SizeType i) { return static_cast<NonAssignableDifferential<X>&>(static_cast<Differential<X>&>(_ary[i])); }

    VectorRange<const Vector<Differential<X>>> operator[](Range rng) const { return project(*this,rng); }

    const Differential<X> zero_element() const { return Differential<X>::constant(this->argument_size(),this->degree(),this->zero_coefficient()); }
    const DifferentialCharacteristics<X> element_characteristics() const { return this->_ary.element_characteristics(); }

    NonAssignableDifferential<X>& at(SizeType i) { return static_cast<NonAssignableDifferential<X>&>(static_cast<Differential<X>&>(_ary[i])); }
    const Differential<X>& at(SizeType i) const { return this->_ary[i]; }
    const Differential<X>& get(SizeType i) const { return this->_ary[i]; }
    Void set(SizeType i, const Differential<X>& x) {
        ARIADNE_PRECONDITION(i<this->size()); ARIADNE_PRECONDITION(this->argument_size()==x.argument_size()); this->_ary[i]=x; }

    SizeType size() const { return this->_ary.size(); }
    SizeType result_size() const { return this->size(); }
    SizeType argument_size() const { return this->_ary.element_characteristics().argument_size(); }
    DegreeType degree() const { return this->_ary.element_characteristics().degree(); }
    X zero_coefficient() const { return this->_ary.element_characteristics().zero_coefficient(); }
    const UniformArray<Differential<X>>& array() const { return _ary; }

    Vector<X> value() const;
    Matrix<X> jacobian() const;

    Void set_value(const Vector<X>& c);

    //! \brief Set the degree to be equal to \a d.
    Void set_degree(DegreeType d);

    friend Vector<X> value(const Vector<Differential<X>>& x) { return x.value(); }
    friend Matrix<X> jacobian(const Vector<Differential<X>>& x) { return x.jacobian(); }
    //! \brief Compute the differential of the composiition \f$f\circ g\f$.
    friend Vector<Differential<X>> compose(const Vector<Differential<X>>& x, Vector<Differential<X>> const& y) { return _compose(x,y); }
    //! \brief Compute the differential of the composiition \f$f\circ g\f$.
    friend Vector<Differential<X>> derivative(const Vector<Differential<X>>& x, SizeType k) { return _derivative(x,k); }
    //! \brief Compute the differential of the antiderivative \f$ \int^{x_k} f(x_0,\ldots,x_{k-1},\xi,x_{k+1},\ldots) d\xi\f$.
    friend Vector<Differential<X>> antiderivative(const Vector<Differential<X>>& x, SizeType k) { return _antiderivative(x,k); }
    //! \brief Compute the differential of the Lie derivative \f$L_fg(x) = f(x) \cdot \nabla{g}(x)\f$.
    friend Vector<Differential<X>> lie_derivative(const Vector<Differential<X> >& df, const Vector<Differential<X> >& dg) { return _lie_derivative(df,dg); }

    //! \brief Compute the differential of the solutions \f$y=h(x)\f$ to \f$f(x,y)=0\f$. Requires \f$f(x0,y0)=0\f$.
    friend Vector<Differential<X>> solve(const Vector<Differential<X> >& df, const Vector<X>& y0) { return _solve(df,y0); }

    //! \brief Compute the differential of the autonomous flow \f$ \phi_{,t}(x_0,t) = f(\phi(x_0,t)) \f$, \f$\phi(x_0,t_0)=x_0\f$ at \f$t=t_0\f$, with respect to the independent variables \f$(x_0,t)\f$.
    friend Vector<Differential<X>> flow(const Vector<Differential<X> >& df, const Vector<X>& x0) { return _flow(df,x0); }
    //! \brief Compute the differential of the autonomous flow \f$ \phi_{,t}(x_0,t,a) = f(\phi(x_0,t),a) \f$, \f$\phi(x_0,t_0,a)=x_0\f$ at \f$t=t_0\f$, with respect to the independent variables \f$(x_0,t,a)\f$.
    friend Vector<Differential<X>> flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const Vector<X>& a) { return _flow(df,x0,a); }
    //! \brief Compute the differential of the flow \f$ \phi_{,t}(x_0,t) = f(\phi(x_0,t),t) \f$, \f$\phi(x_0,t_0)=x_0\f$ at \f$t=t_0\f$, with respect to the independent variables \f$(x_0,t)\f$.
    friend Vector<Differential<X>> flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const X& t0) {
        return _flow(df,x0,t0); }
    //! \brief Compute the differential of the flow \f$ \phi_{,t}(x_0,t,a) = f(\phi(x_0,t,a),t,a) \f$ at \f$t=t_0\f$, with respect to the independent variables \f$(x_0,t,a)\f$.
    friend Vector<Differential<X>> flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const X& t0, const Vector<X>& a) {
        return _flow(df,x0,t0,a); }

    //! \brief Given the derivatives of \f$f:\R^{n+a}\to\R^n\f$ with respect to \f$x,a\f$,
    //! and the derivatives of \f$x_0,a\f$ with respect to independent variables \f$(x_0,t,b)\f$, the \f$n^\mathrm{th}\f$ of which is time \f$t\f$,
    //! compute the derivatives of \f$\phi\f$ satisfying \f$\phi_{,t}(x_0,t,b)=f(\phi(x_0,t,b),a)\f$ and \f$\phi(x_0,t_0,a)=x_0\f$.
    Vector<Differential<X>> flow(const Vector<Differential<X> >& df, const Vector<Differential<X>>& dx0, const Vector<Differential<X>>& da) {
        return _flow(df,dx0,da); }

    friend OutputStream& operator<<(OutputStream& os, const Vector<Differential<X> >& x) { return x._write(os); }
  public:
    static Vector<Differential<X>> _compose(Vector<Differential<X>> const& x, Vector<Differential<X>> const& y);
    static Vector<Differential<X>> _derivative(Vector<Differential<X>> const& x, SizeType k);
    static Vector<Differential<X>> _antiderivative(Vector<Differential<X>> const& x, SizeType k);
    static Vector<Differential<X>> _lie_derivative(const Vector<Differential<X> >& df, const Vector<Differential<X> >& dg);
    static Vector<Differential<X>> _solve(const Vector<Differential<X> >& df, const Vector<X>& y0);
    static Vector<Differential<X>> _flow(const Vector<Differential<X> >& df, const Vector<X>& x0);
    static Vector<Differential<X>> _flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const X& t0);
    static Vector<Differential<X>> _flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const Vector<X>& a);
    static Vector<Differential<X>> _flow(const Vector<Differential<X> >& df, const Vector<X>& x0, const X& t0, const Vector<X>& a);
    static Vector<Differential<X>> _flow(const Vector<Differential<X> >& df, const Vector<Differential<X>>& dx0, const Vector<Differential<X>>& dt0a);
    OutputStream& _write(OutputStream& os) const;
};

template<class X> template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
Vector<Differential<X>>::Vector(const Vector<Differential<Y>>& dv, PRS... prs)
    : _ary(dv._ary,prs...) {
}

template<class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Vector<Differential<X>>::Vector(SizeType rs, SizeType as, DegreeType d, InitializerList<InitializerList<ExactDouble>> lst, PRS... prs)
        : _ary(rs,DifferentialCharacteristics<X>(as,d,prs...)) {
    auto iter=lst.begin();
    for(SizeType i=0; i!=rs; ++i) { _ary[i]=Differential<X>(as,d,*iter,prs...); ++iter; }
}

template<class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Vector<Differential<X>>::Vector(SizeType rs, SizeType as, DegreeType d, InitializerList<InitializerList<Pair<InitializerList<DegreeType>,ExactDouble>>> lst, PRS... prs)
        : _ary(rs,DifferentialCharacteristics<X>(as,d,prs...)) {
    auto iter=lst.begin();
    for(SizeType i=0; i!=rs; ++i) { _ary[i]=Differential<X>(as,d,*iter,prs...); ++iter; }
}

template<class X> template<class G> requires InvocableReturning<Differential<X>,G,SizeType>
Vector<Differential<X>>::Vector(SizeType n, G const& g)
    : _ary(n,g,(assert(n>0),g(0))) { }

template<class X> template<class G, class... PRS> requires InvocableReturning<Differential<X>,G,SizeType> and Constructible<Differential<X>,PRS...>
Vector<Differential<X>>::Vector(SizeType n, G const& g, PRS... prs)
    : _ary(n,g,prs...) {
}

template<class X> template<class E>
Vector<Differential<X>>::Vector(const VectorExpression<E>& ve)
    : _ary(ve().size(), ((assert(ve().size()>0),characteristics(ve()[0u]))),Uninitialised())
{
    for(SizeType i=0; i!=_ary.size(); ++i) { new (&static_cast<Differential<X>&>(_ary[i])) Differential<X>(ve()[i]); }
    ARIADNE_ASSERT(_ary.size()!=0);
}

template<class X> template<class E>
Vector<Differential<X>>& Vector<Differential<X>>::operator=(const VectorExpression<E>& ve) {
    _ary.resize(ve().size()); for(SizeType i=0; i!=_ary.size(); ++i) { static_cast<Differential<X>&>(_ary[i])=ve()[i]; }
    ARIADNE_ASSERT(_ary.size()!=0); return *this;
}





} //namespace Ariadne

#endif // ARIADNE_DIFFERENTIAL_HPP
