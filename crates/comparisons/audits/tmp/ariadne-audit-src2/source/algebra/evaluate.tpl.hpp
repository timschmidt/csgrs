/***************************************************************************
 *            evaluate.tpl.hpp
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

#include "algebra/vector.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"

namespace Ariadne {

//! \details
//! For a polynomial in \f$n\f$ variables, Horner's rule is a recursive formula
//! \f[ p(x) = \bigl( \bigl(  x^{d_k-d_{k-1}} q_k(\hat{x})x^{d_0} + \cdots + q_1(\hat{x}) \bigr) x^{d_1-d_0} + q_0(\hat{x}) \bigr) x^{d_0} \f]
//! where \f$\hat{x}=(x_1,\ldots,x_{n-1})\f$ and \f$q_i\f$ is the polynomial of terms in \f$x_n^{d_i}\f$.
//! To evaluate a polynomial using Horner's rule without using recursive function calls, we maintain registers \f$r_k\f$ containing
//! the current evaluation of a polynomial in \f$(x_1,\ldots,x_k)\f$.
//!
//! We list the terms in reverse lexicographic order, defined as \f$\alpha \prec \beta\f$ if \f$\alpha_j>\beta_j\f$,
//! where \f$j=\max\{i\mid \alpha_i\neq\beta_i\}\f$.
//! For a given term \f$c_\alpha x^\alpha\f$, let \f$k=\max\{j\mid \alpha_j\neq\beta_j\}\f$, where \f$\beta\f$ is the next multi-index.
//! We update register \f$r_k\f$ by \f[r'_k=(((c_\alpha + r_1) x^{\alpha_1} + r_2 )x^{\alpha_2}+\cdots r_k)x^{\alpha_k-\beta_k}.\f]
//! The result is obtained by updating a fictional register \f$r_{n+1}\f$ at the last step.
//! See J. M. Pena and T. Sauer, "On the multivariate Horner scheme", SIAM J. Numer. Anal. 37(4) 1186-1197, 2000.
template<class X, class A> ArithmeticType<X,A> horner_evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& x);

template<class X, class Y> ArithmeticType<X,Y> horner_evaluate(const Expansion<MultiIndex,X>& e, const Vector<Y>& x)
{
    typedef typename Expansion<MultiIndex,X>::ConstIterator ConstIterator;
    typedef ArithmeticType<X,Y> R;
    const SizeType n=e.argument_size();
    const R z(e.zero_coefficient()*x.zero_element()); // The zero element of the ring Y
    if(e.number_of_nonzeros()==0) { return z; }

    Array< R > r(e.argument_size(),z); // An Array of "registers" containing working p(x[0],...,x[k])
    ConstIterator iter=e.begin();
    ConstIterator end=e.end();
    SizeType k=n;   // The current working register
    MultiIndex na=iter->index(); // The values of the next multi-index
    SizeType j=k;   // The lowest register containing a non-zero value
    X c=iter->coefficient();
    R t=z;
    MultiIndex a=na; // The values of the next multi-index
    ++iter;
    while(iter!=end) {
        na=iter->index();
        k=n-1;
        while(a[k]==na[k]) { --k; }
        // Since terms are ordered reverse-lexicographically,
        // previous index must have higher kth value
        assert(a[k]>na[k]);
        // Set r[k]=(((c+r[0])*x[0]^a[0]+r[1])*x[1]^a[1]+...+r[k])*x[k]^(a[k]-na[k])
        // Omit zero terms where possible
        t=c;
        for(SizeType i=0; i!=std::min(j,k); ++i) {
            for(DegreeType ii=0; ii!=a[i]; ++ii) {
                t=t*x[i];
            }
        }
        for(SizeType i=std::min(j,k); i!=k; ++i) {
            t=t+r[i];
            for(DegreeType ii=0; ii!=a[i]; ++ii) {
                t=t*x[i];
            }
            r[i]=z;
        }
        if(j<=k) {
            t=t+r[k];
        }
        for(DegreeType ii=na[k]; ii!=a[k]; ++ii) {
            t=t*x[k];
        }
        r[k]=t;
        //std::cerr<<"a="<<MultiIndex(n,a)<<" c="<<c<<" k="<<k<<" r="<<r<<"\n";
        j=k;
        c=iter->coefficient();
        a=na;
        ++iter;
    }
    // Set r=(((c+r[0])*x[0]^a[0]+r[1])*x[1]^a[1]+...+r[n-1])*x[n-1]^(a[n-1])
    t=c;
    for(SizeType i=0; i!=j; ++i) {
        for(DegreeType ii=0; ii!=a[i]; ++ii) {
            t=t*x[i];
        }
    }
    for(SizeType i=j; i!=n; ++i) {
        t=t+r[i];
        for(DegreeType ii=0; ii!=a[i]; ++ii) {
            t=t*x[i];
        }
    }
    //std::cerr<<"a="<<MultiIndex(n,a)<<" c="<<c<<" k="<<n<<"\n";
    //std::cerr<<"  r="<<t<<"\n";
    return t;
}

template<class X, class Y> ArithmeticType<X,Y> horner_evaluate(const Expansion<UniIndex,X>& e, const Y& x)
{
    typedef ArithmeticType<X,Y> R;
    typedef typename Expansion<UniIndex,X>::ConstIterator ConstIterator;
    if(e.number_of_nonzeros()==0) { return nul(x); }

    ConstIterator iter=e.begin();
    ConstIterator end=e.end();
    DegreeType na=iter->index(); // The values of the next multi-index
    X c=iter->coefficient();
    R r=nul(x);
    r=c;
    DegreeType a=na; // The values of the next multi-index
    ++iter;
    while(iter!=end) {
        na=iter->index();
        c=iter->coefficient();
        // Since terms are ordered in reverse,
        // previous index must have higher value
        assert(a>na);
        // Set r[k]=(((c+r[0])*x[0]^a[0]+r[1])*x[1]^a[1]+...+r[k])*x[k]^(a[k]-na[k])
        // Omit zero terms where possible
        for(SizeType ii=na; ii!=a; ++ii) {
            r=r*x;
        }
        r=r+c;
        a=na;
        ++iter;
    }
    return r;
}

template<class X, class Y>
ArithmeticType<X,Y> power_evaluate(const Expansion<MultiIndex,X>& e, const Vector<Y>& y)
{
    typedef ArithmeticType<X,Y> R;

    R zero = e.zero_coefficient()*y.zero_element();
    R one = zero + 1;

    R r=zero;
    R t=zero;
    for(typename Expansion<MultiIndex,X>::ConstIterator iter=e.begin();
        iter!=e.end(); ++iter)
    {
        UniformConstReference<MultiIndex> j=iter->index();
        UniformConstReference<X> c=iter->coefficient();
        t=one;
        for(SizeType k=0; k!=e.argument_size(); ++k) {
            for(DegreeType l=0; l!=j[k]; ++l) {
                t=t*y[k];
            }
        }
        t*=c;
        r+=t;
    }

    return r;
}


template<class X, class Y>
ArithmeticType<X,Y> evaluate(const Expansion<MultiIndex,X>& e, const Vector<Y>& y)
{
    return power_evaluate(e,y);
}

template<class X, class Y>
ArithmeticType<X,Y> simple_evaluate(const Expansion<MultiIndex,X>& e, const Vector<Y>& y)
{
    return power_evaluate(e,y);
}

template<class X, class Y>
Vector<ArithmeticType<X,Y>> evaluate(const Vector< Expansion<MultiIndex,X> >& x, const Vector<Y>& y)
{
    Vector<Y> r(x.size(),y.zero_element());
    for(SizeType i=0; i!=x.size(); ++i) {
        r[i]=evaluate(x[i],y);
    }
    return r;
}

}

