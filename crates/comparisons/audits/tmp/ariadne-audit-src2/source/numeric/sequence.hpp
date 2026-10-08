/***************************************************************************
 *            numeric/sequence.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file numeric/sequence.hpp
 *  \brief
 */

#ifndef ARIADNE_SEQUENCE_HPP
#define ARIADNE_SEQUENCE_HPP

#include <functional>

namespace Ariadne {

using Nat = unsigned int;
class Natural;

class Dyadic; class Rational; class Real;
template<class Y> struct CompletionTypedef;
template<> struct CompletionTypedef<Dyadic> { typedef Real Type; };
template<> struct CompletionTypedef<Rational> { typedef Real Type; };
template<> struct CompletionTypedef<Real> { typedef Real Type; };
template<class Y> using CompletionType = typename CompletionTypedef<Y>::Type;


//! \brief A function \f$\mathbb{N} \to X\f$.
template<class X> class Sequence {
    std::function<X(Natural)> _fn;
  public:
    Sequence(std::function<X(Natural)> const& fn) : _fn(fn) { }
    X operator[](Natural const& n) const { return _fn(n); }
};

//! \brief A convergent sequence in \f$X\f$, with no further information about the convergence rate
template<class X> class ConvergentSequence : public Sequence<X> {
  public:
    ConvergentSequence(std::function<X(Natural)> const& fn) : Sequence<X>(fn) { }
    ConvergentSequence(Sequence<X> const& seq) : Sequence<X>(seq) { }
};

//! An alternating sequence in a partially-ordered space (X,,≤), satisfying \f$x_n \in [x_{n-1},x_{n-2}]\f$.
template<class X> class AlternatingSequence : public ConvergentSequence<X> {
  public:
    explicit AlternatingSequence(Sequence<X> const& seq) : ConvergentSequence<X>(seq) { }
};

//! \brief An increasing sequence in a partially-ordered space (X,≤).
template<class X> class IncreasingSequence : public Sequence<X> {
  public:
    IncreasingSequence(std::function<X(Natural)> fn) : Sequence<X>(fn) { }
    IncreasingSequence(Sequence<X> const& seq) : Sequence<X>(seq) { }
};
//! \brief A decreasing sequence in a partially-ordered space (X,≤).
template<class X> class DecreasingSequence : public Sequence<X> {
  public:
    DecreasingSequence(std::function<X(Natural)> fn) : Sequence<X>(fn) { }
    DecreasingSequence(Sequence<X> const& seq) : Sequence<X>(seq) { }
};

//! \brief A fast-converging Cauchy sequence in a metric space (X,d), satisfying \f$ d(x_m,x_n) \leq 2^{-min(m,n)}\f$.
template<class X> class FastCauchySequence : public Sequence<X> {
  public:
    FastCauchySequence(std::function<X(Natural)> const& fn) : Sequence<X>(fn) { }
    FastCauchySequence(Sequence<X> const& seq) : Sequence<X>(seq) { }
    friend CompletionType<X> limit(FastCauchySequence<X> const&);
};


using OutputStream = std::ostream;
template<class T, class W> class WritableTemporary;

class SequenceWriter {
    Nat _num;
  public:
    SequenceWriter(Nat const& n) : _num(n) { }
    template<class T> inline WritableTemporary<Sequence<T>,SequenceWriter> operator() (Sequence<T> const& seq) const {
        return WritableTemporary<Sequence<T>,SequenceWriter>(*this,seq); }
    template<class T> OutputStream& _write(OutputStream& os, Sequence<T> const& seq) const {
        os << "["; for (Nat i=0u; i!=_num; ++i) { if (i!=0) { os << ","; } os << seq[i]; } os << "]"; return os; }
};

} // namespace Ariadne

#endif
