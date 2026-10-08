/***************************************************************************
 *            helper/stlio.hpp
 *
 *  Copyright  2005-20  Pieter Collins
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

/*! \file helper/stlio.hpp
 *  \brief Input-output utilities
 */

#ifndef ARIADNE_STLIO_HPP
#define ARIADNE_STLIO_HPP

#include <iostream>
#include <stdexcept>

#include <string>
#include <tuple>
#include <vector>
#include <list>
#include <deque>
#include <valarray>
#include <set>
#include <map>
#include "utility/array.hpp"
#include "utility/tuple.hpp"

#include <memory>


namespace Ariadne {

using OutputStream = std::ostream;
using InputStream = std::istream;

template<class InputIterator>
OutputStream&
write_sequence(OutputStream& os, InputIterator first, InputIterator last,
               char opening='[', char closing=']', char separator=',')
{
    os << opening;
    while(first!=last) {
        os << (*first);
        ++first;
        if(first!=last) {
            os << separator;
        }
    }
    os << closing;
    return os;
}


template<class InputIterator>
OutputStream&
write_pointer_sequence(OutputStream& os, InputIterator first, InputIterator last,
                       char opening='[', char closing=']', char separator=',')
{
    os << opening;
    while(first!=last) {
        os << (**first);
        ++first;
        if(first!=last) {
            os << separator;
        }
    }
    os << closing;
    return os;
}


template<class InputIterator>
OutputStream&
write_map_sequence(OutputStream& os, InputIterator first, InputIterator last,
                   char opening='{', char closing='}', char separator=',', char descriptor=':')
{
    os << opening;
    while(first!=last) {
        os << first->first << descriptor << first->second;
        ++first;
        if(first != last) {
            os << separator;
        }
    }
    os << closing;
    return os;
}

template<class InputIterator>
OutputStream&
write_map_pointer_sequence(OutputStream& os, InputIterator first, InputIterator last,
                           char opening='{', char closing='}', char separator=',', char descriptor=':')
{
    os << opening;
    while(first!=last) {
        os << first->first << descriptor << *first->second;
        ++first;
        if(first != last) {
            os << separator;
        }
    }
    os << closing;
    return os;
}

template<class InputIterator>
OutputStream&
write_ariadne_map_sequence(OutputStream& os, InputIterator first, InputIterator last,
                           char opening='{', char closing='}', char separator=',', char descriptor=':')
{
    os << opening;
    while(first!=last) {
        os << first->index() << descriptor << first->coefficient();
        ++first;
        if(first != last) {
            os << separator;
        }
    }
    os << closing;
    return os;
}


template<class Container>
InputStream&
read_sequence(InputStream& is, Container& v,
              char opening='[', char closing=']', char separator=',')
{
    typedef typename Container::value_type T;

    T x;
    char c;

    v.clear();

    try {
        is >> c;
        if(c != opening) {
            throw std::ios_base::failure(std::string("Ariadne::Base::read_vector: Input must begin with ")+opening);
        }

        /* Handle case of empty list */
        is >> c;
        if(c != closing) {
            is.putback(c);
            c=separator;
        }

        while(c != closing) {
            if(is.eof()) {
                throw std::ios_base::failure("Ariadne::Base::read_vector: End-of-file reached");
            }
            if(c!=separator) {
                throw std::ios_base::failure(std::string("Ariadne::Base::read_vector: Items in list must be separated by ")+separator);
            }
            is >> x;
            if(is.fail()) {
                throw std::ios_base::failure("Ariadne::Base::read_vector: ErrorTag inputting value in list");
            }
            v.push_back(x);
            is >> c;
        }
    }
    catch(...) {
        // is.seekg(pos);
        throw;
    }

    return is;
}


template<class TUP, std::size_t N> void write_tuple(std::ostream& os, TUP const& tup, std::integral_constant<std::size_t,N>) {
    if constexpr (N>1) {
        write_tuple(os,tup,std::integral_constant<std::size_t,N-1>());
        os << ',';
    }
    if constexpr (N>0) {
        os << std::get<N-1>(tup);
    }
}

} // namespace Ariadne


/* FIXME: This is a hack to allow io of STL classes.
   But really we should not modify namespace std.
   Unfortunately, we need to include the code in
   any namespace using operator<<.
*/
namespace std {

template<class S, class T>
inline
std::ostream&
operator<<(std::ostream &os, const std::pair<S,T>& s)
{
    return os << '(' << s.first << ',' << s.second << ')';
}

template<class... TS> inline
std::ostream& operator<<(std::ostream& os, std::tuple<TS...> const& tup) {
    typename std::tuple_size<std::tuple<TS...>>::type sz;
    os << "("; Ariadne::write_tuple(os,tup,sz); os << ")"; return os;
}

/*
template<class T>
inline
std::ostream&
operator<< (std::ostream &os, const std::vector<T>& v)
{
    return Helper::write_sequence(os,v.begin(),v.end());
}
*/


/*
template<class T>
inline
std::ostream&
operator<< (std::ostream &os, const std::deque<T>& d)
{
    return Helper::write_sequence(os,d.begin(),d.end());
}

template<class T>
inline
ostream&
operator<< (std::ostream &os, const std::valarray<T>& v) {
    return Helper::write_sequence(os,&(v[0]),&(v[v.size()-1]));
}

template<class T, class C>
inline
std::ostream&
operator<<(std::ostream &os, const std::set<T,C>& s)
{
    return Helper::write_sequence(os,s.begin(), s.end(), '{', '}');
}

template<class K, class T, class C>
inline
std::ostream&
operator<<(std::ostream &os, const std::map<K,T,C>& m)
{
    return Ariadne::write_map_sequence(os,m.begin(), m.end(), '{', '}');
}

template<class K, class T, class C>
inline
std::ostream&
operator<<(std::ostream &os, const std::map<K,std::shared_ptr<T>,C>& m)
{
    return Ariadne::write_map_pointer_sequence(os,m.begin(), m.end(), '{', '}');
}

*/

template<class T>
inline
std::istream&
operator>> (std::istream &is, std::vector<T>& v) {
    return Ariadne::read_sequence(is,v);
}

} // namespace std


#endif /* ARIADNE_STLIO_HPP */
