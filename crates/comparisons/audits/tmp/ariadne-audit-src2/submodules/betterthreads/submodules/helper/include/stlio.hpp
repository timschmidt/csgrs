/***************************************************************************
 *            stlio.hpp
 *
 *  Copyright  2023  Pieter Collins
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*! \file Helper/stlio.hpp
 *  \brief Input-output utilities
 */

#ifndef HELPER_STLIO_HPP
#define HELPER_STLIO_HPP

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
#include <memory>
#include "array.hpp"
#include "tuple.hpp"

namespace Helper {

using std::ostream;
using std::istream;

template<class InputIterator>
ostream&
write_sequence(ostream& os, InputIterator first, InputIterator last,
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
ostream&
write_pointer_sequence(ostream& os, InputIterator first, InputIterator last,
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
ostream&
write_map_sequence(ostream& os, InputIterator first, InputIterator last,
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
ostream&
write_map_pointer_sequence(ostream& os, InputIterator first, InputIterator last,
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
ostream&
write_coefficient_map_sequence(ostream& os, InputIterator first, InputIterator last,
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
istream&
read_sequence(istream& is, Container& v,
              char opening='[', char closing=']', char separator=',')
{
    typedef typename Container::value_type T;

    T x;
    char c;

    v.clear();

    try {
        is >> c;
        if(c != opening) {
            throw std::ios_base::failure(std::string("Helper::Base::read_vector: Input must begin with ")+opening);
        }

        /* Handle case of empty list */
        is >> c;
        if(c != closing) {
            is.putback(c);
            c=separator;
        }

        while(c != closing) {
            if(is.eof()) {
                throw std::ios_base::failure("Helper::Base::read_vector: End-of-file reached");
            }
            if(c!=separator) {
                throw std::ios_base::failure(std::string("Helper::Base::read_vector: Items in list must be separated by ")+separator);
            }
            is >> x;
            if(is.fail()) {
                throw std::ios_base::failure("Helper::Base::read_vector: ErrorTag inputting value in list");
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
    if constexpr (N!=1) {
        write_tuple(os,tup,std::integral_constant<std::size_t,N-1>());
        os << ',';
    }
    os << std::get<N-1>(tup);
}

} // namespace Helper


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
    os << "("; Helper::write_tuple(os,tup,sz); os << ")"; return os;
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
    return Helper::write_map_sequence(os,m.begin(), m.end(), '{', '}');
}

template<class K, class T, class C>
inline
std::ostream&
operator<<(std::ostream &os, const std::map<K,std::shared_ptr<T>,C>& m)
{
    return Helper::write_map_pointer_sequence(os,m.begin(), m.end(), '{', '}');
}

*/

template<class T>
inline
std::istream&
operator>> (std::istream &is, std::vector<T>& v) {
    return Helper::read_sequence(is,v);
}

} // namespace std


#endif /* HELPER_STLIO_HPP */
