/***************************************************************************
 *            utility/variant.inl.hpp
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


#ifndef ARIADNE_VARIANT_INL_HPP
#define ARIADNE_VARIANT_INL_HPP

#include "variant.hpp"

namespace Ariadne {

template<class V, class C>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        default: abort(); } }

template<class V, class C, class T1>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1());
        default: abort(); } }

template<class V, class C, class T1, class T2>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2());
        default: abort(); } }

template<class V, class C, class T1, class T2, class T3>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11, class T12>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11()); case T12::code(): return v(T12());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11, class T12, class T13>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11()); case T12::code(): return v(T12());
        case T13::code(): return v(T13());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11, class T12, class T13, class T14>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11()); case T12::code(): return v(T12());
        case T13::code(): return v(T13()); case T14::code(): return v(T14());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11, class T12, class T13, class T14, class T15>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11()); case T12::code(): return v(T12());
        case T13::code(): return v(T13()); case T14::code(): return v(T14()); case T15::code(): return v(T15());
        default: abort(); } }

template<class V, class C, class  T1, class  T2, class  T3, class  T4, class  T5, class  T6, class  T7, class  T8,
                            class  T9, class T10, class T11, class T12, class T13, class T14, class T15, class T16>
decltype(auto) coded_visit(V& v, C code) {
    switch (code) {
        case T1::code(): return v(T1()); case T2::code(): return v(T2()); case T3::code(): return v(T3()); case T4::code(): return v(T4());
        case T5::code(): return v(T5()); case T6::code(): return v(T6()); case T7::code(): return v(T7()); case T8::code(): return v(T8());
        case T9::code(): return v(T9()); case T10::code(): return v(T10()); case T11::code(): return v(T11()); case T12::code(): return v(T12());
        case T13::code(): return v(T13()); case T14::code(): return v(T14()); case T15::code(): return v(T15()); case T16::code(): return v(T16());
        default: abort(); } }



template<class C, class... TS> template<class V> inline decltype(auto) CodedVariant<C,TS...>::accept(V const& v) const {
    return coded_visit<V,C,TS...>(const_cast<V&>(v),this->code());
}


} // namespace Ariadne

#endif
