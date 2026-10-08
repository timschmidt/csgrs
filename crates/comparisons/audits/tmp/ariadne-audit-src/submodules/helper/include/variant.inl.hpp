/***************************************************************************
 *            variant.inl.hpp
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


#ifndef HELPER_VARIANT_INL_HPP
#define HELPER_VARIANT_INL_HPP

#include "variant.hpp"

namespace Helper {

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


} // namespace Helper

#endif
