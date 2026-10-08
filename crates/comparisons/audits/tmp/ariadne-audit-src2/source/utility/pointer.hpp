/***************************************************************************
 *            utility/pointer.hpp
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

/*! \file utility/pointer.hpp
 *  \brief Smart pointers in Euclidean space.
 */

#ifndef ARIADNE_POINTER_HPP
#define ARIADNE_POINTER_HPP

#include <memory>
#include <iostream>
#include <cassert>


namespace Ariadne {

template<class T> using SharedPointer = std::shared_ptr<T>;
using OutputStream = std::ostream;

// A tag used as a function argument to denote that the value is a pointer, and not possibly the integer 0
struct PointerTag { };

template<class T>
class copy_on_write_ptr
{
  private:
    mutable int* _ref_count;
    mutable T* _ptr;
  public:
    ~copy_on_write_ptr() { this->_deallocate(); }
    template<class S> copy_on_write_ptr(S* pointer)
        : _ref_count(new int(1)), _ptr(pointer) { }
    copy_on_write_ptr(const copy_on_write_ptr<T>& other)
        : _ref_count(other._ref_count), _ptr(other._ptr) { ++(*_ref_count); }
    template<class S> copy_on_write_ptr<T>& operator=(S* pointer) {
        _deallocate(); _allocate(pointer); return *this; }
    copy_on_write_ptr<T>& operator=(const copy_on_write_ptr<T>& other) {
        if(_ptr!=other._ptr) { _deallocate(); _ptr=other._ptr;
            _ref_count=other._ref_count; ++(*_ref_count); } return *this; }
    const T& operator*() const { return *_ptr; }
    const T* operator->() const { return _ptr; }
    T& operator*() { if(*_ref_count>1) { _make_copy(); } return *_ptr; }
    T* operator->() { if(*_ref_count>1) { _make_copy(); } return _ptr; }
  public:
    int count() const { return *_ref_count; }
    const void* address() const { return _ptr; }
    const T& value() const { return *_ptr; }
  private:
    void _make_copy() {
        --(*_ref_count); _ref_count=new int(1); _ptr=new T(*_ptr); }
    void _deallocate() {
        if(*_ref_count==1) { delete _ptr; _ptr=0; delete _ref_count; }
        else { --(*_ref_count); } }
    template<class S> void _allocate(S* pointer) {
        _ref_count=new int(1); _ptr=pointer; }
};

template<class T> OutputStream&
operator<<(OutputStream& os, const copy_on_write_ptr<T>& ptr) {
    return os << "["<<ptr.count()<<"]<"<<ptr.address()<<">("<<ptr.value()<<")";
}

template<class T>
class clone_on_write_ptr
{
    template<class S> friend class clone_on_write_ptr;
  private:
    mutable int* _ref_count;
    mutable T* _ptr;
  public:
    ~clone_on_write_ptr() { this->_deallocate(); }
    template<class S> clone_on_write_ptr(S* pointer)
        : _ref_count(new int(1)), _ptr(pointer) { }
    clone_on_write_ptr(const clone_on_write_ptr<T>& other)
        : _ref_count(other._ref_count), _ptr(other._ptr) { ++(*_ref_count); }
    template<class S> clone_on_write_ptr(const clone_on_write_ptr<S>& other)
        : _ref_count(other._ref_count), _ptr(other._ptr) { ++(*_ref_count); }
    template<class S> clone_on_write_ptr<T>& operator=(S* pointer) {
        _deallocate(); _allocate(pointer); return *this; }
    clone_on_write_ptr<T>& operator=(const clone_on_write_ptr<T>& other) {
        if(_ptr!=other._ptr) { _deallocate(); _ptr=other._ptr;
            _ref_count=other._ref_count; ++(*_ref_count); } return *this; }
    const T& operator*() const { return *_ptr; }
    const T* operator->() const { return _ptr; }
    T& operator*() { if(*_ref_count>1) { _makeclone(); } return *_ptr; }
    T* operator->() { if(*_ref_count>1) { _makeclone(); } return _ptr; }
  public:
    int count() const { return *_ref_count; }
    const void* address() const { return _ptr; }
    const T& value() const { return *_ptr; }
  private:
    void _makeclone() {
        --(*_ref_count); _ref_count=new int(1); _ptr=_ptr->clone(); }
    void _deallocate() {
        if(*_ref_count==1) { delete _ptr; _ptr=0; delete _ref_count; }
        else { --(*_ref_count); } }
    template<class S> void _allocate(S* pointer) {
        _ref_count=new int(1); _ptr=pointer; }
};

template<class T> OutputStream&
operator<<(OutputStream& os, const clone_on_write_ptr<T>& ptr) {
    return os << "["<<ptr.count()<<"]<"<<ptr.address()<<">("<<ptr.value()<<")";
}

template<class T>
class copy_on_copy_ptr
{
  private:
    mutable T* _ptr;
  public:
    ~copy_on_copy_ptr() { delete _ptr; _ptr=0; }
    copy_on_copy_ptr()
        : _ptr(0u) { }
    template<class S> copy_on_copy_ptr(S* pointer)
        : _ptr(pointer) { }
    copy_on_copy_ptr(const copy_on_copy_ptr<T>& other)
        : _ptr(other._ptr?other._ptr->clone():other._ptr) { }
    template<class S> copy_on_copy_ptr<T>& operator=(S* pointer) {
        delete _ptr; _ptr=pointer; return *this; }
    copy_on_copy_ptr<T>& operator=(const copy_on_copy_ptr<T>& other) {
        if(_ptr!=other._ptr) { delete _ptr; _ptr=other._ptr?other._ptr->clone():other._ptr; } return *this; }
    const T& operator*() const { return *_ptr; }
    const T* operator->() const { return _ptr; }
    T& operator*() { return *_ptr; }
    T* operator->() { return _ptr; }
  public:
    const void* address() const { return _ptr; }
    const T& value() const { return *_ptr; }
};

template<class T> OutputStream&
operator<<(OutputStream& os, const copy_on_copy_ptr<T>& ptr) {
    return os<<"<"<<ptr.address()<<">";
}

template<class T>
class clone_on_copy_ptr
{
  private:
    mutable T* _ptr;
  public:
    ~clone_on_copy_ptr() { delete _ptr; _ptr=0; }
    clone_on_copy_ptr() : _ptr(0) { }
    template<class S> clone_on_copy_ptr(S* pointer)
        : _ptr(pointer) { }
    clone_on_copy_ptr(const clone_on_copy_ptr<T>& other)
        : _ptr(other._ptr?other._ptr->_clone():other._ptr) { }
    template<class S> clone_on_copy_ptr<T>& operator=(S* pointer) {
        delete _ptr; _ptr=pointer; return *this; }
    clone_on_copy_ptr<T>& operator=(const clone_on_copy_ptr<T>& other) {
        if(_ptr!=other._ptr) { delete _ptr; _ptr=other._ptr?other._ptr->_clone():other._ptr; } return *this; }
    const T& operator*() const { return *_ptr; }
    const T* operator->() const { return _ptr; }
    T& operator*() { return *_ptr; }
    T* operator->() { return _ptr; }
  public:
    const void* address() const { return _ptr; }
    const T& value() const { return *_ptr; }
};

template<class T> OutputStream&
operator<<(OutputStream& os, const clone_on_copy_ptr<T>& ptr) {
    return os<<"<"<<ptr.address()<<">";
}


template<class T> class counted_pointer;

template<class T>
class counted_pointer<const T>
{
    const T* _ptr;
  public:
    counted_pointer(const T* p) : _ptr(p) { assert(p!=0); ++_ptr->count; }
    counted_pointer(const counted_pointer<const T>& p) : _ptr(p._ptr) { ++_ptr->count; }
    counted_pointer<const T>& operator=(const counted_pointer<const T>& p) {
        if(_ptr!=p._ptr) { --_ptr->count; if(_ptr->count==0) { delete _ptr; } _ptr=p._ptr; ++_ptr->count; } return *this; }
    ~counted_pointer() { --_ptr->count; if(_ptr->count==0) { delete _ptr; } }
    const T* operator->() const { return _ptr; }
};

} // namespace Ariadne

#endif // ARIADNE_POINTER_HPP
