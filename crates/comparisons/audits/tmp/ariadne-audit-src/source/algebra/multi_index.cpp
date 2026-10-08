/***************************************************************************
 *            algebra/multi_index.cpp
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

#include "helper/stlio.hpp"
#include "utility/container.hpp"
#include "multi_index.hpp"

namespace Ariadne {

MultiIndex& MultiIndex::operator++() {
    assert(_n>0);
    if(_n==1) { ++_p[0]; ++_p[_n]; return *this; }
    if(_p[_n-2]!=0) { --_p[_n-2]; ++_p[_n-1]; return *this; }
    else {
        IndexType li=_p[_n-1]; _p[_n-1]=0;
        for(SizeType k=_n-1; k!=0; --k) { if(_p[k-1]!=0) { --_p[k-1]; _p[k]=li+1u; return *this; } }
        _p[0]=li+1u; ++_p[_n];
    }
    return *this;
}



MultiIndexList::MultiIndexList(SizeType as)
    : _capacity(DEFAULT_CAPACITY), _size(0u), _argument_size(as)
    , _indices(new DegreeType[_capacity*_argument_size])
{
}

MultiIndexList::~MultiIndexList() {
    delete[] _indices; _indices=nullptr;
}

MultiIndexList::MultiIndexList(InitializerList<InitializerList<DegreeType>> const& lst)
    : MultiIndexList((assert(lst.size()!=0),lst.begin()->size()))
{
    for(auto iter=lst.begin(); iter!=lst.end(); ++iter) { this->append(MultiIndex(*iter)); }
}

MultiIndexList::MultiIndexList(InitializerList<MultiIndex> const& lst)
    : MultiIndexList((assert(lst.size()!=0),lst.begin()->size()))
{
    for(auto iter=lst.begin(); iter!=lst.end(); ++iter) { this->append(*iter); }
}

MultiIndexList::MultiIndexList(SizeType n, MultiIndex const& a)
    : MultiIndexList(a.size())
{
    for(SizeType i=0; i!=n; ++i) { this->append(a); }
}

MultiIndexList::MultiIndexList(MultiIndexList const& lst) : MultiIndexList(lst.argument_size()) {
    for(SizeType i=0; i!=lst.size(); ++i) { this->append(lst[i]); }
}


MultiIndexList& MultiIndexList::operator=(MultiIndexList const& lst) {
    if(this != &lst) {
        delete[] _indices;

        _capacity=lst._capacity;
        _size=lst._size;
        _argument_size=lst._argument_size;

        _indices=new DegreeType[_capacity*_argument_size];
        for(SizeType k=0; k!=_size*_argument_size; ++k) { _indices[k]=lst._indices[k]; }
    }
    return *this;
}


Void MultiIndexList::resize(SizeType new_size) {
    this->reserve(new_size);
    if (new_size>_size) for(SizeType k=_size*_argument_size; k!=new_size*_argument_size; ++k) { _indices[k] = 0u; }
    _size=new_size;
}


Void MultiIndexList::reserve(SizeType requested_capacity) {
    if(requested_capacity>_capacity) {
        SizeType new_capacity=std::max(DEFAULT_CAPACITY,_capacity);
        while(new_capacity<requested_capacity) { new_capacity*=2u; }
        new_capacity=requested_capacity;
        DegreeType* new_indices=new DegreeType[new_capacity*_argument_size];
        for(SizeType k=0; k!=_size*_argument_size; ++k) { new_indices[k]=_indices[k]; }

        delete[] _indices;

        _capacity=new_capacity;
        _indices=new_indices;

    }
}

MultiIndexList::Iterator MultiIndexList::erase(Iterator pos) {
    Iterator curr=pos;
    Iterator next=pos+1;
    while(next!=this->end()) {
        *pos=*next;
        ++pos; ++next;
    }
    --_size;
    return pos;
}

Void MultiIndexList::clear() {
    _size=0u; }

Bool operator==(const MultiIndexList& lst1, const MultiIndexList& lst2) {
    if(lst1._size!=lst2._size or lst1._argument_size != lst2._argument_size) { return false; }
    for(SizeType i=0; i!=lst1._size; ++i) { if(lst1[i]!=lst2[i]) { return false; } } return true; }

OutputStream& operator<<(OutputStream& os, MultiIndexList const& lst) {
    os << "MultiIndexList" << std::flush;
    os << "<" << lst.size() << "/" << lst.capacity() << ";" << lst.argument_size() << ">";
    os << "["; for(SizeType i=0; i!=lst.size(); ++i) { if(i!=0) { os << ","; } os << lst[i]; } os << "]"; return os;
}



} // namespace Ariadne
