/*

iRRAM_cache.h -- caching routines for the iterative structure of the iRRAM library
 
Copyright (C) 2001-2003 Norbert Mueller
 
This file is part of the iRRAM Library.
 
The iRRAM Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Library General Public License as published by
the Free Software Foundation; either version 2 of the License, or (at your
option) any later version.
 
The iRRAM Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
License for more details.
 
You should have received a copy of the GNU Library General Public License
along with the iRRAM Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
MA 02111-1307, USA. 
*/

#ifndef iRRAM_CACHE_H
#define iRRAM_CACHE_H

#include <string>
#include <vector>

namespace iRRAM {

namespace internal {

template <typename T,void (*clearfct)(T &)>
struct wrap_type {
	T v;

	wrap_type() noexcept(noexcept(T())) : v(T()) {}

	wrap_type(const T &v) noexcept(noexcept(T(v))) : v(v) {}

	wrap_type(wrap_type &&o) noexcept(noexcept(std::swap(v, o.v)))
	: wrap_type()
	{ using std::swap; swap(v, o.v); }

	~wrap_type() noexcept { clearfct(v); }

	wrap_type & operator=(const T &_v) noexcept(noexcept(const_cast<T &>(_v)=_v))
	{ v = _v; return *this; }

	wrap_type & operator=(wrap_type o) noexcept(noexcept(std::swap(v, o.v)))
	{ using std::swap; swap(v, o.v); return *this; }

	operator T() const noexcept { return v; }
};

template <typename T> struct get_type {
	typedef T type;
};

template <> struct get_type<bool> {
	static void clear(bool &){}
	typedef wrap_type<bool,clear> type;
};

struct cache_type
{
	/* never invoked; just to silence compiler */
	virtual ~cache_type() noexcept = default;
	virtual void clear() = 0;
	virtual void rewind() noexcept = 0;
};

struct cachelist
{
	cache_type * id[50];
};

}

template <class DATA>
class cache : public internal::cache_type
{
	std::vector<typename internal::get_type<DATA>::type> data;
	unsigned int current = 0;
	bool active = false;

	void activate()
	{
		data.clear();
		active = true;
		state->cache_active->id[state->max_active++] = this;
		current = 0;
	}

public:
	void put(const DATA & x)
	{
		if (iRRAM_unlikely(!active))
			activate();
		data.emplace_back(x);
		current++;
	}

	bool get(DATA & x) noexcept(noexcept(x=x))
	{
		if (current >= data.size())
			return false;
		x = data[current++];
		return true;
	}

	void modify(const DATA & x) noexcept(noexcept(const_cast<DATA &>(x)=x))
	{
		data[current - 1] = x;
	}

	void rewind() noexcept { current = 0; }

	void clear()
	{
		data.clear();
		active = false;
		current = 0;
	}
};

namespace internal {

template <typename T> struct is_cacheable : std::false_type {};
template <> struct is_cacheable<bool> : std::true_type {};
template <> struct is_cacheable<short> : std::true_type {};
template <> struct is_cacheable<unsigned short> : std::true_type {};
template <> struct is_cacheable<int> : std::true_type {};
template <> struct is_cacheable<unsigned int> : std::true_type {};
template <> struct is_cacheable<long> : std::true_type {};
template <> struct is_cacheable<unsigned long> : std::true_type {};
template <> struct is_cacheable<long long> : std::true_type {};
template <> struct is_cacheable<unsigned long long> : std::true_type {};
template <> struct is_cacheable<float> : std::true_type {};
template <> struct is_cacheable<double> : std::true_type {};
template <> struct is_cacheable<void *> : std::true_type {};
template <> struct is_cacheable<DYADIC> : std::true_type {};
template <> struct is_cacheable<INTEGER> : std::true_type {};
template <> struct is_cacheable<std::string> : std::true_type {};
template <> struct is_cacheable<std::ostream *> : std::true_type {};
template <> struct is_cacheable<std::istream *> : std::true_type {};

}

struct mv_cache final
: cache<bool>
, cache<short> // unused
, cache<unsigned short> // unused
, cache<int>
, cache<unsigned int> // unused
, cache<long> // unusused
, cache<unsigned long> // unused
, cache<long long> // unused
, cache<unsigned long long> // unused
, cache<float> // unused
, cache<double> // unused
, cache<void*> // unused
, cache<DYADIC>
, cache<INTEGER>
, cache<std::string>
, cache<std::ostream*>
, cache<std::istream*>
{
	mv_cache();
	~mv_cache();
};

} // namespace iRRAM

#endif //define iRRAM_CACHE_H
