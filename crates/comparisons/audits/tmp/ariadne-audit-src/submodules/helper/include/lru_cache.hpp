/***************************************************************************
 *            lru_cache.hpp
 *
 *  Copyright  2007-21  Luca Geretti
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

/*! \file lru_cache.hpp
 *  \brief A Least Recently Used cache for holding a limited number of commonly-accessed objects
 */

#ifndef HELPER_LRU_CACHE_HPP
#define HELPER_LRU_CACHE_HPP

#include <map>
#include "macros.hpp"

namespace Helper {

using std::size_t;

//! \brief A Least Recently Used cache for holding a limited number of commonly-accessed objects of type \a V indexed by a label \a L
template<class L, class V> class LRUCache {
  private:
    struct AgeValuePair {
        AgeValuePair(size_t a, V v) : age(a), value(v) { }
        size_t age;
        V value;
    };
  public:
    //! \brief Construct with a given \a maximum_size
    LRUCache(size_t maximum_size) : _maximum_size(maximum_size) { HELPER_PRECONDITION(maximum_size>0); }

    //! \brief Check whether the label is present
    bool has_label(L const& label) { return (_elements.find(label) != _elements.end()); }

    //! \brief Get the element identified with \a label
    //! \details Updates age accordingly
    V const& get(L const& label) {
        auto e = _elements.find(label);
        HELPER_ASSERT_MSG(e != _elements.end(), "Cache has no element for label " << label);
        for (auto& other : _elements) {
            if (other.second.age < e->second.age)
                other.second.age++;
        }
        e->second.age = 0;
        return e->second.value;
    }

    //! \brief The age of a given \a label in the cache
    size_t age(L const& label) {
        auto e = _elements.find(label);
        HELPER_ASSERT_MSG(e != _elements.end(), "Cache has no element for label " << label);
        return e->second.age;
    }

    //! \brief Insert the element
    //! \details The element must not already exist; updates ages accordingly
    void put(L const& label, V const& val) {
        HELPER_PRECONDITION(not has_label(label));
        if (_elements.size() < _maximum_size) {
            for (auto& other : _elements) other.second.age++;
        } else {
            auto to_remove = _elements.end();
            for (auto it = _elements.begin(); it != _elements.end(); ++it) {
                if (it->second.age == _maximum_size-1) to_remove = it;
                it->second.age++;
            }
            _elements.erase(to_remove);
        }
        _elements.insert({label,AgeValuePair(0,val)});
    }

    //! \brief The current size due to putting elements
    size_t current_size() const {
        return _elements.size();
    }

    //! \brief The maximum size as initially supplied
    size_t maximum_size() const {
        return _maximum_size;
    }

  private:
    size_t _maximum_size;
    std::map<L,AgeValuePair> _elements;
};


} // namespace Helper

#endif // HELPER_LRU_CACHE_HPP
