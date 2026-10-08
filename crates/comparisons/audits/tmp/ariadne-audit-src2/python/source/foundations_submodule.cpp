/***************************************************************************
 *            foundations_submodule.cpp
 *
 *  Copyright  2008-24  Pieter Collins
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

#include "pybind11.hpp"
#include <pybind11/operators.h>

#include "utilities.hpp"

#if defined(__GNUG__) && !defined(__clang__)
#  pragma GCC diagnostic ignored "-Wattributes"
#endif

#include "utility/string.hpp"
#include "foundations/logical.hpp"

namespace Ariadne {


template<class T> struct PythonClassName { static std::string get() { return class_name<T>(); } };

template<class T> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<T>& repr) {
    return os << python_class_name<T>() << "(" << repr.reference() << ")"; }

template<class T> OutputStream& operator<=(OutputStream& os, T const& t) { return os << python_representation(t); }

const Boolean _true_ = Boolean(true);
const Boolean _false_ = Boolean(false);

template<class L> Bool _decide_(L l) { return decide(l); }
template<class L> Bool _definitely_(L l) { return definitely(l); }
template<class L> Bool _possibly_(L l) { return possibly(l); }
template<class L> Bool _probably_(L l) { return probably(l); }

template<class L, class E=Effort> auto _check_(L const& l, E const& e) -> decltype(l.check(e)) { return l.check(e); }

} // namespace Ariadne


using namespace Ariadne;
using pymodule = pybind11::module;
using pybind11::init;
using pybind11::detail::self;
using pybind11::implicitly_convertible;


Void export_effort(pymodule& module) {
    pybind11::class_<Effort> effort_class(module,"Effort");
    effort_class.def(init<Nat>());
    effort_class.def("work",&Effort::work);
    effort_class.def("__str__", &__cstr__<Effort>);
}

template<class L, class E=Effort> concept HasCheck = requires(L l, E e) { { l.check(e) }; };

template<class L> void export_logical(pymodule& module, std::string name)
{
    pybind11::class_<L> logical_class(module,name.c_str());
    logical_class.def(init<bool>());
    logical_class.def(init<L>());
    if constexpr (Constructible<L,Boolean>) { logical_class.def(init<Boolean>()); }
    if constexpr (Constructible<L,Indeterminate>) { logical_class.def(init<Indeterminate>()); }
    logical_class.def("__bool__", &__bool__<Boolean>);
    define_logical(module,logical_class);
    logical_class.def("__str__", &__cstr__<L>);
    logical_class.def("__repr__", &__repr__<L>);
    if constexpr (HasCheck<L>) {
        typedef decltype(declval<L>().check(declval<Effort>())) CheckType;
        logical_class.def("check", (CheckType(L::*)(Effort)) &L::check);
        module.def("check", [](L l, Effort eff){return check(l,eff);});
    } else {
        module.def("decide", &_decide_<L>);
        module.def("possibly", &_possibly_<L>);
        module.def("definitely", &_definitely_<L>);
    }

    if constexpr (Convertible<Boolean,L>) { implicitly_convertible<Boolean,L>(); }
    if constexpr (Convertible<Indeterminate,L>) { implicitly_convertible<Indeterminate,L>(); }
}

template<> void export_logical<Boolean>(pymodule& module, std::string name)
{
    typedef Boolean L;
    OutputStream& operator<<(OutputStream& os, L l);
    pybind11::class_<L> logical_class(module,name.c_str());
    logical_class.def(init<bool>());
    logical_class.def(init<L>());
    logical_class.def("__bool__", &__bool__<Boolean>);
    logical_class.def("__str__", &__cstr__<L>);
    logical_class.def("__repr__", &__repr__<L>);
    define_logical(module,logical_class);

//    implicitly_convertible<LogicalType<ExactTag>,bool>();
}



Void export_logicals(pymodule& module) {
    export_logical<Boolean>(module,"Boolean");
    export_logical<Sierpinskian>(module,"Sierpinskian");
    export_logical<NegatedSierpinskian>(module,"NegatedSierpinskian");
    export_logical<Kleenean>(module,"Kleenean");
    export_logical<LowerKleenean>(module,"LowerKleenean");
    export_logical<UpperKleenean>(module,"UpperKleenean");
    export_logical<ValidatedKleenean>(module,"ValidatedKleenean");
    export_logical<ValidatedUpperKleenean>(module,"ValidatedUpperKleenean");
    export_logical<ValidatedLowerKleenean>(module,"ValidatedLowerKleenean");
    export_logical<ValidatedSierpinskian>(module,"ValidatedSierpinskian");
    export_logical<ApproximateKleenean>(module,"ApproximateKleenean");

    pybind11::class_<Indeterminate> indeterminate_class(module,"Indeterminate");
    indeterminate_class.def("__str__", &__cstr__<Indeterminate>);

    module.attr("true") = _true_;
    module.attr("false") = _false_;
    module.attr("indeterminate") = indeterminate;

}





Void foundations_submodule(pymodule& module) {
    export_effort(module);

    export_logicals(module);
}

