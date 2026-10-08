/***************************************************************************
 *            linear_algebra_submodule.cpp
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

#include "pybind11.hpp"
#include "utilities.hpp"
#include "numeric_submodule.hpp"
#include "linear_algebra_submodule.hpp"

#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/diagonal_matrix.hpp"

#include "algebra/matrix.tpl.hpp"

using namespace Ariadne;

namespace Ariadne {

template<> struct PythonTemplateName<Vector> { static std::string get() { return "Vector"; } };
template<> struct PythonTemplateName<Covector> { static std::string get() { return "Covector"; } };
template<> struct PythonTemplateName<Matrix> { static std::string get() { return "Matrix"; } };
template<> struct PythonTemplateName<DiagonalMatrix> { static std::string get() { return "DiagonalMatrix"; } };

template<class X> struct PythonClassName<Vector<X>> { static std::string get() { return python_template_class_name<X>("Vector"); } };
template<class X> struct PythonClassName<Covector<X>> { static std::string get() { return python_template_class_name<X>("Covector"); } };
template<class X> struct PythonClassName<Matrix<X>> { static std::string get() { return python_template_class_name<X>("Matrix"); } };
template<class X> struct PythonClassName<DiagonalMatrix<X>> { static std::string get() { return python_template_class_name<X>("DiagonalMatrix"); } };


// Define transpose operation which evaluates expression template
template<class X> Matrix<X> transpose(Matrix<X> const& A) { return MatrixTranspose<Matrix<X>>(A); }

// Join two scalars to make a vector
template<class X> Vector<X> _sjoin_(const X& s1, const X& s2) { return Vector<X>{s1,s2}; }


template<class X> pybind11::list vector_to_python(Vector<X> const& v) {
    pybind11::list lst;
    for(SizeType i=0; i!=v.size(); ++i) {
        lst.append(pybind11::cast(v[i]));
    }
    return lst;
}

template<class X> pybind11::list matrix_to_python(Matrix<X> const& A) {
    pybind11::list res;
    for(SizeType i=0; i!=A.row_size(); ++i) {
        pybind11::list row;
        for(SizeType j=0; j!=A.column_size(); ++j) {
            row.append(pybind11::cast(A[i][j]));
        }
        res.append(row);
    }
    return res;
}

namespace { // Don't export the following functions to prevent visibility inconsistency warnings with pybind11 names

template<class X> Vector<X> vector_from_python(pybind11::list const& lst) {
    assert (lst.size()!=0 || DefaultConstructible<X>);
    if constexpr (DefaultConstructible<X>) {
        return Vector<X>( lst.size(), [&lst](SizeType i){return from_python_object_or_literal<X>(lst[i]);} );
    } else {
        auto prs = characteristics(from_python_object_or_literal<X>(lst[0]));
        return Vector<X>( lst.size(), [&lst](SizeType i){return from_python_object_or_literal<X>(lst[i]);}, prs );
    }
}

template<class X> Covector<X> covector_from_python(pybind11::list const& lst) {
    assert (lst.size()!=0 || DefaultConstructible<X>);
    if constexpr (DefaultConstructible<X>) {
        return Covector<X>( lst.size(), [&lst](SizeType i){return from_python_object_or_literal<X>(lst[i]);} );
    } else {
        auto prs = characteristics(from_python_object_or_literal<X>(lst[0]));
        return Covector<X>( lst.size(), [&lst](SizeType i){return from_python_object_or_literal<X>(lst[i]);}, prs );
    }
}

template<class X> Matrix<X> matrix_from_python(pybind11::list const& lst) {
    Array<pybind11::list> rows(lst.size(),[&](SizeType i){return static_cast<pybind11::list const&>(lst[i]);});
    SizeType rs=lst.size();
    assert(rs!=0);
    SizeType cs=rows[0].size();
    assert(cs!=0);
    return Matrix<X>(rs,cs,[&rows](SizeType i, SizeType j){return from_python_object_or_literal<X>(rows[i][j]);});
}

} // namespace

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Vector<X>>& repr) {
    Vector<X> const& v=repr.reference();
    os <<class_name<X>()<<"Vector"<<"([";
    for(SizeType i=0; i!=v.size(); ++i) {
        if(i!=0) { os <<","; }
        if constexpr (HasPrecisionType<X>) { os << python_literal(v[i]); }
        else { os <<python_representation(v[i]); }
    }
    os <<"]";
    if constexpr (HasPrecisionType<X>) { os << "," << v.zero_element().precision(); }
    os <<")";
    return os;
}

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Covector<X>>& repr) {
    Covector<X> const& u=repr.reference();
    os <<class_name<X>()<<"Covector"<<"([";
    for(SizeType i=0; i!=u.size(); ++i) {
        if(i!=0) { os <<","; }
        if constexpr (HasPrecisionType<X>) { os << python_literal(u[i]); }
        else { os <<python_representation(u[i]); }
    }
    os <<"]";
    if constexpr (HasPrecisionType<X>) { os << "," << u.zero_element().precision(); }
    os <<")";
    return os;
}

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Matrix<X>>& repr) {
    Matrix<X> const& A=repr.reference();
    os << class_name<X>() << "Matrix([";
    for(SizeType i=0; i!=A.row_size(); ++i) {
        if(i!=0) { os <<","; } os <<"[";
        for(SizeType j=0; j!=A.column_size(); ++j) {
            if(j!=0) { os <<","; }
        if constexpr (HasPrecisionType<X>) { os << python_literal(A[i][j]); }
        else { os <<python_representation(A[i][j]); }
        }
        os << "]";
    }
    os << "]";
    if constexpr (HasPrecisionType<X>) { os << "," << A.zero_element().precision(); }
    os << ")";
    return os;
}

} // namespace Ariadne



template<class X>
Void define_vector_constructors(pybind11::module& module, pybind11::class_<Vector<X>>& vector_class)
{
    vector_class.def(pybind11::init<Vector<X>>());
    if constexpr (DefaultConstructible<X>) {
        vector_class.def(pybind11::init<Nat>());
        vector_class.def_static("unit",(Vector<X>(*)(SizeType,SizeType)) &Vector<X>::unit);
        vector_class.def_static("basis",(Array<Vector<X>>(*)(SizeType)) &Vector<X>::basis);
    }
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        if constexpr (Constructible<X,Nat,PR>) {
            vector_class.def(pybind11::init<Nat,PR>());
            vector_class.def_static("unit",(Vector<X>(*)(SizeType,SizeType,PR)) &Vector<X>::unit);
            vector_class.def_static("basis",(Array<Vector<X>>(*)(SizeType,PR)) &Vector<X>::basis);
        }
    }
    vector_class.def(pybind11::init<Nat,X>());
    vector_class.def(pybind11::init([](pybind11::list const& lst){return vector_from_python<X>(lst);}));

    // Convert from a Python list and properties
    if constexpr (HasGenericType<X> and HasPrecisionType<X>) {
        typedef typename X::GenericType Y; typedef typename X::PrecisionType PR;
        if constexpr(Constructible<Vector<X>,Vector<Real>,PR>) {
            vector_class.def(pybind11::init<Vector<Real>,PR>());
        }
        if constexpr(Constructible<Vector<X>,Vector<Y>,PR>) {
            vector_class.def(pybind11::init<Vector<Y>,PR>());
            vector_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Vector<X>(vector_from_python<Y>(lst),pr);}));
        } else if constexpr(Constructible<Vector<X>,Vector<Dyadic>,PR>) {
            vector_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Vector<X>(vector_from_python<Dyadic>(lst),pr);}));
        }
    }

    pybind11::implicitly_convertible<pybind11::list,Vector<X>>();
}

template<class Y, class X>
Void define_vector_conversion(pybind11::module& module, pybind11::class_<Vector<X>>& vector_class)
{
    vector_class.def(pybind11::init<Vector<Y>>());
    pybind11::implicitly_convertible<Vector<Y>,Vector<X>>();
}

template<class X, class Y>
Void define_mixed_vector_arithmetic(pybind11::module& module, pybind11::class_<Vector<X>>& vector_class)
{
    vector_class.def("__add__",__add__<Vector<X>,Vector<Y> , Return<Vector<SumType<X,Y>>>>, pybind11::is_operator());
    vector_class.def("__sub__",__sub__<Vector<X>,Vector<Y> , Return<Vector<DifferenceType<X,Y>>> >, pybind11::is_operator());
    vector_class.def("__rmul__",__rmul__<Vector<X>,Y , Return<Vector<ProductType<Y,X>>> >, pybind11::is_operator());
    vector_class.def("__mul__",__mul__<Vector<X>,Y , Return<Vector<ProductType<X,Y>>> >, pybind11::is_operator());
    vector_class.def(__py_div__,__div__<Vector<X>,Y , Return<Vector<QuotientType<X,Y>>> >, pybind11::is_operator());
    module.def("dot",  &_dot_<Vector<X>,Vector<Y>>);
    // Don't use operators self+other (as below) because of
    // below) because of need to convert result expressions to Vector<R>.
    // vector_class.def(self+=Vector<Y>());
    // vector_class.def(self*=Y());
}

template<class X>
Void define_vector(pybind11::module& module, pybind11::class_<Vector<X>>& vector_class)
{
    define_vector_constructors(module,vector_class);
    define_vector_concept(module,vector_class);

//    module.def("join", &_join_<Vector<X>,Vector<X>>);
//    module.def("join", &_join_<Vector<X>,X>);
//    module.def("join", &_join_<X,Vector<X>>);
//    module.def("join", &__sjoin__<X>);

//    vector_class.def(pybind11::init(&vector_from_python<X>));
//    pybind11::implicitly_convertible<pybind11::list, Vector<X>>();
}


template<ARawFloat F> Void define_vector(pybind11::module& module, pybind11::class_<Vector<F>>& vector_class) {
    using X=F;
    define_vector_constructors(module, vector_class);
    define_vector_operations(module, vector_class);
    define_vector_arithmetic<Vector<X>,X>(module,vector_class);

    vector_class.def("__rmul__",&__rmul__<Vector<F>,Scalar<Bounds<F>>>, pybind11::is_operator());
    vector_class.def("__rmul__",&__rmul__<Vector<F>,Scalar<Approximation<F>>>, pybind11::is_operator());

    module.def("norm",&_norm_<Vector<X>>);
    module.def("sup_norm",&_sup_norm_<Vector<X>>);
    module.def("dot",&_dot_<Vector<X>,Vector<X>>);
}

template<class F> Void define_vector(pybind11::module& module, pybind11::class_<Vector<Bounds<F>>>& vector_class) {
    using X=Bounds<F>;
    using PR=PrecisionType<X>;
    define_vector_constructors(module, vector_class);
    define_vector_concept(module, vector_class);
    module.def("norm",&_norm_<Vector<X>>);
    module.def("sup_norm",&_sup_norm_<Vector<X>>);
    module.def("two_norm",&_two_norm_<Vector<X>>);
    module.def("dot",&_dot_<Vector<X>,Vector<X>>);

    vector_class.def("__mul__",&__mul__<Vector<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    vector_class.def("__rmul__",&__rmul__<Vector<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    vector_class.def(__py_div__,&__div__<Vector<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());

    module.def("refinement",(Vector<X>(*)(Vector<X>const&,Vector<X>const&)) &refinement);
    module.def("refines",(bool(*)(Vector<X>const&,Vector<X>const&)) &refines);
    module.def("inconsistent",(bool(*)(Vector<X>const&,Vector<X>const&)) &inconsistent);

//    vector_class.def(pybind11::init<Vector<ValidatedNumber>,PR>());
//    vector_class.def(pybind11::init<Array<ValidatedNumber>,PR>());
    vector_class.def(pybind11::init<Vector<Rational>,PR>());
    vector_class.def(pybind11::init<Vector<F>>());
    pybind11::implicitly_convertible<Vector<F>,Vector<Bounds<F>>>();

}

template<class F> Void define_vector(pybind11::module& module, pybind11::class_<Vector<Approximation<F>>>& vector_class)
{
    using X=Approximation<F>;
    using PR=PrecisionType<X>;
    define_vector_constructors(module, vector_class);
    define_vector_concept(module, vector_class);
    vector_class.def("__mul__",&__mul__<Vector<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());
    vector_class.def("__rmul__",&__rmul__<Vector<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());
    vector_class.def(__py_div__,&__div__<Vector<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());
    module.def("norm",&_norm_<Vector<X>>);
    module.def("sup_norm",&_sup_norm_<Vector<X>>);
    module.def("two_norm",&_two_norm_<Vector<X>>);
    module.def("dot",&_dot_<Vector<X>,Vector<X>>);

//    vector_class.def(pybind11::init<Vector<ApproximateNumber>,PR>());
//    vector_class.def(pybind11::init<Array<ApproximateNumber>,PR>());
    vector_class.def(pybind11::init<Vector<Rational>,PR>());
    vector_class.def(pybind11::init<Vector<Bounds<F>>>());
    pybind11::implicitly_convertible<Vector<Bounds<F>>,Vector<Approximation<F>>>();
    pybind11::implicitly_convertible<Vector<F>,Vector<Approximation<F>>>();

    module.def("cast_exact", (Vector<ExactType<X>>(*)(Vector<X> const&)) &cast_exact);
}


template<class X> pybind11::class_<Vector<X>> export_vector(pybind11::module& module)
{
    pybind11::class_<Vector<X>> vector_class(module,python_class_name<Vector<X>>().c_str());
    define_vector(module, vector_class);
    return vector_class;
}


template<class X>
Void define_covector(pybind11::module& module, pybind11::class_<Covector<X>>& covector_class)
{
    covector_class.def(pybind11::init<Covector<X>>());
    if constexpr (DefaultConstructible<X>) {
        covector_class.def(pybind11::init<Nat>()); }
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        covector_class.def(pybind11::init<Nat,PR>());
    }
    covector_class.def(pybind11::init<Nat,X>());
    covector_class.def(pybind11::init([](pybind11::list const& lst){return covector_from_python<X>(lst);}));
    covector_class.def("size", &Covector<X>::size);
    covector_class.def("__len__", &Covector<X>::size);
    covector_class.def("__setitem__", &__setitem__<Covector<X>,Nat,X>);
    covector_class.def("__getitem__", &__getitem__<Covector<X>,Nat>);
    covector_class.def("__str__",&__cstr__<Covector<X>>);
    covector_class.def("__repr__",&__repr__<Covector<X>>);

    covector_class.def("__pos__",__pos__<Covector<X> , Return<Covector<X>> >, pybind11::is_operator());
    covector_class.def("__neg__",__neg__<Covector<X> , Return<Covector<NegationType<X>>> >, pybind11::is_operator());
    covector_class.def("__add__",__add__<Covector<X>,Covector<X> , Return<Covector<SumType<X,X>>> >, pybind11::is_operator());
    covector_class.def("__radd__",__radd__<Covector<X>,Covector<X> , Return<Covector<SumType<X,X>>> >, pybind11::is_operator());
    covector_class.def("__sub__",__sub__<Covector<X>,Covector<X> , Return<Covector<DifferenceType<X,X>>> >, pybind11::is_operator());
    covector_class.def("__rsub__",__rsub__<Covector<X>,Covector<X> , Return<Covector<DifferenceType<X,X>>> >, pybind11::is_operator());
    covector_class.def("__mul__",__mul__<Covector<X>,X , Return<Covector<ProductType<X,X>>> >, pybind11::is_operator());
    covector_class.def("__rmul__",__rmul__<Covector<X>,X , Return<Covector<ProductType<X,X>>> >, pybind11::is_operator());
    covector_class.def(__py_div__,__div__<Covector<X>,X , Return<Covector<QuotientType<X,X>>> >, pybind11::is_operator());

    if constexpr (HasGenericType<X>) {
        typedef GenericType<X> Y;
        covector_class.def("__mul__",__mul__<Covector<X>,Y , Return<Covector<ProductType<X,Y>>> >, pybind11::is_operator());
        covector_class.def("__rmul__",__rmul__<Covector<X>,Y , Return<Covector<ProductType<X,Y>>> >, pybind11::is_operator());
        covector_class.def(__py_div__,__div__<Covector<X>,Y , Return<Covector<QuotientType<X,Y>>> >, pybind11::is_operator());
    }

    covector_class.def("__mul__",__mul__<Covector<X>,Vector<X> , Return<ProductType<X,X>> >, pybind11::is_operator());

    module.def("transpose", (Covector<X>const&(*)(Vector<X>const&)) &transpose);
    module.def("transpose", (Vector<X>const&(*)(Covector<X>const&)) &transpose);

    module.def("cojoin", &_cojoin_<Covector<X>,Covector<X>>);
    module.def("cojoin", &_cojoin_<Covector<X>,Scalar<X>>);
    module.def("cojoin", &_cojoin_<Scalar<X>,Covector<X>>);

    covector_class.def(pybind11::init([](pybind11::list const& lst){return covector_from_python<X>(lst);}));

    // Convert from a Python list and properties
    if constexpr (HasGenericType<X> and HasPrecisionType<X>) {
        typedef typename X::GenericType Y; typedef typename X::PrecisionType PR;
        if constexpr(Constructible<Covector<X>,Covector<Y>,PR>) {
            covector_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Covector<X>(covector_from_python<Y>(lst),pr);}));
        } else if constexpr(Constructible<Covector<X>,Covector<Dyadic>,PR>) {
            covector_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Covector<X>(covector_from_python<Dyadic>(lst),pr);}));
        }
    }
}

template<class X>
Void define_covector_conversions(pybind11::module& module, pybind11::class_<Covector<X>>& covector_class) {
}

template<class F>
Void define_covector_conversions(pybind11::module& module, pybind11::class_<Covector<Approximation<F>>>& covector_class) {
    covector_class.def(pybind11::init<Covector<Bounds<F>>>());
    pybind11::implicitly_convertible<Covector<Bounds<F>>,Covector<Approximation<F>>>();
}

template<class X>
pybind11::class_<Covector<X>> export_covector(pybind11::module& module)
{
    pybind11::class_<Covector<X>> covector_class(module,python_class_name<Covector<X>>().c_str());
    define_covector(module,covector_class);
    define_covector_conversions(module,covector_class);
    return covector_class;
}



template<class X> class Tag { };

template<class X>
Void define_matrix_class(pybind11::module& module, pybind11::class_<Matrix<X>>& matrix_class)
{
    matrix_class.def(pybind11::init<Matrix<X>>());
    if constexpr (DefaultConstructible<X>) {
        matrix_class.def(pybind11::init<Nat,Nat>());
        matrix_class.def_static("identity",(Matrix<X>(*)(SizeType)) &Matrix<X>::identity);
    }
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        matrix_class.def(pybind11::init<Nat,Nat,PR>());
        matrix_class.def_static("identity",(Matrix<X>(*)(SizeType,PR)) &Matrix<X>::identity);
    }
    matrix_class.def(pybind11::init<Nat,Nat,X>());
    matrix_class.def("rows", &Matrix<X>::row_size);
    matrix_class.def("columns", &Matrix<X>::column_size);
    matrix_class.def("row_size", &Matrix<X>::row_size);
    matrix_class.def("column_size", &Matrix<X>::column_size);
    matrix_class.def("__setitem__", &__msetitem__<X>);
    matrix_class.def("__getitem__", &__mgetitem__<X>);
    matrix_class.def("__str__",&__cstr__<Matrix<X>>);
    matrix_class.def("__repr__",&__repr__<Matrix<X>>);

    module.def("join", &_join_<Matrix<X>,Matrix<X>>);
    module.def("join", &_join_<Matrix<X>,Covector<X>>);
    module.def("join", &_join_<Covector<X>,Matrix<X>>);
    module.def("cojoin", &_cojoin_<Matrix<X>,Matrix<X>>);
    module.def("cojoin", &_cojoin_<Matrix<X>,Vector<X>>);
    module.def("cojoin", &_cojoin_<Vector<X>,Matrix<X>>);

    matrix_class.def(pybind11::init([](pybind11::list const& lst){return matrix_from_python<X>(lst);}));
    pybind11::implicitly_convertible<pybind11::list,Matrix<X>>();

    if constexpr (HasGenericType<X>) {
        typedef typename X::GenericType Y; typedef typename X::PrecisionType PR;
        if constexpr(Constructible<Matrix<X>,Matrix<Y>,PR>) {
            matrix_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Matrix<X>(matrix_from_python<Y>(lst),pr);}));
        } else if constexpr(Constructible<Matrix<X>,Matrix<Dyadic>,PR>) {
            matrix_class.def(pybind11::init([](pybind11::list const& lst, PR pr){return Matrix<X>(matrix_from_python<Dyadic>(lst),pr);}));
        }
    }


}


template<class Y, class X>
Void define_matrix_conversion(pybind11::module& module, pybind11::class_<Matrix<X>>& matrix_class)
{
    matrix_class.def(pybind11::init<Matrix<Y>>());
}

template<class X, class Y>
Void define_matrix_arithmetic(pybind11::module& module, pybind11::class_<Matrix<X>>& matrix_class)
{
    matrix_class.def("__pos__", &__pos__<Matrix<X> , Return<Matrix<X>> >, pybind11::is_operator());
    matrix_class.def("__neg__", &__neg__<Matrix<X> , Return<Matrix<X>> >, pybind11::is_operator());
    matrix_class.def("__add__", &__add__<Matrix<X>,Matrix<Y> , Return<Matrix<SumType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__radd__", &__radd__<Matrix<X>,Matrix<Y> , Return<Matrix<SumType<Y,X>>> >, pybind11::is_operator());
    matrix_class.def("__sub__", &__sub__<Matrix<X>,Matrix<Y> , Return<Matrix<DifferenceType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__rsub__", &__rsub__<Matrix<X>,Matrix<Y> , Return<Matrix<DifferenceType<Y,X>>> >, pybind11::is_operator());
    matrix_class.def("__mul__", &__mul__<Matrix<X>,Y , Return<Matrix<ArithmeticType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__rmul__", &__rmul__<Matrix<X>,Y , Return<Matrix<ProductType<Y,X>>> >, pybind11::is_operator());
    matrix_class.def(__py_div__, &__div__<Matrix<X>,Y , Return<Matrix<QuotientType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__mul__", &__mul__<Matrix<X>,Vector<Y> , Return<Vector<ArithmeticType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__mul__", &__mul__<Matrix<X>,Matrix<Y> , Return<Matrix<ArithmeticType<X,Y>>> >, pybind11::is_operator());
    matrix_class.def("__rmul__", &__rmul__<Matrix<X>,Covector<Y> , Return<Covector<ArithmeticType<Y,X>>> >, pybind11::is_operator());
    matrix_class.def("__rmul__", &__rmul__<Matrix<X>,Matrix<Y> , Return<Matrix<ArithmeticType<Y,X>>> >, pybind11::is_operator());
}

template<class X>
Void define_matrix_operations(pybind11::module& module, pybind11::class_<Matrix<X>>& matrix_class)
{
    module.def("norm", &_norm_<Matrix<X>>);
    module.def("transpose",(Matrix<X>(*)(const Matrix<X>&)) &transpose);

    if constexpr (Same<QuotientType<X>,ArithmeticType<X>>) {
        module.def("inverse",(Matrix<ArithmeticType<X>>(*)(const Matrix<X>&)) &inverse);
        module.def("solve",(Matrix<ArithmeticType<X>>(*)(const Matrix<X>&,const Matrix<X>&)) &solve<X>);
        module.def("solve",(Vector<ArithmeticType<X>>(*)(const Matrix<X>&,const Vector<X>&)) &solve<X>);
    }
}


Void define_matrix(pybind11::module& module, pybind11::class_<Matrix<Real>>& matrix_class)
{
    using X=Real;
    define_matrix_class<X>(module,matrix_class);
}

template<ARawFloat F> Void define_matrix(pybind11::module& module, pybind11::class_<Matrix<F>>& matrix_class)
{
    using X=F;
    define_matrix_class<X>(module,matrix_class);
    define_matrix_arithmetic<X,X>(module,matrix_class);

    matrix_class.def("__mul__",&__mul__<Matrix<F>,Scalar<Bounds<F>>>, pybind11::is_operator());
    matrix_class.def("__mul__",&__mul__<Matrix<F>,Vector<Bounds<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<F>,Scalar<Bounds<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<F>,Covector<Bounds<F>>>, pybind11::is_operator());

    matrix_class.def("__mul__",&__mul__<Matrix<F>,Scalar<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__mul__",&__mul__<Matrix<F>,Vector<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<F>,Scalar<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<F>,Covector<Approximation<F>>>, pybind11::is_operator());

    matrix_class.def("__mul__",&__mul__<Matrix<F>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<F>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    matrix_class.def(__py_div__,&__div__<Matrix<F>,Scalar<ValidatedNumber>>, pybind11::is_operator());
}

template<class F> Void define_matrix(pybind11::module& module, pybind11::class_<Matrix<Bounds<F>>>& matrix_class)
{
    using X=Bounds<F>;
    using PR=PrecisionType<X>;
    define_matrix_class<X>(module,matrix_class);
    matrix_class.def(pybind11::init<Matrix<Rational>, PR>());
    define_matrix_conversion<Float<PR>,FloatBounds<PR>>(module,matrix_class);
    define_matrix_arithmetic<X,X>(module,matrix_class);
    define_matrix_operations<X>(module,matrix_class);
    module.def("gs_inverse", (Matrix<X>(*)(const Matrix<X>&)) &gs_inverse);
    module.def("lu_inverse", (Matrix<X>(*)(const Matrix<X>&)) &lu_inverse);
    module.def("gs_solve", (Vector<X>(*)(const Matrix<X>&,const Vector<X>&)) &gs_solve);
    module.def("gs_solve", (Matrix<X>(*)(const Matrix<X>&,const Matrix<X>&)) &gs_solve);
    module.def("lu_solve", (Matrix<X>(*)(const Matrix<X>&,const Matrix<X>&)) &lu_solve);
    module.def("lu_solve", (Vector<X>(*)(const Matrix<X>&,const Vector<X>&)) &lu_solve);

    module.def("triangular_decomposition",&triangular_decomposition<X>);
    module.def("gram_schmidt_orthogonalisation", (Tuple<Matrix<X>,Matrix<X>>(*)(Matrix<X>const&)) &gram_schmidt_orthogonalisation<X>);
    module.def("orthogonal_decomposition", (Tuple<Matrix<X>,Matrix<X>>(*)(Matrix<X>const&)) &orthogonal_decomposition<X>);

    matrix_class.def("__mul__",&__mul__<Matrix<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());
    matrix_class.def(__py_div__,&__div__<Matrix<Bounds<F>>,Scalar<ValidatedNumber>>, pybind11::is_operator());

    matrix_class.def("__mul__",&__rmul__<Matrix<Bounds<F>>,Scalar<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__mul__",&__mul__<Matrix<Bounds<F>>,Vector<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<Bounds<F>>,Scalar<Approximation<F>>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<Bounds<F>>,Covector<Approximation<F>>>, pybind11::is_operator());

    pybind11::implicitly_convertible<Matrix<F>, Matrix<Bounds<F>>>();
}

template<class F> Void define_matrix(pybind11::module& module, pybind11::class_<Matrix<Approximation<F>>>& matrix_class)
{
    using X=Approximation<F>;
    using PR=PrecisionType<X>;
    define_matrix_class<X>(module,matrix_class);
    matrix_class.def(pybind11::init<Matrix<Rational>, PR>());
    define_matrix_conversion<FloatBounds<PR>,FloatApproximation<PR>>(module,matrix_class);
    define_matrix_arithmetic<X,X>(module,matrix_class);
    define_matrix_operations<X>(module,matrix_class);

    matrix_class.def("__mul__",&__mul__<Matrix<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());
    matrix_class.def("__rmul__",&__rmul__<Matrix<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());
    matrix_class.def(__py_div__,&__div__<Matrix<Approximation<F>>,Scalar<ApproximateNumber>>, pybind11::is_operator());

    module.def("triangular_decomposition",&triangular_decomposition<X>);
    //module.def("householder_orthogonalisation", (Tuple<Matrix<X>,Matrix<X>>(*)(Matrix<X>const&)) &householder_orthogonalisation<X>);
    module.def("gram_schmidt_orthogonalisation", (Tuple<Matrix<X>,Matrix<X>>(*)(Matrix<X>const&)) &gram_schmidt_orthogonalisation<X>);
    module.def("orthogonal_decomposition", (Tuple<Matrix<X>,Matrix<X>>(*)(Matrix<X>const&)) &orthogonal_decomposition<X>);
    module.def("row_norms",(Vector<X>(*)(const Matrix<X>&)) &row_norms<X>);

    pybind11::implicitly_convertible<Matrix<Bounds<F>>, Matrix<Approximation<F>>>();

    module.def("cast_exact", (Matrix<F>(*)(Matrix<Approximation<F>>const&)) &cast_exact);
    //    to_python<Tuple<FloatMatrix,FloatMatrix,PivotMatrix>>();
}

template<class X> Void define_matrix(pybind11::module& module, pybind11::class_<Matrix<X>>& matrix_class)
{
    matrix_class.def(pybind11::init<Matrix<X>>());
    define_matrix_class<X>(module,matrix_class);
    define_matrix_conversion<X,X>(module,matrix_class);
    define_matrix_arithmetic<X,X>(module,matrix_class);
    define_matrix_operations<X>(module,matrix_class);
}


template<class X> pybind11::class_<Matrix<X>> export_matrix(pybind11::module& module)
{
    pybind11::class_<Matrix<X>> matrix_class(module,python_class_name<Matrix<X>>().c_str());
    define_matrix(module,matrix_class);
    return matrix_class;
}

template<class X> pybind11::class_<DiagonalMatrix<X>> export_diagonal_matrix(pybind11::module& module)
{
    pybind11::class_<DiagonalMatrix<X>> diagonal_matrix_class(module,python_class_name<DiagonalMatrix<X>>().c_str());
    if constexpr (Constructible<X,Nat>) {
        diagonal_matrix_class.def(pybind11::init<SizeType>());
    }
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        diagonal_matrix_class.def(pybind11::init<SizeType,PR>());
    }
    diagonal_matrix_class.def(pybind11::init<Vector<X>>());
    diagonal_matrix_class.def("__setitem__", &DiagonalMatrix<X>::set);
    diagonal_matrix_class.def("__getitem__", &DiagonalMatrix<X>::get);
    diagonal_matrix_class.def("__str__",&__cstr__<DiagonalMatrix<X>>);

    diagonal_matrix_class.def("__neg__", &__neg__<DiagonalMatrix<X> , Return<DiagonalMatrix<X>> >);

    if constexpr (Same<ArithmeticType<X>,X>) {
        diagonal_matrix_class.def("__add__", &__add__<DiagonalMatrix<X>,DiagonalMatrix<X>>, pybind11::is_operator());
        diagonal_matrix_class.def("__sub__", &__sub__<DiagonalMatrix<X>,DiagonalMatrix<X>>, pybind11::is_operator());
        diagonal_matrix_class.def("__mul__", &__mul__<DiagonalMatrix<X>,DiagonalMatrix<X>>, pybind11::is_operator());
        diagonal_matrix_class.def(__py_div__, &__div__<DiagonalMatrix<X>,DiagonalMatrix<X>>, pybind11::is_operator());
        diagonal_matrix_class.def("__mul__", &__mul__<DiagonalMatrix<X>,Vector<X>>, pybind11::is_operator());
        diagonal_matrix_class.def("__mul__", &__mul__<DiagonalMatrix<X>,Matrix<X>>, pybind11::is_operator());
        //diagonal_matrix_class.def("__rmul__", &__rmul__<Covector<X>,DiagonalMatrix<X>>, pybind11::is_operator());
        diagonal_matrix_class.def("__rmul__", &__rmul__<Matrix<X>,DiagonalMatrix<X>>, pybind11::is_operator());

        module.def("inverse", &_inverse_<DiagonalMatrix<X>>);
    }

    return diagonal_matrix_class;
}


pybind11::class_<PivotMatrix> export_pivot_matrix(pybind11::module& module)
{

//    pybind11::implicitly_convertible<PivotMatrix, Matrix<FloatDP>>();

    pybind11::class_<PivotMatrix> pivot_matrix_class(module,"PivotMatrix");
    pivot_matrix_class.def("__str__",&__cstr__<PivotMatrix>);
    pivot_matrix_class.def("__repr__",&__cstr__<PivotMatrix>);
    return pivot_matrix_class;
}


Void linear_algebra_submodule(pybind11::module& module) {
    export_vector<FloatDPApproximation>(module);
    export_vector<FloatDPBounds>(module);
    export_vector<FloatDPBall>(module);
    export_vector<FloatDP>(module);

    export_vector<FloatMPApproximation>(module);
    export_vector<FloatMPBounds>(module);
    export_vector<FloatMPBall>(module);
    export_vector<FloatMP>(module);

    export_vector<FloatMPDPBall>(module);

    export_covector<FloatDPApproximation>(module);
    export_covector<FloatDPBounds>(module);
    export_covector<FloatDP>(module);
    export_covector<FloatMPApproximation>(module);
    export_covector<FloatMPBounds>(module);
    export_covector<FloatMP>(module);

    export_matrix<FloatDPApproximation>(module);
    export_matrix<FloatDPBounds>(module);
    export_matrix<FloatDP>(module);

    export_matrix<FloatMPApproximation>(module);
    export_matrix<FloatMPBounds>(module);
    export_matrix<FloatMP>(module);

    export_pivot_matrix(module);

    export_diagonal_matrix<FloatDPApproximation>(module);
    export_diagonal_matrix<FloatDPBounds>(module);
    export_diagonal_matrix<FloatDP>(module);
    export_diagonal_matrix<FloatMPApproximation>(module);
    export_diagonal_matrix<FloatMPBounds>(module);
    export_diagonal_matrix<FloatMP>(module);

    export_vector<Real>(module);
    export_covector<Real>(module);
    export_matrix<Real>(module);

    export_vector<Rational>(module);
    export_covector<Rational>(module);
    export_matrix<Rational>(module);

    export_vector<Decimal>(module);
//    export_covector<Decimal>(module);
    export_matrix<Decimal>(module);

    export_vector<Dyadic>(module);
    export_covector<Dyadic>(module);
//    export_matrix<Dyadic>(module);

    export_vector<ApproximateNumber>(module);
    export_vector<ValidatedNumber>(module);

    template_<Vector> vector_template(module,python_template_name<Vector>().c_str());
    vector_template.instantiate<FloatDPApproximation>();
    vector_template.instantiate<FloatMPApproximation>();
    vector_template.instantiate<FloatDPBounds>();
    vector_template.instantiate<FloatMPBounds>();
    vector_template.instantiate<FloatDPBall>();
    vector_template.instantiate<FloatMPBall>();
    vector_template.instantiate<FloatMPDPBall>();
    vector_template.instantiate<FloatDP>();
    vector_template.instantiate<FloatMP>();
    vector_template.instantiate<Dyadic>();
    vector_template.instantiate<Decimal>();
    vector_template.instantiate<Rational>();
    vector_template.instantiate<Real>();
    vector_template.instantiate<ApproximateNumber>();
    vector_template.instantiate<ValidatedNumber>();

    template_<Covector> covector_template(module,python_template_name<Covector>().c_str());
    covector_template.instantiate<FloatDPApproximation>();
    covector_template.instantiate<FloatMPApproximation>();
    covector_template.instantiate<FloatDPBounds>();
    covector_template.instantiate<FloatMPBounds>();
    covector_template.instantiate<FloatDP>();
    covector_template.instantiate<FloatMP>();
    covector_template.instantiate<Dyadic>();
    covector_template.instantiate<Rational>();
    covector_template.instantiate<Real>();

    template_<Matrix> matrix_template(module,python_template_name<Matrix>().c_str());
    matrix_template.instantiate<FloatDPApproximation>();
    matrix_template.instantiate<FloatMPApproximation>();
    matrix_template.instantiate<FloatDPBounds>();
    matrix_template.instantiate<FloatMPBounds>();
    matrix_template.instantiate<FloatDP>();
    matrix_template.instantiate<FloatMP>();
    matrix_template.instantiate<Decimal>();
    matrix_template.instantiate<Rational>();
    matrix_template.instantiate<Real>();

    template_<DiagonalMatrix> diagonal_matrix_template(module,python_template_name<DiagonalMatrix>().c_str());
    diagonal_matrix_template.instantiate<FloatDPApproximation>();
    diagonal_matrix_template.instantiate<FloatMPApproximation>();
    diagonal_matrix_template.instantiate<FloatDPBounds>();
    diagonal_matrix_template.instantiate<FloatMPBounds>();
    diagonal_matrix_template.instantiate<FloatDP>();
    diagonal_matrix_template.instantiate<FloatMP>();

}
