/***************************************************************************
 *            function_model.tpl.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "function/function_model.hpp"

#include "function/function_interface.hpp"
#include "function/function_mixin.hpp"
#include "function/function.hpp"

#include "numeric/operators.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/operations.hpp"
#include "function/domain.hpp"

namespace Ariadne {


/*
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactory<P,PR,PRE>::create_constant(const DomainType& domain, const CanonicalNumericType<P,PR,PRE>& value) const {
    return ScalarMultivariateFunctionModel<P,PR,PRE>(this->_ptr->_create(domain,EffectiveScalarMultivariateFunction::zero(domain.size())))+value; };
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactory<P,PR,PRE>::create_constants(const DomainType& domain, const Vector<Number<P>>& values) const {
    typename CanonicalNumericType<P,PR,PRE>::PrecisionType pr=this->_ptr->create_number(0).precision();
    Vector<CanonicalNumericType<P,PR,PRE>> concrete_values(values,pr); return this->_ptr->create_constants(domain,concrete_values); }
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactory<P,PR,PRE>::create_constants(const DomainType& domain, const Vector<CanonicalNumericType<P,PR,PRE>>& values) const {
    return VectorMultivariateFunctionModel<P,PR,PRE>(this->_ptr->_create(domain,EffectiveVectorMultivariateFunction::zeros(values.size(),domain.size())))+values; };
*/


/*
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create(const DomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const {
    return this->_create(domain,function);
}
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create(const DomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const {
    return this->_create(domain,function);
}

template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_zero(const DomainType& domain) const {
    return this->_create(domain,EffectiveScalarMultivariateFunction::zero(domain.size())); }
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_zeros(SizeType result_size, const DomainType& domain) const {
    return this->_create(domain,EffectiveVectorMultivariateFunction::zeros(result_size,domain.size())); }
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_constant(const DomainType& domain, const Number<P>& value) const {
    CanonicalNumericType<P,PR,PRE> concrete_value=this->create_number(value); return this->create_constant(domain,concrete_value); }
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_constant(const DomainType& domain, const CanonicalNumericType<P,PR,PRE>& value) const {
    return ScalarMultivariateFunctionModel<P,PR,PRE>(this->_create(domain,EffectiveScalarMultivariateFunction::zero(domain.size())))+value; };
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_constants(const DomainType& domain, const Vector<Number<P>>& values) const {
    typename CanonicalNumericType<P,PR,PRE>::PrecisionType pr=this->create_number(0).precision();
    Vector<CanonicalNumericType<P,PR,PRE>> concrete_values(values,pr); return this->create_constants(domain,concrete_values); }
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_constants(const DomainType& domain, const Vector<CanonicalNumericType<P,PR,PRE>>& values) const {
    return VectorMultivariateFunctionModel<P,PR,PRE>(this->_create(domain,EffectiveVectorMultivariateFunction::zeros(values.size(),domain.size())))+values; };
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_coordinate(const DomainType& domain, SizeType index) const {
    return ScalarMultivariateFunctionModel<P,PR,PRE>(this->_create(domain,EffectiveScalarMultivariateFunction::coordinate(domain.size(),index))); };
template<class P, class PR, class PRE> ScalarMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_identity(const IntervalDomainType& domain) const {
    return this->_create(DomainType(1,domain),EffectiveScalarMultivariateFunction::coordinate(1,0)); };
template<class P, class PR, class PRE> VectorMultivariateFunctionModel<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_identity(const DomainType& domain) const {
    return this->_create(domain,EffectiveVectorMultivariateFunction::identity(domain.size())); };
template<class P, class PR, class PRE> CanonicalNumericType<P,PR,PRE> FunctionModelFactoryInterface<P,PR,PRE>::create_number(const Number<P>& number) const {
    return this->_create(number); }
*/



} // namespace Ariadne
