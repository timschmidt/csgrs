/***************************************************************************
 *            function/symbolic_function.cpp
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

#include "function/functional.hpp"

#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/differential.hpp"
#include "numeric/operators.hpp"
#include "algebra/algebra.hpp"
#include "function/taylor_model.hpp"
#include "function/formula.hpp"
#include "function/formula.tpl.hpp"
#include "function/function.hpp"

#include "function/symbolic_function.hpp"

#include "function/function_interface.hpp"
#include "function/function_mixin.tpl.hpp"
#include "function/function_wrapper.hpp"

namespace Ariadne {

template class LieDerivativeFunction<ValidatedTag>;
template class FunctionWrapper<LieDerivativeFunction<ValidatedTag>,ValidatedTag,ScalarMultivariate>;


template<> String class_name<Function<ApproximateTag,RealScalar(RealScalar)>>() {
    return "ApproximateScalarUnivariateFunction"; }
template<> String class_name<Function<ValidatedTag,RealScalar(RealScalar)>>() {
    return "ValidatedScalarUnivariateFunction"; }
template<> String class_name<Function<EffectiveTag,RealScalar(RealScalar)>>() {
    return "EffectiveScalarUnivariateFunction"; }
template<> String class_name<Function<ApproximateTag,RealVector(RealScalar)>>() {
    return "ApproximateVectorUnivariateFunction"; }
template<> String class_name<Function<ValidatedTag,RealVector(RealScalar)>>() {
    return "ValidatedVectorUnivariateFunction"; }
template<> String class_name<Function<EffectiveTag,RealVector(RealScalar)>>() {
    return "EffectiveVectorUnivariateFunction"; }

template<> String class_name<Function<ApproximateTag,RealScalar(RealVector)>>() {
    return "ApproximateScalarMultivariateFunction"; }
template<> String class_name<Function<ValidatedTag,RealScalar(RealVector)>>() {
    return "ValidatedScalarMultivariateFunction"; }
template<> String class_name<Function<EffectiveTag,RealScalar(RealVector)>>() {
    return "EffectiveScalarMultivariateFunction"; }
template<> String class_name<Function<ApproximateTag,RealVector(RealVector)>>() {
    return "ApproximateVectorMultivariateFunction"; }
template<> String class_name<Function<ValidatedTag,RealVector(RealVector)>>() {
    return "ValidatedVectorMultivariateFunction"; }
template<> String class_name<Function<EffectiveTag,RealVector(RealVector)>>() {
    return "EffectiveVectorMultivariateFunction"; }


template<> String class_name<ScalarUnivariateFormulaFunction<ApproximateNumber>>() { return "ApproximateScalarUnivariateFormulaFunction"; }
template<> String class_name<ScalarUnivariateFormulaFunction<ValidatedNumber>>() { return "ValidatedScalarUnivariateFormulaFunction"; }
template<> String class_name<ScalarUnivariateFormulaFunction<EffectiveNumber>>() { return "EffectiveScalarUnivariateFormulaFunction"; }
template<> String class_name<VectorUnivariateFormulaFunction<ApproximateNumber>>() { return "ApproximateVectorUnivariateFormulaFunction"; }
template<> String class_name<VectorUnivariateFormulaFunction<ValidatedNumber>>() { return "ValidatedVectorUnivariateFormulaFunction"; }
template<> String class_name<VectorUnivariateFormulaFunction<EffectiveNumber>>() { return "EffectiveVectorUnivariateFormulaFunction"; }
template<> String class_name<ScalarFormulaFunction<ApproximateNumber>>() { return "ApproximateScalarMultivariateFormulaFunction"; }
template<> String class_name<ScalarFormulaFunction<ValidatedNumber>>() { return "ValidatedScalarMultivariateFormulaFunction"; }
template<> String class_name<ScalarFormulaFunction<EffectiveNumber>>() { return "EffectiveScalarMultivariateFormulaFunction"; }
template<> String class_name<VectorFormulaFunction<ValidatedNumber>>() { return "ValidatedVectorMultivariateFormulaFunction"; }
template<> String class_name<VectorFormulaFunction<EffectiveNumber>>() { return "EffectiveVectorMultivariateFormulaFunction"; }
template<> String class_name<VectorFormulaFunction<ApproximateNumber>>() { return "ApproximateVectorMultivariateFormulaFunction"; }

template<> String class_name<ConstantFunction<ApproximateNumber,RealScalar>>() { return "ApproximateUnivariateConstantFunction"; }
template<> String class_name<ConstantFunction<ValidatedNumber,RealScalar>>() { return "ValidatedUnivariateConstantFunction"; }
template<> String class_name<ConstantFunction<EffectiveNumber,RealScalar>>() { return "EffectiveUnivariateConstantFunction"; }
template<> String class_name<CoordinateFunction<ApproximateTag,RealScalar>>() { return "ApproximateUnivariateCoordinateFunction"; }
template<> String class_name<CoordinateFunction<ValidatedTag,RealScalar>>() { return "ValidatedUnivariateCoordinateFunction"; }
template<> String class_name<CoordinateFunction<EffectiveTag,RealScalar>>() { return "EffectiveUnivariateCoordinateFunction"; }
template<> String class_name<UnaryFunction<ApproximateTag,RealScalar>>() { return "ApproximateUnivariateUnaryFunction"; }
template<> String class_name<UnaryFunction<ValidatedTag,RealScalar>>() { return "ValidatedUnivariateUnaryFunction"; }
template<> String class_name<UnaryFunction<EffectiveTag,RealScalar>>() { return "EffectiveUnivariateUnaryFunction"; }
template<> String class_name<BinaryFunction<ApproximateTag,RealScalar>>() { return "ApproximateUnivariateBinaryFunction"; }
template<> String class_name<BinaryFunction<ValidatedTag,RealScalar>>() { return "ValidatedUnivariateBinaryFunction"; }
template<> String class_name<BinaryFunction<EffectiveTag,RealScalar>>() { return "EffectiveUnivariateBinaryFunction"; }
template<> String class_name<GradedFunction<ApproximateTag,RealScalar>>() { return "ApproximateUnivariateGradedFunction"; }
template<> String class_name<GradedFunction<ValidatedTag,RealScalar>>() { return "ValidatedUnivariateGradedFunction"; }
template<> String class_name<GradedFunction<EffectiveTag,RealScalar>>() { return "EffectiveUnivariateGradedFunction"; }
template<> String class_name<ConstantFunction<ApproximateNumber,RealVector>>() { return "ApproximateMultivariateConstantFunction"; }
template<> String class_name<ConstantFunction<ValidatedNumber,RealVector>>() { return "ValidatedMultivariateConstantFunction"; }
template<> String class_name<ConstantFunction<EffectiveNumber,RealVector>>() { return "EffectiveMultivariateConstantFunction"; }
template<> String class_name<CoordinateFunction<ApproximateTag,RealVector>>() { return "ApproximateMultivariateCoordinateFunction"; }
template<> String class_name<CoordinateFunction<ValidatedTag,RealVector>>() { return "ValidatedMultivariateCoordinateFunction"; }
template<> String class_name<CoordinateFunction<EffectiveTag,RealVector>>() { return "EffectiveMultivariateCoordinateFunction"; }
template<> String class_name<UnaryFunction<ApproximateTag,RealVector>>() { return "ApproximateMultivariateUnaryFunction"; }
template<> String class_name<UnaryFunction<ValidatedTag,RealVector>>() { return "ValidatedMultivariateUnaryFunction"; }
template<> String class_name<UnaryFunction<EffectiveTag,RealVector>>() { return "EffectiveMultivariateUnaryFunction"; }
template<> String class_name<BinaryFunction<ApproximateTag,RealVector>>() { return "ApproximateMultivariateBinaryFunction"; }
template<> String class_name<BinaryFunction<ValidatedTag,RealVector>>() { return "ValidatedMultivariateBinaryFunction"; }
template<> String class_name<BinaryFunction<EffectiveTag,RealVector>>() { return "EffectiveMultivariateBinaryFunction"; }
template<> String class_name<GradedFunction<ApproximateTag,RealVector>>() { return "ApproximateMultivariateGradedFunction"; }
template<> String class_name<GradedFunction<ValidatedTag,RealVector>>() { return "ValidatedMultivariateGradedFunction"; }
template<> String class_name<GradedFunction<EffectiveTag,RealVector>>() { return "EffectiveMultivariateGradedFunction"; }

template<> String class_name<VectorOfScalarFunction<ApproximateTag,RealScalar>>() { return "ApproximateVectorOfScalarUnivariateFunction"; }
template<> String class_name<VectorOfScalarFunction<ValidatedTag,RealScalar>>() { return "ValidatedVectorOfScalarUnivariateFunction"; }
template<> String class_name<VectorOfScalarFunction<EffectiveTag,RealScalar>>() { return "EffectiveVectorOfScalarUnivariateFunction"; }
template<> String class_name<VectorOfScalarFunction<ApproximateTag,RealVector>>() { return "ApproximateVectorOfScalarMultivariateFunction"; }
template<> String class_name<VectorOfScalarFunction<ValidatedTag,RealVector>>() { return "ValidatedVectorOfScalarMultivariateFunction"; }
template<> String class_name<VectorOfScalarFunction<EffectiveTag,RealVector>>() { return "EffectiveVectorOfScalarMultivariateFunction"; }

template<> String class_name<ComposedFunction<ApproximateTag,RealScalar,RealScalar,RealScalar>>() { return "ApproximateScalarUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealScalar,RealScalar,RealScalar>>() { return "ValidatedScalarUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealScalar,RealScalar,RealScalar>>() { return "EffectiveScalarUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ApproximateTag,RealScalar,RealVector,RealScalar>>() { return "ApproximateScalarUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealScalar,RealVector,RealScalar>>() { return "ValidatedScalarUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealScalar,RealVector,RealScalar>>() { return "EffectiveScalarUnivariateComposedFunction"; }

template<> String class_name<ComposedFunction<ApproximateTag,RealScalar,RealScalar,RealVector>>() { return "ApproximateScalarMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealScalar,RealScalar,RealVector>>() { return "ValidatedScalarMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealScalar,RealScalar,RealVector>>() { return "EffectiveScalarMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ApproximateTag,RealScalar,RealVector,RealVector>>() { return "ApproximateScalarMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealScalar,RealVector,RealVector>>() { return "ValidatedScalarMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealScalar,RealVector,RealVector>>() { return "EffectiveScalarMultivariateComposedFunction"; }

template<> String class_name<ComposedFunction<ApproximateTag,RealVector,RealScalar,RealScalar>>() { return "ApproximateVectorUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealVector,RealScalar,RealScalar>>() { return "ValidatedVectorUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealVector,RealScalar,RealScalar>>() { return "EffectiveVectorUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ApproximateTag,RealVector,RealVector,RealScalar>>() { return "ApproximateVectorUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealVector,RealVector,RealScalar>>() { return "ValidatedVectorUnivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealVector,RealVector,RealScalar>>() { return "EffectiveVectorUnivariateComposedFunction"; }

template<> String class_name<ComposedFunction<ApproximateTag,RealVector,RealScalar,RealVector>>() { return "ApproximateVectorMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealVector,RealScalar,RealVector>>() { return "ValidatedVectorMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealVector,RealScalar,RealVector>>() { return "EffectiveVectorMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ApproximateTag,RealVector,RealVector,RealVector>>() { return "ApproximateVectorMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<ValidatedTag,RealVector,RealVector,RealVector>>() { return "ValidatedVectorMultivariateComposedFunction"; }
template<> String class_name<ComposedFunction<EffectiveTag,RealVector,RealVector,RealVector>>() { return "EffectiveVectorMultivariateComposedFunction"; }


template<> String class_name<JoinedFunction<ValidatedTag,EuclideanDomain,EuclideanDomain,EuclideanDomain>>() {
    return "ValidatedJoinedFunction"; }
template<> String class_name<JoinedFunction<EffectiveTag,EuclideanDomain,EuclideanDomain,EuclideanDomain>>() {
    return "EffectiveJoinedFunction"; }
template<> String class_name<EmbeddedFunction<EffectiveTag,EuclideanDomain,EuclideanDomain,EuclideanDomain,RealDomain>>() {
    return "EffectiveEmbeddedFunction"; }
template<> String class_name<EmbeddedFunction<EffectiveTag,EuclideanDomain,EuclideanDomain,EuclideanDomain,EuclideanDomain>>() {
    return "EffectiveEmbeddedFunction"; }
template<> String class_name<LieDerivativeFunction<ValidatedTag>>() { return "ValidatedLieDerivativeFunction"; }
template<> String class_name<LieDerivativeFunction<EffectiveTag>>() { return "EffectiveLieDerivativeFunction"; }



} // namespace Ariadne
