{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Differentiation
    Description :  partial derivatives of functions and of function approximations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Partial derivatives of functions and function approximations.
    
    Beware, there is an important difference between differentiating
    a function given by an enclosure approximation and differentiating
    a function approximation in its own right.  A function cannot be
    differentiated via its enclosure - to differentiate a function,
    one typically uses a datatype that performs automatic differentiation.
    
    On the other hand, it is usually possible to differentiate 
    the approximation itself.  The values of such a derivative are unrelated
    to the derivatives of the approximated functions.  If an approximation
    is defined by some boundaries, the derivative of the approximation
    is usually related to the derivative of its boundaries.
    
    The only situation where the two notions of derivative coincide is
    where the approximation is exact, ie it is a real function.
-}

module Numeric.AERN.RmToRn.Differentiation where

import Numeric.AERN.RmToRn.Domain

{-|
    An approximate derivative of an approximated function.
    
    This can be instantiated only for datatypes that explicitly enclose, alongside the value, 
    the partial derivatives of the function, using automatic differentiation or other similar technique.
    The operations in this class then simply pick the appropriate partial
    derivative out of the structure. 
-}
class RoundedDerivative f
    where
    type DerivativeEffortIndicator f
    derivativeDefaultEffort :: f -> DerivativeEffortIndicator f
    {-| approximate the primitive function by integrating the given variable,
        starting at the left endpoint of its domain -}
    partialDerivativeOutEff :: 
        DerivativeEffortIndicator f -> 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
    partialDerivativeInEff :: 
        DerivativeEffortIndicator f -> 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ inner approximation of the partial derivative @df/dx@ -}
    
partialDerivativeOut ::
        RoundedDerivative f => 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
partialDerivativeOut fn var =   
    partialDerivativeOutEff (derivativeDefaultEffort fn) fn var

partialDerivativeIn ::
        RoundedDerivative f => 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
partialDerivativeIn fn var =   
    partialDerivativeInEff (derivativeDefaultEffort fn) fn var

{-|
    An approximate derivative of a function approximation.
    
    A measure of the speed of change of the approximation itself as
    its variables are changing within their domains.
    
    Note that these operations are unrelated to the derivatives
    of functions approximated by this approximation.
    
    These operations are used mainly to: 
    
        1. try to detect whether an approximation is monotone
    
        2. shink or extend the domains of variables in order to reach or avoid certain values
        
    All these applications are non-extensional and should hidden from end users 
    of the approximations who should be able to view them only as enclosures 
    of real functions.
    
    We provide them at this level so that some optimisations can be implemented
    generically for all types.
-}
class RoundedFakeDerivative f
    where
    type FakeDerivativeEffortIndicator f
    fakeDerivativeDefaultEffort :: f -> FakeDerivativeEffortIndicator f
    {-| approximate the primitive function by integrating the given variable,
        starting at the left endpoint of its domain -}
    fakePartialDerivativeOutEff :: 
        FakeDerivativeEffortIndicator f -> 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
    fakePartialDerivativeInEff :: 
        FakeDerivativeEffortIndicator f -> 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ inner approximation of the partial derivative @df/dx@ -}
    
fakePartialDerivativeOut ::
        RoundedFakeDerivative f => 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
fakePartialDerivativeOut fn var =   
    fakePartialDerivativeOutEff (fakeDerivativeDefaultEffort fn) fn var
    
fakePartialDerivativeIn ::
        RoundedFakeDerivative f => 
        f {-^ approximation of a function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ -} -> 
        f {-^ outer approximation of the partial derivative @df/dx@ -}
fakePartialDerivativeIn fn var =   
    fakePartialDerivativeInEff (fakeDerivativeDefaultEffort fn) fn var

    
    