{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Integration
    Description :  integrators of function enclosures  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Integrators of function enclosures.
-}

module Numeric.AERN.RmToRn.Integration where

import Numeric.AERN.RmToRn.Domain

class RoundedIntegration f where
    type IntegrationEffortIndicator f
    integrationDefaultEffort :: f -> IntegrationEffortIndicator f
    {-| approximate the primitive function by integrating the given variable,
        starting at the left endpoint of its domain -}
    primitiveFunctionOutEff :: 
        IntegrationEffortIndicator f -> 
        f {-^ function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ with values in @[l,r]@ -} -> 
        f {-^ primitive function of @f(x,...)@ by @x@ with @f(l,...) = 0@ -}
    primitiveFunctionInEff :: 
        IntegrationEffortIndicator f -> 
        f {-^ function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ with values in @[l,r]@ -} -> 
        f {-^ primitive function of @f(x,...)@ by @x@ with @f(l,...) = 0@ -}

primitiveFunctionOut ::
        RoundedIntegration f => 
        f {-^ function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ with values in @[l,r]@ -} -> 
        f {-^ primitive function of @f(x,...)@ by @x@ with @f(l,...) = 0@ -}
primitiveFunctionOut fn var =
    primitiveFunctionOutEff (integrationDefaultEffort fn) fn var
    
primitiveFunctionIn ::
        RoundedIntegration f => 
        f {-^ function @f@ -} -> 
        (Var f) {-^ variable @x@ in the domain box of @f@ with values in @[l,r]@ -} -> 
        f {-^ primitive function of @f(x,...)@ by @x@ with @f(l,...) = 0@ -}
primitiveFunctionIn fn var =
    primitiveFunctionInEff (integrationDefaultEffort fn) fn var
    
    