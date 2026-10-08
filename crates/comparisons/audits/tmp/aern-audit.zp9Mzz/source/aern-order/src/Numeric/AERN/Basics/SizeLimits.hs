{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.Basics.SizeLimits
    Description :  intrinsic size limit such as precision
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Intrinsic size limit such as precision.
-}
module Numeric.AERN.Basics.SizeLimits where

import Numeric.AERN.Basics.Effort

{-|
    An abstraction of concepts such as precision in MPFR numbers, maximum degree of a polynomial.
    An intrinsic limitation on the size or precision of a value and values derived
    from it by common operations.
-}
class HasSizeLimits f where
    type SizeLimits f
    getSizeLimits :: f -> SizeLimits f
    defaultSizeLimits :: f -> SizeLimits f

{-|
    Ability to increase or decrease a size limit, leading to a value conversion.

    For most types, the instance will define either the Up/Dn operations
    or the In/Out operations and let the others throw an exception.
-}
class
    (EffortIndicator (SizeLimitsChangeEffort f)) 
    => 
    CanChangeSizeLimits f 
    where
    type SizeLimitsChangeEffort f
    sizeLimitsChangeDefaultEffort :: f -> SizeLimitsChangeEffort f
    changeSizeLimitsDnEff :: SizeLimitsChangeEffort f -> SizeLimits f -> f -> f
    changeSizeLimitsUpEff :: SizeLimitsChangeEffort f -> SizeLimits f -> f -> f
    changeSizeLimitsOutEff :: SizeLimitsChangeEffort f -> SizeLimits f -> f -> f
    changeSizeLimitsInEff :: SizeLimitsChangeEffort f -> SizeLimits f -> f -> f

changeSizeLimitsOut :: 
    CanChangeSizeLimits f => 
    SizeLimits f -> f -> f
changeSizeLimitsOut limits a =
    changeSizeLimitsOutEff (sizeLimitsChangeDefaultEffort a) limits a 

changeSizeLimitsIn :: 
    CanChangeSizeLimits f => 
    SizeLimits f -> f -> f
changeSizeLimitsIn limits a =
    changeSizeLimitsInEff (sizeLimitsChangeDefaultEffort a) limits a 

changeSizeLimitsDn :: 
    CanChangeSizeLimits f => 
    SizeLimits f -> f -> f
changeSizeLimitsDn limits a =
    changeSizeLimitsDnEff (sizeLimitsChangeDefaultEffort a) limits a 

changeSizeLimitsUp :: 
    CanChangeSizeLimits f => 
    SizeLimits f -> f -> f
changeSizeLimitsUp limits a =
    changeSizeLimitsUpEff (sizeLimitsChangeDefaultEffort a) limits a 

