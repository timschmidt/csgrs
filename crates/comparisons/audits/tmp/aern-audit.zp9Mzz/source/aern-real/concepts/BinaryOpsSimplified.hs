{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-
    This module is not used by any other module.
    This module is a proof of concept and an aid in documenting the concept.
    
    This module shows on a simple example how mixed-type binary operators are
    implemented in AERN. 
-}
module BinaryOpsSimplified where

{- approximate addition rounded up or down: -}

class UpDnMixedAddEffort t tn where
    type UpDnMixedAddEffortIndicator t tn
    
class 
    (UpDnMixedAddEffort t tn)
    =>
    UpDnMixedAdd t tn 
    where
    mxAddUpEff :: UpDnMixedAddEffortIndicator t tn -> t -> tn -> t
    mxAddDnEff :: UpDnMixedAddEffortIndicator t tn -> t -> tn -> t

{- approximate addition rounded inwards or outwards: -}
   
class OutMixedAddEffort t tn where
    type OutMixedAddEffortIndicator t tn
    
class 
    (OutMixedAddEffort t tn)
    =>
    OutMixedAdd t tn 
    where
    mxAddOutEff :: OutMixedAddEffortIndicator t tn -> t -> tn -> t

{- The following function represents algorithms that are
   defined generically for two types t1 and t2 that may
   sometime be the same type.  -}

genericAlgorithm ::
    (OutMixedAdd t1 t2) 
    => 
    OutMixedAddEffortIndicator t1 t2 -> t1 -> t2 -> t1    
genericAlgorithm = mxAddOutEff

{- Mock interval and polynomial types: -}

data I t = I t
data P cf = P cf

type DI = I Double
type Poly = P DI
type PI = I Poly

{- The goal: write instances such that the generic algorithm can applied to different concrete types: -}

test1 :: () -> DI -> DI -> DI
test1 = genericAlgorithm

test2 :: () -> PI -> PI -> PI
test2 = genericAlgorithm

test3 :: () -> PI -> DI -> PI
test3 = genericAlgorithm


{- To get OutMixedAdd DI DI (test1), we use:

   * UpDnMixedAdd e1 e2 => OutMixedAdd (I e1) (I e2)

   * UpDnMixedAdd Double Double
-}

{- Outwards rounded addition for intervals: -}

instance
    (UpDnMixedAddEffort e1 e2)
    => 
    OutMixedAddEffort (I e1) (I e2) 
    where
    type OutMixedAddEffortIndicator (I e1) (I e2) = 
        UpDnMixedAddEffortIndicator e1 e2
    
instance
    (UpDnMixedAdd e1 e2)
    => 
    OutMixedAdd (I e1) (I e2)
    where
    mxAddOutEff eff (I v1) (I v2) =
        I (mxAddUpEff eff v1 v2)

{- A pseudo-mixed up/dn-rounded addition of Doubles: -}
        
instance
    UpDnMixedAddEffort Double Double
    where
    type UpDnMixedAddEffortIndicator Double Double = ()

instance
    UpDnMixedAdd Double Double
    where
    mxAddDnEff _ a b = a+b -- dummy
    mxAddUpEff _ a b = a+b -- dummy

{- To get OutMixedAdd PI PI (test2), we use:

   * UpDnMixedAdd e1 e2 => OutMixedAdd (I e1) (I e2)
     (defined for test1)

   * (OutMixedAdd (P cf1) (P cf2)) => UpDnMixedAdd (P cf1) (P cf2)

   * (OutMixedAdd cf1 cf2) => OutMixedAdd (P cf1) (P cf2)
   
   * (OutMixedAdd DI DI)
     (defined for test1)
-}

{- A pseudo-mixed up/dn-rounded addition of polynomials,
   derived from outwards-rounded addition: -}
        
instance
    (OutMixedAddEffort (P cf1) (P cf2))
    =>
    UpDnMixedAddEffort (P cf1) (P cf2) 
    where
    type UpDnMixedAddEffortIndicator (P cf1) (P cf2) =
        OutMixedAddEffortIndicator (P cf1) (P cf2)

instance
    (OutMixedAdd (P cf1) (P cf2))
    =>
    UpDnMixedAdd (P cf1) (P cf2)
    where
    mxAddDnEff = mxAddOutEff -- dummy
    mxAddUpEff = mxAddOutEff -- dummy

{- A pseudo-mixed outwards-rounded addition of polynomials: -}
        
instance
    (OutMixedAddEffort cf1 cf2)
    =>
    OutMixedAddEffort (P cf1) (P cf2) 
    where
    type OutMixedAddEffortIndicator (P cf1) (P cf2) =
        OutMixedAddEffortIndicator cf1 cf2

instance
    (OutMixedAdd cf1 cf2)
    =>
    OutMixedAdd (P cf1) (P cf2)
    where
    mxAddOutEff eff (P c1) (P c2) = P (mxAddOutEff eff c1 c2) -- dummy

{- To get OutMixedAdd PI DI (test3), we use:

   * UpDnMixedAdd e1 e2 => OutMixedAdd (I e1) (I e2)
     (defined for test1)

   * (UpDnMixedAdd (P cf1) (I Double)) => UpDnMixedAdd (P cf1) Double

   * (OutMixedAdd (P cf1) (I e)) => UpDnMixedAdd (P cf1) (I e)
   
   * (OutMixedAdd cf1 (I e)) => OutMixedAdd (P cf1) (I e)

   * (OutMixedAdd DI DI)
     (defined for test1)   
-}

{- Mixed up/dn-rounded addition of polynomials by Doubles: -}

instance
    (UpDnMixedAddEffort (P cf1) (I Double))
    =>
    UpDnMixedAddEffort (P cf1) Double
    where
    type UpDnMixedAddEffortIndicator (P cf1) Double =
        OutMixedAddEffortIndicator (P cf1) (I Double)

instance
    (UpDnMixedAdd (P cf1) (I Double))
    =>
    UpDnMixedAdd (P cf1) Double
    where
    mxAddDnEff eff p d = mxAddDnEff eff p (I d) -- dummy
    mxAddUpEff eff p d = mxAddUpEff eff p (I d) -- dummy

{- Mixed up/dn-rounded addition of polynomials by intervals,
   derived from outwards-rounded addition: -}
        
instance
    (OutMixedAddEffort (P cf1) (I e))
    =>
    UpDnMixedAddEffort (P cf1) (I e) 
    where
    type UpDnMixedAddEffortIndicator (P cf1) (I e) =
        OutMixedAddEffortIndicator (P cf1) (I e)

instance
    (OutMixedAdd (P cf1) (I e))
    =>
    UpDnMixedAdd (P cf1) (I e)
    where
    mxAddDnEff = mxAddOutEff -- dummy
    mxAddUpEff = mxAddOutEff -- dummy

{- Mixed outwards-rounded addition of polynomials by intervals: -}
        
instance
    (OutMixedAddEffort cf1 (I e))
    =>
    OutMixedAddEffort (P cf1) (I e) 
    where
    type OutMixedAddEffortIndicator (P cf1) (I e) =
        OutMixedAddEffortIndicator cf1 (I e)

instance
    (OutMixedAdd cf1 (I e))
    =>
    OutMixedAdd (P cf1) (I e)
    where
    mxAddOutEff eff (P c1) c2 = P (mxAddOutEff eff c1 c2) -- dummy

