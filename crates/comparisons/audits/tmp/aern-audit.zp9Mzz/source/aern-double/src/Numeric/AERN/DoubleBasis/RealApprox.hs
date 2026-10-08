{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.DoubleBasis.RealApprox
    Description :  Double intervals for approximating real numbers
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Intervals with Double endpoints as an
    abstract data type for approximating real numbers.
    Each interval represents a single real number.
    Only operations that respect this view are available
    via this module.
-}
module Numeric.AERN.DoubleBasis.RealApprox
(
    -- |
    -- A convenience module re-exporting various interval operations 
    -- with default effort indicators.

    -- * Main type
    RealApprox,
    
    -- * Order relations
    -- | 
    -- There are two types of order relations to consider: 
    -- 
    --   * the /numerical/ order, generalising the order relation
    --     on Doubles and
    -- 
    --   * the /refinement/ order, generalising the reverse-inclusion 
    --     relation on consistent intervals.  
    --
    -- The intervals in 'RealApprox' form a /meet/-semilattice
    -- corresponding to the refiniement order under the operation /\\ 
    -- returning the subset-least interval containing the /union/ of 
    -- its argument intervals.
    -- 
    -- A safe approximation of the exact operation /\\ 
    -- is given by '</\>'.
    --
    -- The dual operation to /\\ is partial 
    -- since the /intersection/ of disjoint sets is empty. 
    -- 
    -- A safe approximation of the exact operation \\/\?
    -- is given by '<\/>?'.

    -- ** Numerical order
    -- | 
    -- Interval extensions of the corresponding tests and relations on 
    -- Double.
    
    -- *** Extrema
    -- |
    -- The values retured by 'least' and 'greatest' complete the 
    -- numerical partial order on 'RealApprox'.
    least,greatest,
    
    -- *** Comparability tests
    (==?),(<==>?),(</=>?),
    
    -- *** Order relations
    (<?),(>?),(<=?),(>=?),
    
    -- ** Refinement order
    -- | 
    -- Tests and relations in the interval poset.
    
    -- *** Extrema
    -- |
    -- The values retured by 'bottom' and 'top' complete the 
    -- refinement partial order on 'RealApprox'.

    -- **** ASCII versions
    bottom,top,
    
    -- **** Unicode versions
    (⊥),(⊤),
    
    -- *** Comparability tests
    (|==?),(|<==>?),(|</=>?),

    -- *** Order relations
    
    -- **** ASCII versions
    (|<?),(|>?),(|<=?),(|>=?),
    
    -- **** Unicode versions
    (⊏?),(⊑?),(⊒?),(⊐?),
    
    -- * Outward rounded operations
    -- | 
    -- Interval extensions of common functions. The 'Num', 'Fractional' 
    -- and 'Floating' instances for 'RealApprox' use such versions as instance 
    -- methods.
    
    -- ** Order operations
    
    -- *** Numerical order
    -- | 
    -- Outward rounded interval extensions of the corresponding 
    -- operations on Double.
    minOut,maxOut,

    -- *** Refinement order
    -- | 
    -- Outward rounded lattice operations in the interval poset.
    
    -- **** ASCII versions
    (</\>),(<\/>?),

    -- **** Unicode versions
    (<⊓>),(<⊔>?),

    -- ** Field operations

    -- *** Interval operations
    (<+>),(<->),(<*>),(</>),

    -- *** Mixed type operations (eg for scaling by an integer)
    (|<+>),(<+>|),(|<*>),(<*>|),(</>|),(<^>),

    -- ** Special constants 
    piOut,eOut,
    
    -- ** Elementary functions
    absOut,
    sqrtOut, sqrtOutWithIters,
    expOut,expOutWithDegree,
    sinOut, sinOutWithDegree, cosOut, cosOutWithDegree,
    
)
where

import Prelude hiding ((<*>))

import Numeric.AERN.Basics.Interval
    (Interval)

import Numeric.AERN.NumericOrder 
    hiding (least,greatest)
import qualified Numeric.AERN.NumericOrder as BNO 
    (least,greatest)

import Numeric.AERN.RefinementOrder


import Numeric.AERN.RealArithmetic.RefinementOrderRounding
    hiding (piOut,eOut)
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as RAROR
    (piOut,eOut)

import Numeric.AERN.RealArithmetic.Interval
    (SqrtThinEffortIndicator(..),
     ExpThinEffortIndicator(..), 
     SineCosineThinEffortIndicator(..))

import Numeric.AERN.RealArithmetic.Basis.Double()

-- | 
-- Intervals with Double endpoints, presented as an abstract
-- data type for approximating real numbers.  One interval
-- represents a single real number contained in it.  Only
-- operations supporting this view are provided by this module. 
type RealApprox = Interval Double

sampleRealApprox :: RealApprox
sampleRealApprox = 0

least :: RealApprox
least = BNO.least sampleRealApprox

greatest :: RealApprox
greatest = BNO.greatest sampleRealApprox

-- | Outward rounded pi
piOut :: RealApprox
piOut = RAROR.piOut sampleRealApprox

-- | Outward rounded e
eOut :: RealApprox
eOut = RAROR.eOut sampleRealApprox

--newtype PositiveRealApprox = 
--    PositiveRealApprox { unPositiveRealApprox :: RealApprox }
--
--instance Show PositiveRealApprox where
--    show (PositiveRealApprox i) = show i
--
--instance Arbitrary PositiveRealApprox
--    where
--    arbitrary =
--        do
--        NumOrd.UniformlyOrderedPair (l,h) <- arbitrary
--        return $ PositiveRealApprox (Interval (pos l) (pos h))
--        where
--        pos e 
--            | e > 0 =  e
--            | e == 0 =  1
--            | otherwise = (-e) 

-- | Outwards rounded square root with convenient effort regulation
sqrtOutWithIters :: Int -> RealApprox -> RealApprox
sqrtOutWithIters iters x =
    sqrtOutEff eff x
    where
    eff = 
        (sqrtDefaultEffort x)
        {
            sqrteff_newtonIters = iters
        }

-- | Inwards rounded square root with convenient effort regulation
sqrtInWithDegree :: Int -> RealApprox -> RealApprox
sqrtInWithDegree iters x =
    sqrtInEff eff x
    where
    eff = 
        (sqrtDefaultEffort x)
        {
            sqrteff_newtonIters = iters
        }

-- | Outwards rounded exponential with convenient effort regulation
expOutWithDegree :: Int -> RealApprox -> RealApprox
expOutWithDegree degree x =
    expOutEff eff x
    where
    eff = 
        (expDefaultEffort x)
        {
            expeff_taylorDeg = degree
        }

-- | Inwards rounded exponential with convenient effort regulation
expInWithDegree :: Int -> RealApprox -> RealApprox
expInWithDegree degree x =
    expInEff eff x
    where
    eff = 
        (expDefaultEffort x)
        {
            expeff_taylorDeg = degree
        }

-- | Outwards rounded sine with convenient effort regulation
sinOutWithDegree :: Int -> RealApprox -> RealApprox
sinOutWithDegree degree x =
    sinOutEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Outwards rounded cosine with convenient effort regulation
cosOutWithDegree :: Int -> RealApprox -> RealApprox
cosOutWithDegree degree x =
    cosOutEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Inwards rounded sine with convenient effort regulation
sinInWithDegree :: Int -> RealApprox -> RealApprox
sinInWithDegree degree x =
    sinInEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Inwards rounded cosine with convenient effort regulation
cosInWithDegree :: Int -> RealApprox -> RealApprox
cosInWithDegree degree x =
    cosInEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

sincosDefaultEffortWithDegree x degree =
    (sincosDefaultEffort x)
    {
        sincoseff_taylorDeg = degree
    }

        
