{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.DoubleBasis.RealIntervalApprox
    Description :  Double intervals for approximating real intervals
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Intervals with Double endpoints as an
    abstract data type for approximating real number intervals.
    Each interval is either an outer or inner approximation
    of another interval. 
    Only operations that respect this view are available
    via this module.
-}
module Numeric.AERN.DoubleBasis.RealIntervalApprox
(
    -- |
    -- A convenience module re-exporting various interval operations 
    -- with default effort indicators.

    -- * Main type
    RealIntervalApprox,
    
    -- ** associated operations
    width, bisect,

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
    -- The consistent intervals in 'RealIntevalApprox' form a /meet/-semilattice
    -- corresponding to the refiniement order under the operation /\\ 
    -- returning the subset-least interval containing the /union/ of 
    -- its argument intervals. The operation is extended to all of 'RealIntevalApprox'
    -- by returning the highest interval below both of its argument 
    -- intervals.  
    -- 
    -- The structure ({ 'di' | 'di' is consistent \}, /\\, 
    -- 'bottom') is a complete meet-semilattice.
    -- 
    -- Lower and upper approximations of the exact operation /\\ 
    -- are given by '</\>' and '>/\<' respectively.
    --
    -- The dual operation to /\\ is partial on consistent intervals, 
    -- since the /intersection/ of disjoint sets is empty. Therefore,
    -- the /join/-semilattice structure on 'RealIntevalApprox' comes in two flavours:
    --
    --   * the partial consistent interval-valued join \\/\? which 
    --     returns 'Nothing' for disjoint and anticonsistent arguments
    --     and
    --
    --   * the total join \\/ which returns the lowest interval in
    --     'RealIntevalApprox' above both of its argument intervals. 
    -- 
    -- The structure ('RealIntevalApprox', \/\\, \\\/, 'bottom', 'top') is a complete 
    -- lattice.
    --
    -- Lower and upper approximations of the exact operations \\/\?
    -- and \\\/ are given by '<\/>?', '<\/>' and '>\/<' respectively.

    -- ** Numerical order
    -- | 
    -- Interval extensions of the corresponding tests and relations on 
    -- Double.
    
    -- *** Extrema
    -- |
    -- The values retured by 'least' and 'greatest' complete the 
    -- numerical partial order on 'RealIntevalApprox'.
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
    -- refinement partial order on 'RealIntevalApprox'.

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
    -- and 'Floating' instances for 'RealIntevalApprox' use such versions as instance 
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
    (</\>),(<\/>),(<\/>?),

    -- **** Unicode versions
    (<⊓>),(<⊔>),(<⊔>?),

    -- ** Field operations

    -- *** Interval operations
    (<+>),(<->),(<*>),(</>),

    -- *** Mixed type operations
    (|<+>),(<+>|),(|<*>),(<*>|),(</>|),(<^>),

    -- ** Special constants 
    piOut,eOut,
    
    -- ** Elementary functions
    absOut,
    sqrtOut, sqrtOutWithIters,
    expOut,expOutWithDegree,
    sinOut, sinOutWithDegree, cosOut, cosOutWithDegree,

    -- * Inward rounded operations 

    -- ** Order operations

    -- *** Numerical order
    -- | 
    -- Inward rounded interval extensions of the corresponding 
    -- operations on Double.
    minIn,maxIn,

    -- *** Refinement order

    -- **** ASCII versions
    (>/\<),(>\/<),

    -- **** Unicode versions
    (>⊓<),(>⊔<),

    -- ** Field operations
    
    -- *** Interval operations
    (>+<),(>-<),(>*<),(>/<),
    
    -- *** Mixed type operations
    (|>+<),(>+<|),(|>*<),(>*<|),(>/<|),(>^<),

    -- ** Special constants 
    piIn,eIn,
    
    -- ** Elementary functions
    absIn,
    sqrtIn, sqrtInWithIters,
    expIn,expInWithDegree,
    sinIn, sinInWithDegree, cosIn, cosInWithDegree,
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
    hiding (bottom,top,(⊥),(⊤))
import qualified Numeric.AERN.RefinementOrder as BRO
    (bottom,top,(⊥),(⊤))

import Numeric.AERN.RealArithmetic.Interval
    (SqrtThinEffortIndicator(..),
     ExpThinEffortIndicator(..), 
     SineCosineThinEffortIndicator(..))

import Numeric.AERN.RealArithmetic.RefinementOrderRounding 
  hiding (piOut,piIn,eOut,eIn)
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as RAROR 
  (piOut,piIn,eOut,eIn)
 
import Numeric.AERN.RealArithmetic.Basis.Double()

import Numeric.AERN.RealArithmetic.Interval.Double (width, bisect)

-- | 
-- Intervals with Double endpoints. 
-- 
-- Note that ('l','r') = 'getEndpoints' ('di' :: 'RealIntervalApprox') does not 
-- fix an ordering of 'l' and 'r'. 
-- 
--   * 'di' is called /consistent/ when 'l' '<=' 'r'
--
--   * 'di' is called /anticonsistent/ when 'r' '<=' 'l' 
--
-- A consistent interval 'di' may be identified with the set defined by
-- \{ 'x' | 'l' '<=' 'x' and 'x' '<=' 'r' \}.
type RealIntervalApprox = Interval Double

sampleRealIntervalApprox :: RealIntervalApprox
sampleRealIntervalApprox = 0

least :: RealIntervalApprox
least = BNO.least sampleRealIntervalApprox

greatest :: RealIntervalApprox
greatest = BNO.greatest sampleRealIntervalApprox

bottom :: RealIntervalApprox
bottom = BRO.bottom sampleRealIntervalApprox

top :: RealIntervalApprox
top = BRO.top sampleRealIntervalApprox

-- | Convenience Unicode notation for 'bottom'
(⊥) :: RealIntervalApprox
(⊥) = (BRO.⊥) sampleRealIntervalApprox

-- | Convenience Unicode notation for 'top'
(⊤) :: RealIntervalApprox
(⊤) = (BRO.⊤) sampleRealIntervalApprox

-- | Outward rounded pi
piOut :: RealIntervalApprox
piOut = RAROR.piOut sampleRealIntervalApprox

-- | Outward rounded e
eOut :: RealIntervalApprox
eOut = RAROR.eOut sampleRealIntervalApprox

-- | Inward rounded pi
piIn :: RealIntervalApprox
piIn = RAROR.piIn sampleRealIntervalApprox

-- | Inward rounded e
eIn :: RealIntervalApprox
eIn = RAROR.eIn sampleRealIntervalApprox

-- | Outwards rounded square root with convenient effort regulation
sqrtOutWithIters :: Int -> RealIntervalApprox -> RealIntervalApprox
sqrtOutWithIters iters x =
    sqrtOutEff eff x
    where
    eff = 
        (sqrtDefaultEffort x)
        {
            sqrteff_newtonIters = iters
        }

-- | Inwards rounded square root with convenient effort regulation
sqrtInWithIters :: Int -> RealIntervalApprox -> RealIntervalApprox
sqrtInWithIters iters x =
    sqrtInEff eff x
    where
    eff = 
        (sqrtDefaultEffort x)
        {
            sqrteff_newtonIters = iters
        }

-- | Outwards rounded exponential with convenient effort regulation
expOutWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
expOutWithDegree degree x =
    expOutEff eff x
    where
    eff = 
        (expDefaultEffort x)
        {
            expeff_taylorDeg = degree
        }

-- | Inwards rounded exponential with convenient effort regulation
expInWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
expInWithDegree degree x =
    expInEff eff x
    where
    eff = 
        (expDefaultEffort x)
        {
            expeff_taylorDeg = degree
        }

-- | Outwards rounded sine with convenient effort regulation
sinOutWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
sinOutWithDegree degree x =
    sinOutEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Outwards rounded cosine with convenient effort regulation
cosOutWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
cosOutWithDegree degree x =
    cosOutEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Inwards rounded sine with convenient effort regulation
sinInWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
sinInWithDegree degree x =
    sinInEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

-- | Inwards rounded cosine with convenient effort regulation
cosInWithDegree :: Int -> RealIntervalApprox -> RealIntervalApprox
cosInWithDegree degree x =
    cosInEff eff x
    where
    eff = sincosDefaultEffortWithDegree x degree

sincosDefaultEffortWithDegree x degree =
    (sincosDefaultEffort x)
    {
        sincoseff_taylorDeg = degree
    }

        
