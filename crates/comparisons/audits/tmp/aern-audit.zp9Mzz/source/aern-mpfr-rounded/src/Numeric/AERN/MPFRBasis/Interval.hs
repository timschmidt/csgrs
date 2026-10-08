{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.MPFRBasis.Interval
    Description :  Interval MPFR type and operations  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Intervals with MPFR endpoints.
-}
module Numeric.AERN.MPFRBasis.Interval
(
    -- |
    -- A convenience module re-exporting various interval operations 
    -- with default effort indicators.

    -- * Main type
    MI,
    
    -- ** associated operations
    width, bisect,
    fromIntWithPrec, fromIntegerWithPrec, fromRationalWithPrec,
    fromDoubleWithPrec, doubleBounds,

    -- * Order relations
    -- | 
    -- There are two types of order relations to consider: 
    -- 
    --   * the /numerical/ order, generalising the order relation
    --     on MPFRs and
    -- 
    --   * the /refinement/ order, generalising the reverse-inclusion 
    --     relation on consistent intervals.  
    --
    -- The consistent intervals in 'MI' form a /meet/-semilattice
    -- corresponding to the refiniement order under the operation /\\ 
    -- returning the subset-least interval containing the /union/ of 
    -- its argument intervals. The operation is extended to all of 'MI'
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
    -- the /join/-semilattice structure on 'MI' comes in two flavours:
    --
    --   * the partial consistent interval-valued join \\/\? which 
    --     returns 'Nothing' for disjoint and anticonsistent arguments
    --     and
    --
    --   * the total join \\/ which returns the lowest interval in
    --     'MI' above both of its argument intervals. 
    -- 
    -- The structure ('MI', \/\\, \\\/, 'bottom', 'top') is a complete 
    -- lattice.
    --
    -- Lower and upper approximations of the exact operations \\/\?
    -- and \\\/ are given by '<\/>?', '<\/>' and '>\/<' respectively.

    -- ** Numerical order
    -- | 
    -- Interval extensions of the corresponding tests and relations on 
    -- MPFR.
    
    -- *** Extrema
    -- |
    -- The values retured by 'least' and 'greatest' complete the 
    -- numerical partial order on 'MI'.
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
    -- refinement partial order on 'MI'.

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
    -- and 'Floating' instances for 'MI' use such versions as instance 
    -- methods.
    
    -- ** Order operations
    
    -- *** Numerical order
    -- | 
    -- Outward rounded interval extensions of the corresponding 
    -- operations on MPFR.
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
    expOut,
    sqrtOut,

    -- * Inward rounded operations 

    -- ** Order operations

    -- *** Numerical order
    -- | 
    -- Inward rounded interval extensions of the corresponding 
    -- operations on MPFR.
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
    expIn,
    sqrtIn,

    -- * Low level facilities

    -- ** Access functions
    getEndpoints,fromEndpoints,

    -- ** Precision inspection and control
    getPrec, setPrecOut, Precision,

    -- ** Base type
    Interval(..),
    MPFR
)
where

import Prelude hiding ((<*>))

import Numeric.AERN.Basics.Interval
  (Interval(..))

import qualified Numeric.AERN.Basics.Interval as BI
  (getEndpoints,fromEndpoints)

import qualified Numeric.AERN.NumericOrder as BNO
  (least,greatest,
   (==?),(<==>?),(</=>?),
   (<?),(>?),(<=?),(>=?),
   minOut,maxOut,minIn,maxIn,
   UniformlyOrderedPair(..))

import qualified Numeric.AERN.RefinementOrder as BRO
  (bottom,top,(⊥),(⊤),
   (|==?),(|<==>?),(|</=>?),
   (|<?),(|>?),(|<=?),(|>=?),(⊏?),(⊑?),(⊒?),(⊐?),
   (</\>),(<\/>),(<\/>?),(<⊓>),(<⊔>),(<⊔>?),
   (>/\<),(>\/<),(>⊓<),(>⊔<))

import Numeric.AERN.RealArithmetic.Interval()

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as RAROR 
  (RoundedMixedAdd(..),RoundedMixedMultiply(..),RoundedMixedDivide(..),
   (<+>),(<->),(<*>),(</>),(|<+>),(<+>|),(|<*>),(<*>|),(</>|),(<^>),
   piOut,eOut,absOut,expOut,sqrtOut,
   (>+<),(>-<),(>*<),(>/<),(|>+<),(>+<|),(|>*<),(>*<|),(>/<|),(>^<),
   piIn,eIn,absIn,expIn,sqrtIn
  )

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as RANOR 
   (convertUpEff, convertDnEff)
   
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis
    (asIntervalElementaryFromBasis)

import Numeric.AERN.RealArithmetic.Basis.MPFR(MPFR, MPFRPrec, getPrecision, withPrec, defaultPrecision)

import Numeric.AERN.RealArithmetic.Interval.MPFR(width, bisect)

import Test.QuickCheck

infix 4 ==?, <==>?, </=>?, <?, <=?, >=?, >?

infix 4 |==?, |<==>?, |</=>?, |<?, |<=?, |>=?, |>?, ⊏?, ⊑?, ⊒?, ⊐?
infixr 3 </\>, >/\<, <⊓>, >⊓< 
infixr 2 <\/>?, <\/>, >\/<, <⊔>?, <⊔>, >⊔< 

infixl 6 <+>, >+<, <->, >-<
infixl 7 <*>, >*<
infixl 8 <^>, >^<
infixl 7 </>, >/<

infixr 6 |<+>, |>+<
infixl 6 <+>|, >+<|
infixr 7 |<*>, |>*<
infixl 7 <*>|, >*<|
infixl 7 </>|, >/<|

-- | 
-- Intervals with MPFR endpoints. 
-- 
-- Note that ('l','r') = 'getEndpoints' ('mi' :: 'MI') does not 
-- fix an ordering of 'l' and 'r'. 
-- 
--   * 'mi' is called /consistent/ when 'l' '<=' 'r'
--
--   * 'mi' is called /anticonsistent/ when 'r' '<=' 'l' 
--
-- A consistent interval 'di' may be identified with the set defined by
-- \{ 'x' | 'l' '<=' 'x' and 'x' '<=' 'r' \}.
type MI = Interval MPFR

type Precision = MPFRPrec

-- | Given an argument interval 'i' 'getEndpoints' returns the endpoint pair 
--   ('leftEndpoint' 'i','rightEndpoint' 'i').
getEndpoints :: MI -> (MI, MI)
getEndpoints i = (Interval l l, Interval r r)
    where
    (l,r) = BI.getEndpoints i

-- | Constructs an interval from an endpoint pair.
fromEndpoints :: (MI, MI) -> MI
fromEndpoints (Interval l _, Interval _ r) = 
    BI.fromEndpoints (l,r)

sampleMIWithPrec :: Precision -> MI
sampleMIWithPrec prec = Interval z z
    where
    z = withPrec prec 0 

sampleMI = sampleMIWithPrec defaultPrecision

getPrec :: MI -> Precision
getPrec x = getPrecision l 
    where
    (l,r) = BI.getEndpoints x

setPrecOut :: Precision -> MI -> MI
--setPrecOut = changeSizeLimitsOut -- TODO, using
setPrecOut prec x =
    BI.fromEndpoints (lP, rP)
    where
    lP = withPrec prec (fromRational $ toRational l) 
    rP = withPrec prec (fromRational $ toRational r) 
    (l,r) = BI.getEndpoints x

fromIntWithPrec :: Precision -> Int -> MI
fromIntWithPrec prec ii = fromEndpoints (l,r)
    where
    Just l = RANOR.convertDnEff () (sampleMIWithPrec prec) ii
    Just r = RANOR.convertUpEff () (sampleMIWithPrec prec) ii

fromIntegerWithPrec :: Precision -> Integer -> MI
fromIntegerWithPrec prec ii = fromEndpoints (l,r)
    where
    Just l = RANOR.convertDnEff () (sampleMIWithPrec prec) ii
    Just r = RANOR.convertUpEff () (sampleMIWithPrec prec) ii

fromRationalWithPrec :: Precision -> Rational -> MI
fromRationalWithPrec prec rr = fromEndpoints (l,r)
    where
    Just l = RANOR.convertDnEff () (sampleMIWithPrec prec) rr
    Just r = RANOR.convertUpEff () (sampleMIWithPrec prec) rr

fromDoubleWithPrec :: Precision -> Double -> MI
fromDoubleWithPrec prec dbl = fromEndpoints (l,r)
    where
    Just l = RANOR.convertDnEff () (sampleMIWithPrec prec) dbl
    Just r = RANOR.convertUpEff () (sampleMIWithPrec prec) dbl

doubleBounds :: MI -> (Double, Double)
doubleBounds (Interval l r) = (lD, rD)
    where
    Just lD = RANOR.convertDnEff () 0 l
    Just rD = RANOR.convertUpEff () 0 r

least :: MI
least = BNO.least sampleMI

greatest :: MI
greatest = BNO.greatest sampleMI

-- | Partial equality
(==?) :: MI -> MI -> Maybe Bool
(==?) = (BNO.==?) 

-- | Partial `is comparable to`
(<==>?) :: MI -> MI -> Maybe Bool
(<==>?) = (BNO.<==>?)

-- | Partial `is not comparable to`
(</=>?) :: MI -> MI -> Maybe Bool
(</=>?) = (BNO.</=>?)

-- | Partial `strictly less than`
(<?) :: MI -> MI -> Maybe Bool
(<?) = (BNO.<?)

-- | Partial `strictly greater than`
(>?) :: MI -> MI -> Maybe Bool
(>?) = (BNO.>?)

-- | Partial `less than or equal to`
(<=?) :: MI -> MI -> Maybe Bool
(<=?) = (BNO.<=?)

-- | Partial `greater than or equal to`
(>=?) :: MI -> MI -> Maybe Bool
(>=?) = (BNO.>=?)
 
-- | Outward rounded minimum
minOut :: MI -> MI -> MI
minOut = BNO.minOut

-- | Outward rounded maximum
maxOut :: MI -> MI -> MI
maxOut = BNO.maxOut

-- | Inward rounded minimum
minIn :: MI -> MI -> MI
minIn = BNO.minIn

-- | Inward rounded maximum
maxIn :: MI -> MI -> MI
maxIn = BNO.maxIn

bottom :: MI
bottom = BRO.bottom sampleMI

top :: MI
top = BRO.top sampleMI

-- | Convenience Unicode notation for 'bottom'
(⊥) :: MI
(⊥) = (BRO.⊥) sampleMI

-- | Convenience Unicode notation for 'top'
(⊤) :: MI
(⊤) = (BRO.⊤) sampleMI

-- | Partial equality
(|==?) :: MI -> MI -> Maybe Bool
(|==?) = (BRO.|==?)

-- | Partial `is comparable to`
(|<==>?) :: MI -> MI -> Maybe Bool
(|<==>?) = (BRO.|<==>?)

-- | Partial `is not comparable to`
(|</=>?) :: MI -> MI -> Maybe Bool
(|</=>?) = (BRO.|</=>?)

-- | Partial `strictly below`
(|<?) :: MI -> MI -> Maybe Bool
(|<?) = (BRO.|<?)

-- | Partial `strictly above`
(|>?) :: MI -> MI -> Maybe Bool
(|>?) = (BRO.|>?)

-- | Partial `below or equal to`
(|<=?) :: MI -> MI -> Maybe Bool
(|<=?) = (BRO.|<=?)

-- | Partial `above or equal to`
(|>=?) :: MI -> MI -> Maybe Bool
(|>=?) = (BRO.|>=?)

{-| Convenience Unicode notation for '|<?' -}
(⊏?) :: MI -> MI -> Maybe Bool
(⊏?) = (BRO.⊏?)

{-| Convenience Unicode notation for '|<=?' -}
(⊑?) :: MI -> MI -> Maybe Bool
(⊑?) = (BRO.⊑?)

{-| Convenience Unicode notation for '|>=?' -}
(⊒?) :: MI -> MI -> Maybe Bool
(⊒?) = (BRO.⊒?)

{-| Convenience Unicode notation for '|>?' -}
(⊐?) :: MI -> MI -> Maybe Bool 
(⊐?) = (BRO.⊐?)

-- | Outward rounded meet
(</\>) :: MI -> MI -> MI
(</\>) = (BRO.</\>)

-- | Outward rounded join
(<\/>) :: MI -> MI -> MI
(<\/>) = (BRO.<\/>)

-- | Inward rounded meet
(>/\<) :: MI -> MI -> MI
(>/\<) = (BRO.>/\<)

-- | Inward rounded join
(>\/<) :: MI -> MI -> MI
(>\/<) = (BRO.>\/<)

{-| Convenience Unicode notation for '</\>' -}
(<⊓>) :: MI -> MI -> MI
(<⊓>) = (BRO.<⊓>)

{-| Convenience Unicode notation for '<\/>' -}
(<⊔>) :: MI -> MI -> MI
(<⊔>) = (BRO.<⊔>)

{-| Convenience Unicode notation for '>/\<' -}
(>⊓<) :: MI -> MI -> MI
(>⊓<) = (BRO.>⊓<)

{-| Convenience Unicode notation for '>\/<' -}
(>⊔<) :: MI -> MI -> MI
(>⊔<) = (BRO.>⊔<)
 
-- | Partial outward rounded join
(<\/>?) :: MI -> MI -> Maybe MI
(<\/>?) = (BRO.<\/>?)

{-| Convenience Unicode notation for '<\/>?' -}
(<⊔>?) :: MI -> MI -> Maybe MI 
(<⊔>?) = (BRO.<⊔>?)

-- | Outward rounded addition
(<+>) :: MI -> MI -> MI
(<+>) = (RAROR.<+>)

-- | Outward rounded subtraction
(<->) :: MI -> MI -> MI
(<->) = (RAROR.<->)

-- | Outward rounded multiplication
(<*>) :: MI -> MI -> MI
(<*>) = (RAROR.<*>)

-- | Outward rounded division
(</>) :: MI -> MI -> MI
(</>) = (RAROR.</>)

-- | Inward rounded addition
(>+<) :: MI -> MI -> MI
(>+<) = (RAROR.>+<)

-- | Inward rounded subtraction
(>-<) :: MI -> MI -> MI
(>-<) = (RAROR.>-<)

-- | Inward rounded multiplication
(>*<) :: MI -> MI -> MI
(>*<) = (RAROR.>*<)

-- | Inward rounded division
(>/<) :: MI -> MI -> MI
(>/<) = (RAROR.>/<)

-- | Outward rounded additive scalar left action
(|<+>) :: RAROR.RoundedMixedAdd MI tn => tn -> MI -> MI
(|<+>) = (RAROR.|<+>)

-- | Inward rounded additive scalar left action
(|>+<) :: RAROR.RoundedMixedAdd MI tn => tn -> MI -> MI
(|>+<) = (RAROR.|>+<)

-- | Outward rounded additive scalar right action
(<+>|) :: RAROR.RoundedMixedAdd MI tn => MI -> tn -> MI
(<+>|) = (RAROR.<+>|)

-- | Inward rounded additive scalar right action
(>+<|) :: RAROR.RoundedMixedAdd MI tn => MI -> tn -> MI
(>+<|) = (RAROR.>+<|)

-- | Outward rounded multiplicative scalar left action
(|<*>) :: RAROR.RoundedMixedMultiply MI tn => tn -> MI -> MI
(|<*>) = (RAROR.|<*>)

-- | Inward rounded multiplicative scalar left action
(|>*<) :: RAROR.RoundedMixedMultiply MI tn => tn -> MI -> MI
(|>*<) = (RAROR.|>*<)

-- | Outward rounded multiplicative scalar right action
(<*>|) :: RAROR.RoundedMixedMultiply MI tn => MI -> tn -> MI
(<*>|) = (RAROR.<*>|)

-- | Inward rounded multiplicative scalar right action
(>*<|) :: RAROR.RoundedMixedMultiply MI tn => MI -> tn -> MI
(>*<|) = (RAROR.>*<|)

-- | Outward rounded multiplicative scalar reciprocal right action
(</>|) :: RAROR.RoundedMixedDivide MI tn => MI -> tn -> MI
(</>|) = (RAROR.</>|)

-- | Inward rounded multiplicative scalar reciprocal right action
(>/<|) :: RAROR.RoundedMixedDivide MI tn => MI -> tn -> MI  
(>/<|) = (RAROR.>/<|)

-- | Outward rounded power
(<^>) :: MI -> Int -> MI
(<^>) = (RAROR.<^>)

-- | Inward rounded power
(>^<) :: MI -> Int -> MI
(>^<) = (RAROR.>^<)

-- | Outward rounded pi with sample value parameter to determine precision from
piOut :: MI -> MI
piOut = RAROR.piOut

-- | Outward rounded e with sample value parameter to determine precision from
eOut :: MI -> MI
eOut = RAROR.eOut

-- | Inward rounded pi with sample value parameter to determine precision from
piIn :: MI -> MI
piIn = RAROR.piIn

-- | Inward rounded e with sample value parameter to determine precision from
eIn :: MI -> MI
eIn = RAROR.eIn

-- | Outward rounded absolute value
absOut :: MI -> MI
absOut = RAROR.absOut

-- | Outward rounded exponential
expOut :: MI -> MI
expOut = asIntervalElementaryFromBasis $ RAROR.expOut

-- | Outward rounded square root
sqrtOut :: MI -> MI
sqrtOut = asIntervalElementaryFromBasis $ RAROR.sqrtOut

-- | Inward rounded absolute value
absIn :: MI -> MI
absIn = RAROR.absIn

-- | Inward rounded exponential
expIn :: MI -> MI
expIn = asIntervalElementaryFromBasis $ RAROR.expIn

-- | Inward rounded square root
sqrtIn :: MI -> MI
sqrtIn = asIntervalElementaryFromBasis $ RAROR.sqrtIn

newtype PositiveMI = PositiveMI { unPositiveMI :: MI }

instance Show PositiveMI where
    show (PositiveMI i) = show i

instance Arbitrary PositiveMI
    where
    arbitrary =
        do
        BNO.UniformlyOrderedPair (l,h) <- arbitrary
        return $ PositiveMI (Interval (pos l) (pos h))
        where
        pos e 
            | e > 0 =  e
            | e == 0 =  1
            | otherwise = (-e) 
