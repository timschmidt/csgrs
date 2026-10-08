{-|
    Module      :  Numeric.AERN.RealArithmetic.ExactOps
    Description :  access to exact zero and one  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Access to exact zero and one.
-}
module Numeric.AERN.RealArithmetic.ExactOps where

import Prelude hiding (EQ, LT, GT)
import Numeric.AERN.Basics.PartialOrdering

import Control.Monad.ST
import Data.STRef

import qualified Numeric.AERN.NumericOrder as NumOrd

import Data.Ratio

class HasSampleFromContext t where
    sampleFromContext :: t

class HasZero t where
    zero :: t -> t
    
pNonnegNonposEff effort a =
    (nonneg, nonpos)
    where
    (_, nonneg, _, nonpos) =
        pPosNonnegNegNonposEff effort a
    
pPosNonnegNegNonposEff effort a =
    (infoGT, infoGEQ, infoLT, infoLEQ) 
    where
    info = NumOrd.pCompareInFullEff effort a (zero a)
    infoLT = pOrdInfLT info
    infoGT = pOrdInfGT info
    infoLEQ = pOrdInfLEQ info
    infoGEQ = pOrdInfGEQ info
        
class HasOne t where
    one :: t -> t
    
class HasInfinities t where
    plusInfinity :: t -> t
    minusInfinity :: t -> t
    excludesPlusInfinity :: t -> Bool
    excludesMinusInfinity :: t -> Bool
    excludesInfinity :: t -> Bool
    excludesInfinity a = 
        excludesMinusInfinity a && excludesPlusInfinity a 
    
class Neg t where
    neg :: t -> t
    
propNegFlip ::
    (Eq t, Neg t) =>
    t -> t -> Bool
propNegFlip _ e =
    neg (neg e) == e 

-- instances for some common types:

instance HasSampleFromContext Int where sampleFromContext = 0
instance HasZero Int where zero _ = 0
instance HasOne Int where one _ = 1
instance Neg Int where neg = negate

instance HasSampleFromContext Integer where sampleFromContext = 0
instance HasZero Integer where zero _ = 0
instance HasOne Integer where one _ = 1
instance Neg Integer where neg = negate

instance (HasSampleFromContext t, HasOne t, Integral t) => 
    HasSampleFromContext (Ratio t) 
    where 
    sampleFromContext = sampleN % (one sampleN)
        where
        sampleN = sampleFromContext
instance (HasZero t, HasOne t, Integral t) => 
    HasZero (Ratio t) 
    where 
    zero sample = (zero sampleN) % (one sampleN)
        where
        sampleN = numerator sample
instance (HasOne t, Integral t) => 
    HasOne (Ratio t) 
    where 
    one sample = (one sampleN) % (one sampleN)
        where
        sampleN = numerator sample
instance (Integral t) => Neg (Ratio t) where neg = negate

instance HasSampleFromContext Double where sampleFromContext = 0
instance HasZero Double where zero _ = 0
instance HasOne Double where one _ = 1
instance Neg Double where neg = negate

instance HasInfinities Double where
    plusInfinity _ = 1/0
    minusInfinity _ = -1/0
    excludesPlusInfinity a = (a /= (plusInfinity a))
    excludesMinusInfinity a = (a /= (minusInfinity a))
    
instance HasSampleFromContext Char 
    where 
    sampleFromContext = 'S'
instance HasSampleFromContext t => HasSampleFromContext [t] 
    where 
    sampleFromContext = [sampleFromContext]
