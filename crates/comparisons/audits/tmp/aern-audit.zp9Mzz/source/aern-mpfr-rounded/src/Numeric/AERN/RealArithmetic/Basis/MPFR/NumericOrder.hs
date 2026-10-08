{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE RankNTypes #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
    Description :  numeric order instances for MPFR  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Numeric order Comparison and lattice instances for MPFR.
    
    This is a private module reexported publicly via its parent.
-}
module Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
where

import Prelude hiding (EQ,LT,GT)

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import qualified Numeric.Rounded as R

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Exception
import Control.Exception

import Numeric.AERN.Basics.PartialOrdering
import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder 
        (PartialCompareEffortIndicator,
         MinmaxEffortIndicator)

import Numeric.AERN.Basics.SizeLimits

import Test.QuickCheck
import Numeric.AERN.Misc.QuickCheck

import Numeric.AERN.Misc.List
import Numeric.AERN.Misc.Debug

import Data.Maybe

import System.IO.Unsafe

sampleM :: MPFR
sampleM = 0

nanM :: MPFR
nanM = 0/0

instance HasSizeLimits MPFR where
    type SizeLimits MPFR = MPFRPrec
    getSizeLimits a = getPrecision a
    defaultSizeLimits _a = defaultPrecision
       
instance CanChangeSizeLimits MPFR where
    type SizeLimitsChangeEffort MPFR = ()
    sizeLimitsChangeDefaultEffort _ = ()
--    changeSizeLimitsDnEff _ prec a = M.set M.Down prec a
--    changeSizeLimitsUpEff _ prec a = M.set M.Up prec a
    changeSizeLimitsDnEff = error $ "AERN: changeSizeLimitsDnEff not defined for MPFR"
    changeSizeLimitsUpEff = error $ "AERN: changeSizeLimitsUpEff not defined for MPFR"
    changeSizeLimitsOutEff = error $ "AERN: changeSizeLimitsOutEff not defined for MPFR"
    changeSizeLimitsInEff = error $ "AERN: changeSizeLimitsInEff not defined for MPFR"

instance NumOrd.HasLeast MPFR where
    least sample = withPrec (getPrecision sample) (- 1/0)

instance NumOrd.HasGreatest MPFR where
    greatest sample = withPrec (getPrecision sample) (1/0)

instance NumOrd.HasExtrema MPFR where

instance NumOrd.PartialComparison MPFR where
    type PartialCompareEffortIndicator MPFR = ()
    pCompareEff _ a b = Just $ toPartialOrdering $ Prelude.compare a b
--        case (M.isNaN a, M.isNaN b) of
--           (False, False) -> Just $ toPartialOrdering $ Prelude.compare a b  
--           (True, True) -> Just EQ
--           _ -> Just NC 
    pCompareDefaultEffort _ = ()

instance NumOrd.RoundedLatticeEffort MPFR where
    type MinmaxEffortIndicator MPFR = ()
    minmaxDefaultEffort _ = ()

instance NumOrd.RoundedLattice MPFR where
    maxUpEff _ a b = Prelude.max a b
--        case (M.isNaN a, M.isNaN b) of
--           (False, False) -> Prelude.max a b  
--           _ -> throw (AERNException $ "illegal MPFR argument: NumOrd.Lattice.max " 
--                        ++ show a ++ " " ++ show b)
    maxDnEff = NumOrd.maxUpEff 
    minUpEff _ a b = Prelude.min a b
--        case (M.isNaN a, M.isNaN b) of
--           (False, False) -> Prelude.min a b  
--           _ -> throw (AERNException $ "illegal MPFR argument: NumOrd.Lattice.min " 
--                        ++ show a ++ " " ++ show b) 
    minDnEff = NumOrd.minUpEff 
    
--    -- a version with artificially added rounding for "testing" the tests
--    maxUpEff [effort] e1 e2 = NumOrd.max e1 e2 + (1/(convert effort))
--    maxDnEff [effort] e1 e2 = NumOrd.max e1 e2 - (1/(convert effort))
--    minUpEff [effort] e1 e2 = NumOrd.min e1 e2 + (1/(convert effort))
--    minDnEff [effort] e1 e2 = NumOrd.min e1 e2 - (1/(convert effort))
--    minmaxDefaultEffort _ = [10]


instance Arbitrary MPFR where
    arbitrary =
        sized $ \size ->
        do
        let sizeI = toInteger size
        precisionI <- choose (50,50+2*sizeI)
        significandI <- arbitrary
        expI <- choose (-10,10) 
           -- it is not essential to test with massive exponents 
           -- except when we test overflows - can set the exponent
           -- high artificially using M.setExp
        let result = constructMPFR precisionI significandI (expI :: Int)
        return result
        where
        constructMPFR precisionI significandI expI =
            withPrec (fromInteger precisionI) $ ((fromInteger significand) * 2^^expI)
            where
            significand
                | significandI == 0 = 1 
                | otherwise = fromInteger significandI 
--            precision = fromInteger precisionI 
--            exp = fromInteger expI 

instance ArbitraryWithArea MPFR where
    type (Area MPFR) = NumOrd.AreaLinear MPFR
    areaWhole _ = NumOrd.areaLinearWhole [-1/0,-1,0,1,1/0]
    arbitraryInArea =
        NumOrd.arbitraryLinear (-1/0, 1/0) id id chooseNearerZero
        where
        chooseNearerZero (lo, hi) =
--                unsafePrint ("chooseNearerZero: "
--                    ++ "\n loT = " ++ show loT
--                    ++ "\n hiT = " ++ show hiT
--                ) $
            do
            eT <- chooseMPFR (loT, hiT)
            return $ transform eT 
            where
            loT = transformInv lo
            hiT = transformInv hi
            transform :: MPFR -> MPFR
            transform x = x*x*x/1000000 -- 100^3 = 1000000
            transformInv x 
                | x > 0 = 100 * exp ((log x)/3)
                | x < 0 = - (transformInv (-x))
                | otherwise = 0
        chooseMPFR (lb, ub)
            | lbBounded && ubBounded =
                do
                x <- arbitrary
                let xUniform = liftRoundedToMPFR1 (\ v -> snd $ properFraction $ abs v) x
                return $ lb + ((xUniform)*(ub-lb))
            | lbBounded =
                do
                x <- arbitrary
                return $ lb + (abs x :: MPFR) 
            | ubBounded =
                do
                x <- arbitrary
                return $ ub - (abs x :: MPFR)
            | otherwise = arbitrary 
            where
            lbBounded = -1/0 < lb
            ubBounded = ub < 1/0 

instance (NumOrd.AreaHasBoundsConstraints MPFR)
    where
    areaSetLowerBound = NumOrd.areaLinearAddLowerBound
    areaSetUpperBound = NumOrd.areaLinearAddUpperBound


instance NumOrd.ArbitraryOrderedTuple MPFR where
    arbitraryTupleInAreaRelatedBy area = 
        NumOrd.linearArbitraryTupleRelatedBy 
                (arbitraryInArea area)
    arbitraryTupleRelatedBy =
        NumOrd.arbitraryTupleInAreaRelatedBy (areaWhole (0::MPFR))
    
instance (AreaHasNonNegativeOption MPFR)
    where
    areaRestrictToNonNeg _ =
        NumOrd.areaLinearRestrictToNonNeg 0

instance (AreaHasForbiddenValues MPFR)
    where
    areaGetForbiddenValues = areaWholeForbiddenValues . NumOrd.areaLinWhole
    areaAddForbiddenValues = NumOrd.areaLinearAddForbiddenValues

    
