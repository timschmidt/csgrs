{-# LANGUAGE TypeFamilies #-}
module Main where

import Control.Monad (unless)
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as N
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import qualified Numeric.AERN.NumericOrder as O
import qualified Numeric.AERN.Basics.PartialOrdering as P
import Numeric.AERN.RealArithmetic.ExactOps

-- An independent exact-rational specimen with deliberately coarser lattice
-- rounding than division. All operations used by the counterexample obey
-- their directed bounds; this is not an AERN MPFR/Double instance test.
data Scalar = NegInf | Finite Rational | PosInf deriving (Eq, Ord, Show)
instance HasZero Scalar where zero _ = Finite 0
instance HasOne Scalar where one _ = Finite 1
instance HasInfinities Scalar where
    plusInfinity _ = PosInf
    minusInfinity _ = NegInf
    excludesPlusInfinity x = x /= PosInf
    excludesMinusInfinity x = x /= NegInf
instance N.RoundedAddEffort Scalar where
    type AddEffortIndicator Scalar = Int
    addDefaultEffort _ = 13
instance N.RoundedMultiplyEffort Scalar where
    type MultEffortIndicator Scalar = Int
    multDefaultEffort _ = 29
instance N.RoundedDivideEffort Scalar where
    type DivEffortIndicator Scalar = Int
    divDefaultEffort _ = 47
instance R.RoundedMultiplyEffort Scalar where
    type MultEffortIndicator Scalar = Int
    multDefaultEffort _ = 29
instance R.RoundedDivideEffort Scalar where
    type DivEffortIndicator Scalar = Int
    divDefaultEffort _ = 47
instance O.RoundedLatticeEffort Scalar where
    type MinmaxEffortIndicator Scalar = ()
    minmaxDefaultEffort _ = ()
up, down :: Scalar -> Scalar
up (Finite x) = Finite (fromInteger (ceiling x))
up x = x
down (Finite x) = Finite (fromInteger (floor x))
down x = x
instance O.RoundedLattice Scalar where
    maxUpEff _ a b = up (max a b)
    maxDnEff _ a b = down (max a b)
    minUpEff _ a b = up (min a b)
    minDnEff _ a b = down (min a b)
instance O.PartialComparison Scalar where
    type PartialCompareEffortIndicator Scalar = ()
    pCompareDefaultEffort _ = ()
    pCompareEff _ a b = Just $ case compare a b of
        LT -> P.LT
        EQ -> P.EQ
        GT -> P.GT
divide :: Scalar -> Scalar -> Scalar
divide (Finite a) (Finite b) | b /= 0 = Finite (a/b)
divide _ _ = error "probe division outside finite nonzero domain"
instance N.RoundedDivide Scalar where
    divUpEff _ = divide
    divDnEff _ = divide

ensure :: String -> Bool -> IO ()
ensure name ok = unless ok (error name)
main :: IO ()
main = do
    let sample = Finite 1
        divisor = Finite 2
        (nm,_,_) = N.mixedMultDefaultEffortByConversion sample divisor
        (nd,_,_) = N.mixedDivDefaultEffortByConversion sample divisor
        (rm,_) = R.mixedMultDefaultEffortByConversion sample divisor
        (rd,_) = R.mixedDivDefaultEffortByConversion sample divisor
    ensure "numeric defaults route through add" (nm == 13 && nd == 13)
    ensure "refinement defaults select actual operations" (rm == 29 && rd == 47)
    putStrLn ("numeric mixed mult/div efforts: " ++ show (nm,nd))
    putStrLn ("refinement mixed mult/div efforts: " ++ show (rm,rd))
    let actual = N.mixedDivUpEffByConversion (47,(),((),())) sample divisor
        exact = Finite (1/2)
        correctedCombination = O.maxUpEff () exact exact
    ensure "upward mixed division falls below exact result" (actual == Finite 0 && actual < exact)
    ensure "upward combination would enclose" (correctedCombination >= exact)
    putStrLn ("upward mixed division 1/2: " ++ show actual ++ "; exact: " ++ show exact)
    putStrLn "generic directed-bound counterexample and default-effort dispatch reproduced"
