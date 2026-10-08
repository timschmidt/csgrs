{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.FieldOps
    Description :  rounded arithmetic instances for Double
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded arithmetic instances for Double.
    
    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.Double.FieldOps 
(setMachineRoundingModeUp)
where

import Numeric.AERN.RealArithmetic.Basis.Double.NumericOrder

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Effort

import Numeric.IEEE.RoundMode
import System.IO.Unsafe

import Numeric.AERN.Basics.Exception
import Control.Exception

withUpwardsRounding :: a -> a
withUpwardsRounding a =
    unsafePerformIO $ 
        do 
        setMachineRoundingModeUp
        aa <- return $! a
        return aa

setMachineRoundingModeUp :: IO ()
setMachineRoundingModeUp =
    do
    currentRndMode <- getRound
    case currentRndMode == Upward of
        True ->
            do
--            putStrLn "setMachineRoundingModeUp: already up"
            return ()
        False ->
            do
            success <- setRound Upward
            case success of
                True ->
                    do 
--                    putStrLn $ "setMachineRoundingModeUp: switching up from " ++ show currentRndMode
                    return ()
                False -> 
                    error "Numeric.AERN.RealArithmetic.Basics.Double: failed to switch rounding mode"

detectNaNThrow :: String -> Double -> Double
detectNaNThrow msg a 
    | isNaN a =
        throw (AERNDomViolationException $ "domain violation in " ++ msg)
    | otherwise = a

detectNaNUp :: String -> Double -> Double
detectNaNUp _ a 
    | isNaN a = 1/0
    | otherwise = a

detectNaNDn :: String -> Double -> Double
detectNaNDn _ a 
    | isNaN a = -1/0
    | otherwise = a

instance RoundedAddEffort Double where
    type AddEffortIndicator Double = () 
    addDefaultEffort _ = ()

instance RoundedAdd Double where
    addUpEff effort d1 d2 = 
        detectNaNUp ("addition " ++ show d1 ++ " +^ " ++ show d2 ) $ 
            withUpwardsRounding $ d1 + d2
    addDnEff effort d1 d2 =
        detectNaNDn ("addition " ++ show d1 ++ " +. " ++ show d2 ) $ 
            negate $ withUpwardsRounding $ (negate d1) + (negate d2)
    -- the following is an exagerated rounding version meant for testing:
--    type AddEffortIndicator Double = Int1To100 
--    addDefaultEffort _ = Int1To100 10
--    addUpEff effort d1 d2 = 
--        withUpwardsRounding $ 
--            d1 + d2 + (1/effortD)
--        where
--        effortD = fromInteger $ toInteger $ fromInt1To100 $ effort
--    addDnEff effort d1 d2 = 
--        negate $ withUpwardsRounding $ 
--            (negate d1) + (negate d2) + (1/effortD)
--        where
--        effortD = fromInteger $ toInteger $ fromInt1To100 $ effort

instance RoundedSubtr Double

instance RoundedAbsEffort Double where
    type AbsEffortIndicator Double = ()
    absDefaultEffort _ = ()

instance RoundedAbs Double where
    absDnEff _ = abs
    absUpEff _ = abs

instance RoundedMultiplyEffort Double where
    type MultEffortIndicator Double = () 
    multDefaultEffort _ = ()

instance RoundedMultiply Double where
    multUpEff effort d1 d2 = 
        detectNaNUp ("multiplication " ++ show d1 ++ " *^ " ++ show d2 ) $ 
            withUpwardsRounding $ d1 * d2
    multDnEff effort d1 d2 = 
        detectNaNDn ("multiplication " ++ show d1 ++ " *. " ++ show d2 ) $ 
            negate $ withUpwardsRounding $ (negate d1) * d2
    -- the following is an exagerated rounding version meant for testing:
--    type MultEffortIndicator Double = Int1To100 
--    multDefaultEffort _ = Int1To100 10
--    multUpEff effort d1 d2 = 
--        withUpwardsRounding $ 
--            d1 * d2 + (1/effortD)
--        where
--        effortD = fromInteger $ toInteger $ fromInt1To100 $ effort
--    multDnEff effort d1 d2 = 
--        negate $ withUpwardsRounding $ 
--            (negate d1) * d2 + (1/effortD)
--        where
--        effortD = fromInteger $ toInteger $ fromInt1To100 $ effort

instance RoundedPowerNonnegToNonnegIntEffort Double where
    type PowerNonnegToNonnegIntEffortIndicator Double = 
        PowerNonnegToNonnegIntEffortIndicatorFromMult Double
    powerNonnegToNonnegIntDefaultEffort = 
        powerNonnegToNonnegIntDefaultEffortFromMult

instance RoundedPowerNonnegToNonnegInt Double where
    powerNonnegToNonnegIntUpEff = 
        powerNonnegToNonnegIntUpEffFromMult
    powerNonnegToNonnegIntDnEff = 
        powerNonnegToNonnegIntDnEffFromMult

instance RoundedPowerToNonnegIntEffort Double where
    type PowerToNonnegIntEffortIndicator Double = 
        PowerToNonnegIntEffortIndicatorFromMult Double
    powerToNonnegIntDefaultEffort = 
        powerToNonnegIntDefaultEffortFromMult

instance RoundedPowerToNonnegInt Double where
    powerToNonnegIntUpEff = 
        powerToNonnegIntUpEffFromMult
    powerToNonnegIntDnEff = 
        powerToNonnegIntDnEffFromMult

instance RoundedDivideEffort Double where
    type DivEffortIndicator Double = () 
    divDefaultEffort _ = ()

instance RoundedDivide Double where
    divUpEff effort d1 d2 = 
        detectNaNUp ("division " ++ show d1 ++ " /^ " ++ show d2 ) $ 
            withUpwardsRounding $ d1 / d2
    divDnEff effort d1 d2 = 
        detectNaNDn ("division " ++ show d1 ++ " /. " ++ show d2 ) $ 
            negate $ withUpwardsRounding $ (negate d1) / d2

instance RoundedRingEffort Double
    where
    type RingOpsEffortIndicator Double = ()
    ringOpsDefaultEffort _ = ()
    ringEffortAdd _ _ = ()
    ringEffortMult _ _ = ()
    ringEffortPow _ _ = ()

instance RoundedRing Double

instance RoundedFieldEffort Double
    where
    type FieldOpsEffortIndicator Double = ()
    fieldOpsDefaultEffort _ = ()
    fldEffortAdd _ _ = ()
    fldEffortMult _ _ = ()
    fldEffortPow _ _ = ()
    fldEffortDiv _ _ = ()

instance RoundedField Double

--test1 :: [Double]
--test1 =
--    let ?multUpDnEffort = () :: MultEffortIndicator Double in
--    let up = foldl1 (*^) [1/3,1/3,1/3]
--        dn = foldl1 (*.) [1/3,1/3,1/3]
--    in 
--    [up, dn]
    

    