{-# OPTIONS_GHC -fno-warn-incomplete-patterns #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Examples.Hybrid.Simple
    Description :  simple examples of hybrid system IVPs
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Simple examples of hybrid system IVPs.
-}

module Numeric.AERN.IVP.Examples.Hybrid.Simple where

import Prelude hiding ((<*>))

import Numeric.AERN.IVP.Specification.Hybrid

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding 
       as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (dblToReal)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps

import qualified 
       Numeric.AERN.RefinementOrder 
       as RefOrd
import Numeric.AERN.RefinementOrder.Operators

import qualified 
       Numeric.AERN.NumericOrder 
       as NumOrd
import Numeric.AERN.NumericOrder.Operators

import Numeric.AERN.Basics.Consistency

import qualified Data.Map as Map

import Debug.Trace
_ = trace -- stop the unused warning

ivpByNameMap ::
    (Var f ~ String,
     HasConstFns f,
     CanEvaluate f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    =>
    f ->
    Map.Map String (HybridIVP f)    
ivpByNameMap sampleFn =
    Map.fromList
    [
--        ("expDec-resetOnce", ivpExpDecay_resetTHalf sampleFn),
--        ("expDec-resetOn34", ivpExpDecay_resetOn34 sampleFn),
--        ("springMass-resetOnce", ivpSpringMass_resetTHalf sampleFn),
--        ("springMass-resetOn34", ivpSpringMass_resetOn34 sampleFn),
        ("pendulumDampened", ivpDampenedPendulum sampleFn),
        ("bouncingBall", ivpBouncingBall sampleFn),
        ("bouncingBallEnergy", ivpBouncingBallEnergy sampleFn),
        ("bouncingBallFloorRise", ivpBouncingBallFloorRise sampleFn),
        ("bouncingBallFloorRiseEnergy", ivpBouncingBallFloorRiseEnergy sampleFn),
        ("bouncingBallFloorDropEnergy", ivpBouncingBallFloorDropEnergy sampleFn),
        ("bouncingBallRiseFall", ivpBouncingBallRiseFall sampleFn),
        ("bouncingBallEnergyRiseFall", ivpBouncingBallRiseFallEnergy sampleFn),
        ("bouncingBallDrag", ivpBouncingBallDrag sampleFn),
        ("bouncingBallDragEnergy", ivpBouncingBallDragEnergy sampleFn),
        ("bouncingBallDragPosNeg", ivpBouncingBallDragPosNeg sampleFn),
        ("bouncingBallDragPosNegEnergy", ivpBouncingBallDragPosNegEnergy sampleFn),
        ("bouncingBallDragNewtonianGravityEnergy", ivpBouncingBallDragNewtonianGravityEnergy sampleFn),
        ("bouncingBallDragPosNegNewtonianGravityEnergy", ivpBouncingBallDragPosNegNewtonGravityEnergy sampleFn),
        ("bouncingBallNewtonianGravityEnergy", ivpBouncingBallNewtonianGravityEnergy sampleFn),
        ("bouncingBallCubicDrag", ivpBouncingBallCubicDrag sampleFn),
        ("bouncingBallCubicDragEnergy", ivpBouncingBallCubicDragEnergy sampleFn),
        ("bouncingBallCircle", ivpBouncingBallCircle sampleFn),
        ("2BeadColumnEnergy", ivp2BeadColumnEnergy sampleFn),
        ("2BeadColumnEnergyVDiff", ivp2BeadColumnEnergyVDiff sampleFn),
--        ,
--        ("bouncingBallVibr-graze", ivpBouncingBallVibr_AtTime 2 sampleFn),
--    -- TODO: define "bouncingBallVibrEnergy-graze" 
--        ("bouncingBallDrop", ivpBouncingBallDrop_AtTime 3 2 0 5 sampleFn),
--        ("bouncingBallEnergyDrop", ivpBouncingBallEnergyDrop_AtTime 3 2 0 5 sampleFn),
--        ("twoBouncingBallsDrop", ivpTwoBouncingBallsDrop_AtTime 30 20 25 10 45 sampleFn),
--        ("twoBouncingBallsEnergyDrop", ivpTwoBouncingBallsEnergyDrop_AtTime 30 20 25 10 45 sampleFn),
--    -- TODO: fix breakage at time 20
--        ("bouncingSpring-4", ivpBouncingSpring_AtTime 4 sampleFn),
        ("twoTanks", ivpTwoTanks 0 sampleFn),
        ("twoTanksR", ivpTwoTanks 0.5 sampleFn),
        ("twoTanksSum", ivpTwoTanksSum 0 sampleFn),
        ("twoTanksSumR", ivpTwoTanksSum 0.5 sampleFn)
    ]


ivpByName :: 
    (Var f ~ String,
     HasConstFns f,
     CanEvaluate f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    => 
    String {-^ IVP name - see source code for the list -} -> 
    Double {-^ end time -} -> 
    f {-^ sample function of the type to be used in simulation -} -> 
    Maybe (HybridIVP f)
ivpByName name endTimeDbl sampleFn =
    do
    ivp <- Map.lookup name $ ivpByNameMap sampleFn
    return $ ivp { hybivp_tEnd = z <+>| endTimeDbl }
    where
    z = zero sampleDom
    sampleDom = getSampleDomValue sampleFn
    
ivpByNameReportError ::
    (Var f ~ String,
     HasConstFns f,
     CanEvaluate f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    => 
    String ->
    Double -> 
    f -> 
    HybridIVP f
ivpByNameReportError ivpName endTimeDbl samplePoly =
    case ivpByName ivpName endTimeDbl samplePoly of
        Just ivp -> ivp
        _ -> 
            error $
                "unknown ivp: " ++ ivpName
                ++ "\n known ivps:\n"
                ++ unlines (map ("    " ++) (ivpNames samplePoly))
    
ivpNames :: 
    (Var f ~ String,
     HasConstFns f,
     CanEvaluate f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    =>
    f -> [String]
ivpNames sampleFn = Map.keys $ ivpByNameMap sampleFn
    

--ivpExpDecay_resetTHalf ::
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedReal (Domain f),
--     Show (Domain f)
--    )
--    => 
--    f -> HybridIVP f
--ivpExpDecay_resetTHalf (sampleFn :: f) =
--    ivp
--    where
----    system :: HybridSystem f
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","time"],
--            hybsys_modeFields = Map.fromList [(modeBefore, odeBefore), (modeAfter, odeAfter)],
--            hybsys_modeInvariants = Map.fromList [(modeBefore, id), (modeAfter, id)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventReset, (modeAfter, resetReset))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeBefore = HybSysMode "before"
--    modeAfter = HybSysMode "after"
--    odeBefore, odeAfter :: [f] -> [f]
--    odeBefore [x,time] = [neg x, newConstFnFromSample time (toD 1)]
--    odeAfter = odeBefore
--    eventReset = HybSysEventKind "reset"
--    resetReset :: [f] -> [f]
--    resetReset [x,time] = [newConstFnFromSample x initValue, time]
--    eventSpecMap (HybSysMode "after") = Map.empty -- reset only once!
--    eventSpecMap _ =
--        Map.singleton eventReset $
--            ([True,True], timeDip, const (Just True), timeReset)
--        where
--        timeDip [_, t] = tEventP <-> t
--            where
--            tEventP = newConstFnFromSample t $ (one sampleDom) <*>| tEventDbl
--        timeReset [x,t] = [x,zP]
--            where
--            zP = zero t
--    tEventDbl = 0.5 :: Double
--    tEvent = (zero sampleDom) <+>| tEventDbl
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = toD 0,
--            hybivp_tEnd = toD 1,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeBefore [initValue, tStart],
--            hybivp_maybeExactStateAtTEnd = Just $
--                Map.singleton modeAfter [xEnd, tEnd <-> tEvent]
--        }
--    description =
--        "v = -x; if t = " ++ show tEventDbl ++ " then x := " ++ show initValue 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initValue
--    initValue = (one sampleDom) :: Domain f
--    tStart = hybivp_tStart ivp
--    tEnd = hybivp_tEnd ivp
--    xEnd = (one sampleDom) <*>| (exp (-tEndDbl+tEventDbl) :: Double)
--    tEndDbl :: Double
--    (Just tEndDbl) = ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort tEnd (0::Double)) 0 tEnd
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--ivpExpDecay_resetOn34 ::
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedReal (Domain f),
--     Show (Domain f)
--    )
--    => 
--    f -> HybridIVP f
--ivpExpDecay_resetOn34 (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x"],
--            hybsys_modeFields = Map.fromList [(modeNormal, odeNormal)],
--            hybsys_modeInvariants = Map.fromList [(modeNormal, id)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventReset, (modeNormal, resetReset))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeNormal = HybSysMode "normal"
--    odeNormal :: [f] -> [f]
--    odeNormal [x] = [neg x]
--    eventReset = HybSysEventKind "reset"
--    resetReset :: [f] -> [f]
--    resetReset [x] = [newConstFnFromSample x initValue]
--    eventSpecMap _mode =
--        Map.singleton eventReset $
--            ([True], xDip, const (Just True), id)
--        where
--        xDip [x] = x <-> xEventFn
--            where
--            xEventFn = newConstFnFromSample x $ (toD 1) <*>| xEventDbl
--
--    xEventDbl = 0.75 :: Double
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = toD 0,
--            hybivp_tEnd = toD 1,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeNormal [initValue],
--            hybivp_maybeExactStateAtTEnd = Just $
--                Map.singleton modeNormal [xEnd]
--        }
--    description =
--        "v = -x; if x <= " ++ show xEventDbl ++ " then x := " ++ show initValue 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initValue
--    initValue = (toD 1) :: Domain f
--    tStart = hybivp_tStart ivp
--    tEnd = hybivp_tEnd ivp
----    tVar = hybivp_tVar ivp
--    xEnd = (toD 1) <*>| (exp (-tEndDbl-3*(log xEventDbl)) :: Double)
--    tEndDbl :: Double
--    (Just tEndDbl) = ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort tEnd (0::Double)) 0 tEnd
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--
--ivpSpringMass_resetTHalf ::
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedReal (Domain f),
--     Show (Domain f)
--    )
--    => 
--    f -> HybridIVP f
--ivpSpringMass_resetTHalf (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v","time"],
--            hybsys_modeFields = Map.fromList [(modeBefore, odeBefore), (modeAfter, odeAfter)],
--            hybsys_modeInvariants = Map.fromList [(modeBefore, id), (modeAfter, id)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventReset, (modeAfter, resetReset))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeBefore = HybSysMode "before"
--    modeAfter = HybSysMode "after"
--    odeBefore, odeAfter :: [f] -> [f]
--    odeBefore [x,v,time] = [v, neg x, newConstFnFromSample time (toD 1)]
--    odeAfter = odeBefore
--    eventReset = HybSysEventKind "reset"
--    resetReset :: [f] -> [f]
--    resetReset [x,_v,time] = map (newConstFnFromSample x) initValues ++ [time]
--    eventSpecMap (HybSysMode "after") = Map.empty -- reset only once!
--    eventSpecMap _ =
--        Map.singleton eventReset $
--            ([True,True,True], timeDip, const (Just True), timeReset)
--        where
--        timeDip [_, _, t] = tEventP <-> t
--            where
--            tEventP = newConstFnFromSample t $ (toD 1) <*>| tEventDbl
--        timeReset [x,v,t] = [x,v,zP]
--            where
--            zP = zero t
--    tEventDbl = 0.5 :: Double
--    tEvent = ((zero sampleDom) :: Domain f) <+>| tEventDbl
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = toD 0,
--            hybivp_tEnd = toD 1,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeBefore (initValues ++ [tStart]),
--            hybivp_maybeExactStateAtTEnd = Just $
--                Map.singleton modeAfter [xEnd, xDerEnd, tEnd <-> tEvent]
--        }
--    description =
--        "x'' = -x; if t = " ++ show tEventDbl ++ " then [x,v] := " ++ show initValues 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
--    initValues@[initX, initX'] = [toD 1,toD 0] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    tEnd = hybivp_tEnd ivp
--    xEnd = (toD 1) <*>| (cos (tEndDbl - tEventDbl) :: Double)
--    xDerEnd = (toD $ -1) <*>| (sin (tEndDbl - tEventDbl) :: Double)
--    tEndDbl :: Double
--    (Just tEndDbl) = ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort tEnd (0::Double)) 0 tEnd
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--    
--    
--ivpSpringMass_resetOn34 :: 
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedReal (Domain f),
--     Show (Domain f)
--    )
--    => 
--    f -> HybridIVP f
--ivpSpringMass_resetOn34 (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v"],
--            hybsys_modeFields = Map.fromList [(modeNormal, odeNormal)],
--            hybsys_modeInvariants = Map.fromList [(modeNormal, id)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventReset, (modeNormal, resetReset))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeNormal = HybSysMode "normal"
--    odeNormal :: [f] -> [f]
--    odeNormal [x,v] = [v, neg x]
--    eventReset = HybSysEventKind "reset"
--    resetReset :: [f] -> [f]
--    resetReset [x,_v] = map (newConstFnFromSample x) initValues
--    eventSpecMap _mode =
--        Map.singleton eventReset $
--            ([True, True], xDip, const (Just True), id)
--        where
--        xDip [x,_v] = x <-> xEventFn
--            where
--            xEventFn = newConstFnFromSample x $ (toD 1) <*>| xEventDbl
--    xEventDbl = 0.75 :: Double
--    tEventDbl = acos xEventDbl -- 0.72273424781341...
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = toD 0,
--            hybivp_tEnd = toD 1,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeNormal (initValues ++ [tStart]),
--            hybivp_maybeExactStateAtTEnd = Just $
--                Map.singleton modeNormal [xEnd, xDerEnd, tEnd]
--        }
--    description =
--        "x'' = -x; if x <= " ++ show xEventDbl ++ " then [x,v] := " ++ show initValues 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
--    initValues@[initX, initX'] = [one sampleDom,zero sampleDom] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    tEnd = hybivp_tEnd ivp
----    tVar = hybivp_tVar ivp
--    xEnd = (one sampleDom) <*>| (cos (tEndDbl - tEventDbl) :: Double)
--    xDerEnd = (toD $ -1) <*>| (sin (tEndDbl - tEventDbl) :: Double)
--    tEndDbl :: Double
--    (Just tEndDbl) = ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort tEnd (0::Double)) 0 tEnd
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn

ivpDampenedPendulum :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpDampenedPendulum (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v] = [v, ((-0.5 :: Double) |<*> v) <-> x]
--    invariantMove = id
    invariantMove [x,v] = Just [x,v]

    eventSpecMap _mode = Map.empty
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "x''=-1.5x'-x"
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initV
    initValues@[initX, initV] = [toD 1, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
--    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn


ivpBouncingBall :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBall (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v] = [v, newConstFnFromSample x (toD $ -10)]
--    invariantMove = id
    invariantMove [x,v] = 
        do 
        xNN <- makeNonneg x 
        return [xNN,v]

    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,v] = 
        do  
        vNP <- makeNonpos v
        _ <- isect z x
        return [z, vNP]
        where
--        z = toD 0 
        
    resetBounce [x,v] = 
        [x, (-0.5 :: Double) |<*> v]
--        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB"
--        "if x = 0 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
    initValues@[initX, initX'] = [toD 5, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn

ivpBouncingBallEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
--        z </\> 
        (v <*> v <+> (toD 20) <*> x)
        -- added zero so that after reset the interval refines the original (model-level hack!)  
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v,r] = [v, newConstFnFromSample x (toD $ -10), newConstFnFromSample r (toD 0)]
    invariantMove [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- |v| = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let absV = ArithInOut.sqrtOut vSqr1
        vNew <- isect v ((neg absV) </\> absV)
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] =
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x,
         (-0.5 :: Double) |<*> v, 
         (toDInterval 0 0.25) <*> r
        ] -- deliberately lose precision to facilitate quicker event tree convergence (HACK!)
    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn


ivpBouncingBallRiseFall :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallRiseFall (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeRise, odeMove), (modeFall, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeRise, invariantRise), (modeFall, invariantFall)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeRise = HybSysMode "rise"
    modeFall = HybSysMode "fall"
    odeMove :: [f] -> [f]
    odeMove [x,v] = [v, newConstFnFromSample x (toD $ -10)]
    invariantRise [x,v] = 
        do 
        xNN <- makeNonneg x
        vNN <- makeNonneg v 
        return [xNN,vNN]
    invariantFall [x,v] = 
        do 
        xNN <- makeNonneg x
        vNP <- makeNonpos v 
        return [xNN,vNP]

    eventSpecMap mode 
        | mode == modeRise =
            Map.singleton eventPeak $
                (modeFall, resetPeak, [True, True], prunePeak)
        | mode == modeFall =
            Map.singleton eventBounce $
                (modeRise, resetBounce, [True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,v] = 
        do
        vNP <- makeNonpos v
        _ <- isect z x
        return [z, vNP]
        where
--        z = toD 0 
        
    resetBounce [x,v] = 
        [x, (-0.5 :: Double) |<*> v]
--        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
    eventPeak = HybSysEventKind "peak"
    prunePeak _ [x,v] = 
        do
        _ <- isect z v
        return [x, z]
    resetPeak [x,v] = [x,v] 
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeFall initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-M"
--        "if x = 0 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
    initValues@[initX, initX'] = [toD 5, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn


ivpBouncingBallRiseFallEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallRiseFallEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = (v <*> v <+> (toD 20) <*> x)
        -- added zero so that after reset the interval refines the original (model-level hack!)  
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeRise, odeMove), (modeFall, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeRise, invariantRise), (modeFall, invariantFall)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeRise = HybSysMode "rise"
    modeFall = HybSysMode "fall"
    odeMove :: [f] -> [f]
    odeMove [x,v,r] = [v, newConstFnFromSample x (toD $ -10), newConstFnFromSample r (toD 0)]
    invariantRise [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- v >= 0:
--        vNN <- makeNonneg v
        -- v = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let v1 = ArithInOut.sqrtOut vSqr1
        vNew <- isect v v1
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    invariantFall [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- v <= 0:
--        vNP <- makeNonpos v
        -- v = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let v1 = neg $ ArithInOut.sqrtOut vSqr1
        vNew <- isect v v1
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] = 
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x, 
         (-0.5 :: Double) |<*> v, 
         (toDInterval 0 0.25) <*> r]
    eventPeak = HybSysEventKind "pk"
    prunePeak _ [x,v,r] = 
        do
        _ <- isect z v
        return [x, z, r]
    resetPeak [x,v,r] = [x,v,r] 
    eventSpecMap mode 
        | mode == modeFall =
            Map.singleton
                eventBounce (modeRise, resetBounce, [True, True, True], pruneBounce)
        | mode == modeRise =
            Map.singleton
                eventPeak (modeFall, resetPeak, [False, True, False], prunePeak)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeFall initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-M"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

ivpBouncingBallFloorRise :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallFloorRise (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","xv","y","yv"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,xv,y,yv] = [xv, newConstFnFromSample x (toD $ -10), yv, newConstFnFromSample y (toD $ 1)]
--    invariantMove = id
    invariantMove [x,xv,y,yv] = 
        do 
        xMyNN <- makeNonneg $ x <-> y 
        return [xMyNN <+> y,xv,y,yv]

    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True, True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,xv,y,yv] = 
        do  
        vDiffNP <- makeNonpos $ xv <-> yv 
        _ <- isect x y
        return [y, vDiffNP <+> yv, y, yv]
        
    resetBounce [x,xv,y,yv] = 
        [x, yv <+> (-0.5 :: Double) |<*> (xv <-> yv), y, yv]
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-R"
--        "if x = y && x' <= y' then post(x') = y'-0.5*(x'-y') else x'' = -10, y'' = 1" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", x'(" ++ show tStart ++ ") = " ++ show initXV
        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
        ++ ", y'(" ++ show tStart ++ ") = " ++ show initYV
    initValues@[initX, initXV, initY, initYV] = [toD 5, toD 0, toD 0, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
--    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn


ivpBouncingBallFloorRiseEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallFloorRiseEnergy (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","xv","y","yv","yvv", "r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    initValues@[initX, initXV, initY, initYV, _initYVV, initR] = 
        [toD 5, toD 0, 
            toD 0, toD 1, toD 0, toD 101] -- linear rise
--            toD 0, toD 1, toD floorRiseAccelD, toD $ 101] -- quadratic
--            toD 3.5, toD 0, toD 6, toD 30] -- cubic hill
                    -- r = 2g(x-y) + (x'-y')^2
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,xv,_y,yv,yvv,r] =
        [xv, 
         newConstFnFromSample x (toD $ -10), 
         yv,
         yvv,
         newConstFnFromSample yvv (toD $ 0), -- linear/quadratic (y''' = 0)
--         newConstFnFromSample y (toD $ -6), -- cubic hill
         -- r' = 2(x'-y')(g+x''-y'')
         newConstFnFromSample r (toD $ 0) -- linear
--         (-2 * floorRiseAccelD) |<*> (xv <-> yv) -- quadratic
--         (-2 :: Double) |<*> ((xv <-> yv) <*> yvv) -- cubic
        ]
--    invariantMove = id
--    floorRiseAccelD = 1 :: Double 
--    floorRiseAccelD = -0.375 :: Double 
--    floorRiseAccelD = -2 :: Double 
--    floorRiseThirdD = -11 :: Double 
    invariantMove [x,xv,y,yv,yvv,r] = 
        do 
        -- x >= y:
        xMyNN <- makeNonneg $ x <-> y
        let xFromY = xMyNN <+> y  
        -- r = 2g(x-y) + (x'-y')^2
        -- r >= 0:
        rNN <- makeNonneg r
        -- x-y = (r - (x'-y')^2) / 2g:
        let xvMyv = xv <-> yv
        xvMyvSqr2 <- makeNonneg $ xvMyv <*> xvMyv
        let xFromXV = (rNN <-> xvMyvSqr2) </> (toD 20) <+> y
        xNew <- isect xFromY xFromXV
        -- |x'-y'| = sqrt(r - 2g(x-y)):
        xvMyvSqr <- makeNonneg $ rNN <-> ((toD 20) <*> xMyNN)
        let xvMyvAbs = ArithInOut.sqrtOut xvMyvSqr
        xvNew <- isect xv ((neg xvMyvAbs </\> xvMyvAbs) <+> yv) 
        return [xNew,xvNew,y,yv,yvv,rNN]

    eventSpecMap _mode =
        Map.singleton eventBounce $
--            (modeMove, resetBounce, [True, True, False, False, False, True], pruneBounce)
            (modeMove, resetBounce, [True, True, True, True, True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,xv,y,yv,yvv,r] = 
        do  
        vDiffNP <- makeNonpos $ xv <-> yv 
        _ <- isect x y
        return [y, vDiffNP <+> yv, y, yv,yvv,r]
        
    resetBounce [x,xv,y,yv,yvv,r] = 
        [x, 
         yv <+> (-0.5 :: Double) |<*> (xv <-> yv), -- x' := y' - (x' - y')/2 
         y, yv, yvv,
         (toDInterval 0 0.25) <*> r] -- r := r/4
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description = 
        "BB-R+"
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", x'(" ++ show tStart ++ ") = " ++ show initXV
        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
        ++ ", y'(" ++ show tStart ++ ") = " ++ show initYV
        ++ ", r(" ++ show tStart ++ ") = " ++ show initR
    tStart = hybivp_tStart ivp
--    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

ivpBouncingBallFloorDropEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallFloorDropEnergy (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","xv","y","tm","r"],
            hybsys_modeFields = Map.fromList [(modeMove1, odeMove), (modeMove2, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove1, invariantMove1), (modeMove2, invariantMove2)],
            hybsys_eventSpecification = eventSpecMap
        }
    initValues = [initX, initXV, initY, initTM, initR]
    initX = toD 6
    initXV = toD 0
    initY = toD 1
    initTM = toD 0
    initR = toD 100 -- r = 2g(x-y) + (x'-y')^2
                  
    tDrop = 3.03125
    yDrop = 0

    modeMove1 = HybSysMode "move1"
    modeMove2 = HybSysMode "move2"
    odeMove :: [f] -> [f]
    odeMove [x,xv,_y,_t,r] =
        [xv, 
         newConstFnFromSample x (toD $ -10), 
         newConstFnFromSample x (toD $ 0), -- y'
         newConstFnFromSample x (toD $ 1), -- tm'
         -- r' = 2(x'-y')(g+x''-y'')
         newConstFnFromSample r (toD $ 0) -- r'
        ]
    invariantMove1 orig@[_x,_xv,_y,tm,_r] =
        do
        _ <- isect (tStart </\> (toD tDrop)) tm
        invariantMove orig
    invariantMove2 = 
        invariantMove
    invariantMove [x,xv,y,tm,r] = 
        do 
        let yv = zero y
        -- x >= y:
        xMyNN <- makeNonneg $ x <-> y
        let xFromY = xMyNN <+> y  
        -- r = 2g(x-y) + (x'-y')^2
        -- r >= 0:
        rNN <- makeNonneg r
        -- x-y = (r - (x'-y')^2) / 2g:
        let xvMyv = xv <-> yv
        xvMyvSqr2 <- makeNonneg $ xvMyv <*> xvMyv
        let xFromXV = (rNN <-> xvMyvSqr2) </> (toD 20) <+> y
        xNew <- isect xFromY xFromXV
        -- |x'-y'| = sqrt(r - 2g(x-y)):
        xvMyvSqr <- makeNonneg $ rNN <-> ((toD 20) <*> xMyNN)
        let xvMyvAbs = ArithInOut.sqrtOut xvMyvSqr
        xvNew <- isect xv ((neg xvMyvAbs </\> xvMyvAbs) <+> yv) 
        return [xNew,xvNew,y,tm,rNN]

    eventSpecMap mode 
        | mode == modeMove1 =
            Map.fromList $ 
                [ 
                    (eventBounce1, 
                        (modeMove1, resetBounce, 
--                            [True, True, True, True, True], 
                            [True, True, False, False, True], 
                                pruneBounce))
                ,
                    (eventDrop, 
                        (modeMove2, resetDrop, 
                            [True, True, True, True, True], 
                                pruneDrop))
                ]
        | mode == modeMove2 =
            Map.fromList $ 
                [ 
                    (eventBounce2, 
                        (modeMove2, resetBounce, 
                            [True, True, False, False, True], 
--                            [True, True, True, True, True], 
                                pruneBounce))
                ]
        | otherwise =
            error "internal error: eventSpecMap"
                
    eventBounce1 = HybSysEventKind "bounce1"
    eventBounce2 = HybSysEventKind "bounce2"
    pruneBounce _ [x,xv,y,tm,r] = 
        do  
        let yv = zero y
        vDiffNP <- makeNonpos $ xv <-> yv 
        _ <- isect x y
        return [y, vDiffNP <+> yv, y, tm, r]
    pruneBounce _ _ = error "internal error: pruneBounce"
        
    resetBounce [x,xv,y,tm,r] =
        let yv = zero y in 
        [x, 
         yv <+> (-0.5 :: Double) |<*> (xv <-> yv), -- x' := y' - (x' - y')/2 
         y, tm,
         (toDInterval 0 0.25) <*> r] -- r := r/4
    resetBounce _ = error "internal error: resetBounce"
    
    eventDrop = HybSysEventKind "drop"
    pruneDrop _ orig@[_x,_xv,_y,tm,_r] =
        do
        _ <- isect tm (toD tDrop)
        return orig 
    pruneDrop _ _ = error "internal error: pruneDrop"

    resetDrop [x,xv,y,tm,r] =
        [x, xv, 
         toD yDrop, 
         tm,
         ((toD 20) <*> (y <-> (toD yDrop))) <+> r] -- -- r = 2g(x-y) + (x'-y')^2
    resetDrop _ = error "internal error: resetDrop"
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove1 initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description = 
        "BB-R+"
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", x'(" ++ show tStart ++ ") = " ++ show initXV
        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
        ++ ", t(" ++ show tStart ++ ") = " ++ show initTM
        ++ ", r(" ++ show tStart ++ ") = " ++ show initR
    tStart = hybivp_tStart ivp
--    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

ivpBouncingBallDrag :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDrag (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v] = [v, newConstFnFromSample x (toD $ -10) <-> ((ArithInOut.absOut v) <*> v <*>| (0.1 :: Double))]
--    invariantMove = id
    invariantMove [x,v] = 
        do 
        xNN <- makeNonneg x 
        return [xNN,v]

    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,v] = 
        do  
        vNP <- makeNonpos v
        _ <- isect z x
        return [z, vNP]
        where
        z = toD 0 
        
    resetBounce [x,v] = 
        [x, (-0.5 :: Double) |<*> v]
--        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-D"
--        "if x = 0 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
    initValues@[initX, initX'] = [toD 5, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
--    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn



ivpBouncingBallDragPosNeg :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDragPosNeg (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeRise, odeRise), (modeFall, odeFall)],
            hybsys_modeInvariants = Map.fromList [(modeRise, invariantRise), (modeFall, invariantFall)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeRise = HybSysMode "rise"
    modeFall = HybSysMode "fall"
    odeRise :: [f] -> [f]
    odeRise [x,v] = [v, newConstFnFromSample x (toD $ -10) <-> (v <*> v <*>| (0.1 :: Double))]
    odeFall :: [f] -> [f]
    odeFall [x,v] = [v, newConstFnFromSample x (toD $ -10) <+> (v <*> v <*>| (0.1 :: Double))]
    invariantRise [x,v] = 
        do 
        xNN <- makeNonneg x
        vNN <- makeNonneg v 
        return [xNN,vNN]
    invariantFall [x,v] = 
        do 
        xNN <- makeNonneg x
        vNP <- makeNonpos v 
        return [xNN,vNP]

    eventSpecMap mode 
        | mode == modeRise =
            Map.singleton eventPeak $
                (modeFall, resetPeak, [True, True], prunePeak)
        | mode == modeFall =
            Map.singleton eventBounce $
                (modeRise, resetBounce, [True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,v] = 
        do
        vNP <- makeNonpos v
        _ <- isect z x
        return [z, vNP]
        
    resetBounce [x,v] = 
        [x, (-0.5 :: Double) |<*> v]
--        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
    eventPeak = HybSysEventKind "peak"
    prunePeak _ [x,v] = 
        do
        _ <- isect z v
        return [x, z]
    resetPeak [x,v] = [x,v] 
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeFall initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-D2"
--        "if x = 0 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
    initValues@[initX, initX'] = [toD 5, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn

{-

    Minimal system spec:
    
    x' = v
    v' = -10 - 0.1*v*|v|
    
    Extended with energy:
    
    r = v^2 + 20x
    r' = 2v*(-10 - 0.1*v*|v|) + 20v
       = -0.2 * |v|^3
    
-}

ivpBouncingBallDragEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDragEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
--        z </\> 
        (v <*> v <+> (toD 20) <*> x)
        -- added zero so that after reset the interval refines the original (model-level hack!)  
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
--    odeMove [x,v,r] = [v, newConstFnFromSample x (toD $ -10), newConstFnFromSample r (toD 0)]
    odeMove [x,v,_r] = 
        [v, 
         newConstFnFromSample x (toD $ -10) <-> ((ArithInOut.absOut v) <*> v <*>| (0.1 :: Double)), 
         ((ArithInOut.absOut v) <*> v <*> v <*>| (-0.2 :: Double))]
    invariantMove [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- |v| = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let absV = ArithInOut.sqrtOut vSqr1
        vNew <- isect v ((neg absV) </\> absV)
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] =
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x,
         (-0.5 :: Double) |<*> v, 
         (toDInterval 0 0.25) <*> r
        ] -- deliberately lose precision to facilitate quicker event tree convergence (HACK!)
    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-D+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

ivpBouncingBallDragPosNegEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDragPosNegEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        z </\> 
        (v <*> v <+> (toD 20) <*> x)
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeRise, odeRise), (modeFall, odeFall)],
            hybsys_modeInvariants = Map.fromList [(modeRise, invariantRise), (modeFall, invariantFall)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeRise = HybSysMode "rise"
    modeFall = HybSysMode "fall"
    odeRise :: [f] -> [f]
    odeRise [x,v,_r] = 
        [v, 
         newConstFnFromSample x (toD $ -10) <-> (v <*> v <*>| (0.1 :: Double)), 
         (v <*> v <*> v <*>| (-0.2 :: Double))]
    odeFall :: [f] -> [f]
    odeFall [x,v,_r] = 
        [v, 
         newConstFnFromSample x (toD $ -10) <+> (v <*> v <*>| (0.1 :: Double)), 
         (v <*> v <*> v <*>| (0.2 :: Double))]
    invariantRise = invariantMove False
    invariantFall = invariantMove True
    invariantMove vIsNeg [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- v >= 0:
        vNN <- case vIsNeg of
            False -> makeNonneg v
            True -> makeNonpos v
        -- v = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let v1 = (if vIsNeg then neg else id) $ ArithInOut.sqrtOut vSqr1 
        vNew <- isect vNN v1
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ vNew <*> vNew
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] = 
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x, 
         (-0.5 :: Double) |<*> v, 
         (0.25 :: Double) |<*> r]
    eventPeak = HybSysEventKind "pk"
    prunePeak _ [x,v,r] = 
        do
        _ <- isect z v
        return [x, z, r]
    resetPeak [x,v,r] = [x,v,r] 
    eventSpecMap mode 
        | mode == modeFall =
            Map.singleton
                eventBounce (modeRise, resetBounce, [True, True, True], pruneBounce)
        | mode == modeRise =
            Map.singleton
                eventPeak (modeFall, resetPeak, [False, True, False], prunePeak)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeFall initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-D2+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn

{-

    Minimal system spec:
    
    x' = v
    v' = -5000/(20+x)^2 - 0.1*v*|v|
    
    Extended with energy:
    
    r = v^2 + 500 - 10000/(20+x)
    r' = 2v*(-5000/(20+x)^2 - 0.1*v*|v|) + 10000*v/(20+x)^2
       = -10000*v/(20+x)^2 - 0.2*|v|^3 + 10000*v/(20+x)^2
       = - 0.2*|v|^3
-}

ivpBouncingBallDragNewtonianGravityEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     CanEvaluate f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDragNewtonianGravityEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        z </\> 
        (v <*> v <+> (toD 500) <+> (toD (-10000)) </> ((toD 20) <+> x))
        -- r = v^2 + 500 - 10000/(20+x)
        -- r' = 2v(-5000/(20+x)^2) + 10000 v/(20+x)^2 = 0
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v,_r] = 
        [v, 
         (newConstFnFromSample x (toD $ -5000) </> (((20 :: Double) |<+> x) <^> 2)) <-> ((ArithInOut.absOut v) <*> v <*>| (0.1 :: Double)),
         ((ArithInOut.absOut v) <*> v <*> v <*>| (-0.2 :: Double))]
    invariantMove [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- r = v^2 + 500 - 10000/(20+x) --->
        -- v = sqrt(r - 500 + 10000/(20+x)):
        vSqr1 <- makeNonneg $ rNN <+> (toD (-500)) <+> ((toD 10000) </> ((toD 20) <+> xNN))
        let v1 = ArithInOut.sqrtOut vSqr1
        vNew <- 
            case (v >=? z) of
                Just True -> isect v v1
                Just False -> isect v (neg v1)
                _ -> isect v (neg v1 </\> v1)

        -- r = v^2 + 500 - 10000/(20+x) --->
        -- r - v^2 - 500 =  -10000/(20+x) --->
        -- 20 + x =  -10000/(r - v^2 - 500) --->
        -- x = -20 +10000/(v^2 + 500 - r)
        vSqr2 <- makeNonneg $ vNew <*> vNew
        let x2 = (toD (-20)) <+> ((toD 10000)  </> (vSqr2 <-> rNN <+> (toD 500))) 
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] = 
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x, 
         (-0.5 :: Double) |<*> v,
         (0.25 :: Double) |<*> r]
    eventSpecMap _mode = 
        Map.singleton
            eventBounce (modeMove, resetBounce, [True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-DG+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn



ivpBouncingBallDragPosNegNewtonGravityEnergy ::
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     CanEvaluate f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallDragPosNegNewtonGravityEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        z </\> 
        (v <*> v <+> (toD 500) <+> (toD (-10000)) </> ((toD 20) <+> x))
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeRise, odeRise), (modeFall, odeFall)],
            hybsys_modeInvariants = Map.fromList [(modeRise, invariantRise), (modeFall, invariantFall)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeRise = HybSysMode "rise"
    modeFall = HybSysMode "fall"
    odeRise :: [f] -> [f]
    odeRise [x,v,_r] = 
        [v, 
         (newConstFnFromSample x (toD $ -5000) </> (((20 :: Double) |<+> x) <^> 2)) <-> (v <*> v <*>| (0.1 :: Double)),
         (v <*> v <*> v <*>| (-0.2 :: Double))]
    odeFall :: [f] -> [f]
    odeFall [x,v,_r] = 
        [v, 
         (newConstFnFromSample x (toD $ -5000) </> (((20 :: Double) |<+> x) <^> 2)) <+> (v <*> v <*>| (0.1 :: Double)),
         (v <*> v <*> v <*>| (0.2 :: Double))]
    invariantRise = invariantMove False
    invariantFall = invariantMove True

    invariantMove vIsNeg [x,v,r] =
        do
        -- v >= 0:
        vNN <- case vIsNeg of
            False -> makeNonneg v
            True -> makeNonpos v

        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- r = v^2 + 500 - 10000/(20+x) --->
        -- v = sqrt(r - 500 + 10000/(20+x)):
        vSqr1 <- makeNonneg $ rNN <+> (toD (-500)) <+> ((toD 10000) </> ((toD 20) <+> xNN))
        let v1 = ArithInOut.sqrtOut vSqr1
        vNew <- 
            case vIsNeg of
                False -> isect vNN v1
                True -> isect vNN (neg v1)

        -- r = v^2 + 500 - 10000/(20+x) --->
        -- r - v^2 - 500 =  -10000/(20+x) --->
        -- 20 + x =  -10000/(r - v^2 - 500) --->
        -- x = -20 +10000/(v^2 + 500 - r)
        vSqr2 <- makeNonneg $ vNew <*> vNew
        let x2 = (toD (-20)) <+> ((toD 10000)  </> (vSqr2 <-> rNN <+> (toD 500))) 
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]

    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] = 
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x, 
         (-0.5 :: Double) |<*> v, 
         (0.25 :: Double) |<*> r]
    eventPeak = HybSysEventKind "pk"
    prunePeak _ [x,v,r] = 
        do
        _ <- isect z v
        return [x, z, r]
    resetPeak [x,v,r] = [x,v,r] 
    eventSpecMap mode 
        | mode == modeFall =
            Map.singleton
                eventBounce (modeRise, resetBounce, [True, True, True], pruneBounce)
        | mode == modeRise =
            Map.singleton
                eventPeak (modeFall, resetPeak, [False, True, False], prunePeak)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeFall initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-D2G+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn




{-

    Minimal system spec:
    
    x' = v
    v' = -5000/(20+x)^2
    
    if 
    
    Extended with energy:
    
    r = v^2 + 500 - 10000/(20+x)
    r' = 0
    
    
-}
ivpBouncingBallNewtonianGravityEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     CanEvaluate f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedPowerToNonnegInt f,
     ArithInOut.RoundedDivide f,
     ArithInOut.RoundedMixedAdd f Double,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f,
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallNewtonianGravityEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        z </\> 
        (v <*> v <+> (toD 500) <+> (toD (-10000)) </> ((toD 20) <+> x))
        -- r = v^2 + 500 - 10000/(20+x)
        -- r' = 2v(-5000/(20+x)^2) + 10000 v/(20+x)^2 = 0
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v,r] = 
--        trace (
--           "odeMove:"
--           ++ "\n  Dom(x) = " ++ (show (toAscList $ getDomainBox x))
--           ++ "\n  x =  " ++ (show x)
--           ++ "\n      Ran(x) <- " ++ (show (getRangeOut x))
--           ++ "\n  v = " ++ (show v)
--           ++ "\n      Ran(v) <- " ++ (show (getRangeOut v))
----           ++ "\n  Ran(r) <- " ++ (show r) -- (getRangeOut r))
--        ) $
        [v, 
         newConstFnFromSample x (toD $ -5000) </> (((20 :: Double) |<+> x) <^> 2),
         newConstFnFromSample r (toD 0)]
    invariantMove [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- r = v^2 + 500 - 10000/(20+x) --->
        -- v = sqrt(r - 500 + 10000/(20+x)):
        vSqr1 <- makeNonneg $ rNN <+> (toD (-500)) <+> ((toD 10000) </> ((toD 20) <+> xNN))
        let v1 = ArithInOut.sqrtOut vSqr1
        vNew <- 
            case (v >=? z) of
                Just True -> isect v v1
                Just False -> isect v (neg v1)
                _ -> isect v (neg v1 </\> v1)

        -- r = v^2 + 500 - 10000/(20+x) --->
        -- r - v^2 - 500 =  -10000/(20+x) --->
        -- 20 + x =  -10000/(r - v^2 - 500) --->
        -- x = -20 +10000/(v^2 + 500 - r)
        vSqr2 <- makeNonneg $ vNew <*> vNew
        let x2 = (toD (-20)) <+> ((toD 10000)  </> (vSqr2 <-> rNN <+> (toD 500))) 
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] = 
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x, 
         (-0.5 :: Double) |<*> v,
         (0.25 :: Double) |<*> r]
    eventSpecMap _mode = 
        Map.singleton
            eventBounce (modeMove, resetBounce, [True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-G+"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn


ivpBouncingBallCubicDrag :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     HasConsistency (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallCubicDrag (sampleFn :: f) =
    ivp
    where
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v] = 
        [
            v, 
            newConstFnFromSample x (toD $ -10) <-> (v <*> v <*> v) </>| (1000 :: Double) 
        ]
--    invariantMove = id
    invariantMove [x,v] = 
        do 
        xNN <- makeNonneg x 
        return [xNN,v]

    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True], pruneBounce)
    eventBounce = HybSysEventKind "bounce"
    pruneBounce _ [x,v] = 
        do  
        vNP <- makeNonpos v
        _ <- isect z x
        return [z, vNP]
        
    resetBounce [x,v] = 
        [x, (-0.5 :: Double) |<*> v]
--        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = toD 0,
            hybivp_tEnd = toD 1,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB"
--        "if x = 0 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
    initValues@[initX, initX'] = [toD 5, toD 0] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn



ivpBouncingBallCubicDragEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedMixedDivide f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallCubicDragEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
--        z </\> 
        (v <*> v <+> (toD 20) <*> x)
        -- added zero so that after reset the interval refines the original (model-level hack!)  
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x","v","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x,v,_r] = 
        [
            v, 
            newConstFnFromSample x (toD $ -10) <-> (v <*> v <*> v) </>| (1000 :: Double), 
            (v <*> v <*> v <*> v) </>| (-500 :: Double)
        ]
    invariantMove [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- |v| = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let absV = ArithInOut.sqrtOut vSqr1
        vNew <- isect v ((neg absV) </\> absV)
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x,v,r] =
        do
        _ <- isect z x
        vNP <- makeNonpos v
        return $ [z,vNP,r]
    resetBounce [x,v,r] = 
        [x,
         (-0.5 :: Double) |<*> v, 
         (toDInterval 0 0.25) <*> r
        ] -- deliberately lose precision to facilitate quicker event tree convergence (HACK!)
    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "EBBCD"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
    initValues@[initX, initX', initR] = [toD 5, toD 0, energyWith initX initX'] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn


{-|
    A bouncing ball on a circle, Example 1 from paper:
    http://www.bipedalrobotics.com/uploads/8/0/6/8/8068963/lyapunov_zeno_2012.pdf
-}
ivpBouncingBallCircle :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivpBouncingBallCircle (sampleFn :: f) =
    ivp
    where
    g = toD gD; gD = 1
    e = toD eD; eD = 0.5
    initX1 = toD 0.15
--    initX1 = toD 0.191 -- Zeno
--    initX1 = toD 0.192 -- non-Zeno
--    initX1 = toD 0.2 
--    initX1 = toD 0.21 -- 4 bounces, OK with locate prec 21
--    initX1 = toD 0.22 -- 4 bounces, OK with locate prec 18
--    initX1 = toD 0.25 -- 3 bounces, OK with locate prec 16
    initX2 = toD 1.2
    initV1 = toD 0
    initV2 = toD 0
    initD = (initX1<*>initX1) <+> (initX2<*>initX2)
    initS = toD 0
    initR = neg g <*> initX2

    system =
        HybridSystem
        {
            hybsys_componentNames = ["x1","x2","v1","v2","d","s","r"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove [x1,x2,v1,v2,_d,s,r] = 
        [v1, 
         v2, 
         newConstFnFromSample x1 (toD 0), 
         newConstFnFromSample x2 (neg g),
         (2 :: Double) ArithInOut.|<*> s,
         r,
         (-3 * gD) ArithInOut.|<*> v2
        ]
    odeMove _ = error "odeMove: internal error"
    invariantMove [x1,x2,v1,v2,d,s,r] =
        do
        -- d >= 1:
        dM1NN <- makeNonneg (d <-> c1)
        -- d = x1^2 + x2^2
        x1New <- 
            case (x1 >? z) of
                Just True -> 
                    isect x1 $ ArithInOut.sqrtOut (NumOrd.maxOut z $ dM1NN <+> c1 <-> (x2<^>2))
                _ -> return x1
        x2New <- 
            case (x2 >? z) of
                Just True -> 
                    isect x2 $ ArithInOut.sqrtOut (NumOrd.maxOut z $ dM1NN <+> c1 <-> (x1<^>2))
                _ -> return x2
        dNew <- isect (dM1NN <+> c1) ((x1<^>2) <+> (x2<^>2))
        
        -- s = x1*v1 + x2*v2
                
        -- r = v1^2 + v2^2 - x2*g
                
        return [x1New,x2New,v1,v2,dNew,s,r]
    invariantMove _ = error "invariantMove: internal error"
    eventBounce = HybSysEventKind "bc"
    pruneBounce _ [x1,x2,v1,v2,d,s,r] =
        do
        _ <- isect z (d <-> c1)
        x1New <- isect x1 $ ArithInOut.sqrtOut (NumOrd.maxOut z $ c1 <-> (x2<^>2))
        x2New <- isect x2 $ ArithInOut.sqrtOut (NumOrd.maxOut z $ c1 <-> (x1<^>2))
        sNP <- makeNonpos s
        return $ [x1New,x2New,v1,v2,c1,sNP,r]
    pruneBounce _ _ = error "pruneBounce: internal error"
    resetBounce [x1,x2,v1,v2,d,s,r] = 
        [x1, 
         x2,
         v1 <-> c <*> x1,
         v2 <-> c <*> x2,
         d,
         (neg e) <*> s,
         r <-> (s<^>2)<*>(c1 <-> e<^>2)
        ]
        where
        c = (c1 <+> e) <*> s
    resetBounce _ = error "resetBounce: internal error"
    eventSpecMap _mode =
        Map.singleton eventBounce $
            (modeMove, resetBounce, [True, True, True, True, True, True, True], pruneBounce)
    
    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "BB-O"
--        "" ++ "if x = 0 && v <= 0 then post(v) = -v/2, post(r) = r/4 else x''= -10, r' = 0, r = v^2+20x, x >= 0, r >= 0)" 
        ++ "; x1(" ++ show tStart ++ ") = " ++ show initX1
        ++ "; x2(" ++ show tStart ++ ") = " ++ show initX2
        ++ ", v1(" ++ show tStart ++ ") = " ++ show initV1
        ++ ", v2(" ++ show tStart ++ ") = " ++ show initV2
        ++ ", d(" ++ show tStart ++ ") = " ++ show initD
        ++ ", s(" ++ show tStart ++ ") = " ++ show initS
        ++ ", r(" ++ show tStart ++ ") = " ++ show initR
    initValues = [initX1, initX2, initV1, initV2, initD, initS, initR] 
    tStart = hybivp_tStart ivp
    z = toD 0
    c1 = toD 1
    toD = dblToReal sampleDom
--    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

--ivpBouncingBallVibr_AtTime :: 
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    f ->
--    HybridIVP f
--ivpBouncingBallVibr_AtTime tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v","y","w"],
--            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventBounce, (modeMove, resetBounce))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove = HybSysMode "move"
--    odeMove :: [f] -> [f]
--    odeMove [x,v,y,w] = 
--        [v, newConstFnFromSample x (toD $ -0.81056947), -- 8/pi^2 
--         w, (1::Double) |<*> (neg y)]
----    invariantMove = id
--    invariantMove [x,v,y,w] = [y <+> (makeNonneg (x <-> y)),v,y,w]
--    eventBounce = HybSysEventKind "bounce"
--    pruneBounce [_x,v,y,w] = [y, w <-> (makeNonneg (w <-> v)),y,w]
--    resetBounce :: [f] -> [f]
--    resetBounce [x,v,y,w] = 
--        [x, w <+> ((-0.5 :: Double) |<*> (v <-> w)), y, w]
----        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
--    eventSpecMap _mode =
--        Map.singleton eventBounce $
--            ([True, True, False, False], xDip, vNegative, pruneBounce)
--        where
--        xDip [x,_v,y,_w] = x <-> y
--        vNegative [_x,v,_y,w] = (v <-> w <? z)
--
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if x = y && v <= w then post(v) = w -0.5*(pre(v)-prec(w)) else x'' = -10, y'' = -y" 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initV
--        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
--        ++ ", w(" ++ show tStart ++ ") = " ++ show initW
--    initValues@[initX, initV, initY, initW] = (map toD [0,1.2732395,0,1]) :: [Domain f]
----    initValues@[initX, initX'] = [0,0] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    tEnd = toD tEndDbl
--    z = toD 0
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--
--ivpBouncingBallDrop_AtTime :: 
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedMixedAdd f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    f ->
--    HybridIVP f
--ivpBouncingBallDrop_AtTime groundInitDbl tDropDbl groundDropDbl tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v","y","tt"],
--            hybsys_modeFields = Map.fromList [(modeMove1, odeMove), (modeMove2, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove1, invariantMove), (modeMove2, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList 
--                    [
--                        (eventBounce1, (modeMove1, resetBounce)),
--                        (eventBounce2, (modeMove2, resetBounce)),
--                        (eventDrop, (modeMove2, resetDrop))
--                    ],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove1 = HybSysMode "move1"
--    modeMove2 = HybSysMode "move2"
--    odeMove :: [f] -> [f]
--    odeMove [x,v,_y,_tt] = 
--        [v, newConstFnFromSample x (toD $ -10), 
--         newConstFnFromSample x (toD 0), 
--         newConstFnFromSample x (toD 1)]
----    invariantMove = id
--    invariantMove [x,v,y,tt] = [y <+> (makeNonneg (x<->y)),v,y,tt]
--    eventBounce1 = HybSysEventKind "bounce1"
--    eventBounce2 = HybSysEventKind "bounce2"
--    pruneBounce [_x,v,y,tt] = [y, neg (makeNonneg (neg v)),y,tt]
--    resetBounce :: [f] -> [f]
--    resetBounce [x,v,y,tt] = 
--        [x, ((-0.5 :: Double) |<*> v), y, tt]
--    eventDrop = HybSysEventKind "drop"
--    pruneDrop [x,v,y,_tt] = [x,v,y,tDrop]
--    resetDrop :: [f] -> [f]
--    resetDrop [x,v,y,tt] = 
--        [x, v, newConstFnFromSample y groundDrop, tt]
--    eventSpecMap mode 
--        | mode == modeMove1 =
--            (eventsBounce eventBounce1) `Map.union` eventsDrop
--        | mode == modeMove2 =
--            eventsBounce eventBounce2
--        where
--        eventsDrop =
--            Map.singleton eventDrop $
--                ([False, False, True, False], tDip, const (Just True), pruneDrop)
--            where
--            tDip [_x,_v,_y,tt] = tDropP <-> tt
--                where
--                tDropP = newConstFnFromSample tt tDrop
--        eventsBounce eventBounce =
--            Map.singleton eventBounce $
--                ([True, True, False, False], xDip, vNegative, pruneBounce)
--            where
--            xDip [x,_v,y,_tt] = x <-> y
--            vNegative [_x,v,_y,_tt] = (v <? z)
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = toD 0,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove1 initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if t = " ++ show tDrop ++ " then post(y) = " ++ show groundDrop 
--        ++ "else (if x = y && v <= 0 then post(v) = -0.5*(pre(v)) else x'' = -10, y' = 0)" 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initV
--        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
--    initValues@[initX, initV, initY, _initTT] = (map toD [5,0,groundInitDbl,0]) :: [Domain f]
----    initValues@[initX, initX'] = [0,0] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    [_groundInit, tDrop, groundDrop, tEnd] = map toD [groundInitDbl, tDropDbl, groundDropDbl, tEndDbl]
--    z = toD 0
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--ivpBouncingBallEnergyDrop_AtTime :: 
--    (Var f ~ String,
--     HasConstFns f,
--     RefOrd.RoundedLattice f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedMixedAdd f Double,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedMixedDivide f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     ArithInOut.RoundedSquareRoot (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    f ->
--    HybridIVP f
--ivpBouncingBallEnergyDrop_AtTime groundInitDbl tDropDbl groundDropDbl tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    energyWith x v = z </\> (v <*> v <+> (toD 20) <*> x)
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v","r","y","tt"],
--            hybsys_modeFields = Map.fromList [(modeMove1, odeMove), (modeMove2, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove1, invariantMove), (modeMove2, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList 
--                    [
--                        (eventBounce1, (modeMove1, resetBounce)),
--                        (eventBounce2, (modeMove2, resetBounce)),
--                        (eventDrop, (modeMove2, resetDrop))
--                    ],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove1 = HybSysMode "move1"
--    modeMove2 = HybSysMode "move2"
--    odeMove :: [f] -> [f]
--    odeMove [x,v,r,y,tt] = 
--        [v, 
--         newConstFnFromSample x (toD $ -10), 
--         newConstFnFromSample r (toD 0), 
--         newConstFnFromSample y (toD 0), 
--         newConstFnFromSample tt (toD 1)]
----    invariantMove = id
--    invariantMove [x,v,r,y,tt] = 
--        [xNN <\/> x2, 
--         v <\/> ((neg absV) </\> absV), 
--         rNN,
--         y, tt]
--        {- making use of the energy conservation law: 
--           (v)^2 + 2gx = r
--           
--           which implies 
--           |v| = sqrt(r - 2gx) 
--           x = (r - (v)^2) / 2g 
--        -}
--        where
--        rNN = makeNonneg r
--        xNN = y <+> (makeNonneg (x <-> y))
--        absV = sqrtOut $ makeNonneg $ rNN <-> ((toD 20) <*> xNN)
--        x2 = (rNN <-> (makeNonneg $ v <*> v)) </> (toD 20)
--    eventBounce1 = HybSysEventKind "bounce1"
--    eventBounce2 = HybSysEventKind "bounce2"
--    pruneBounce [_x,v,r,y,tt] = [y, neg (makeNonneg (neg v)),r,y,tt]
--    resetBounce :: [f] -> [f]
--    resetBounce [x,v,r,y,tt] = 
--        [x, 
--         ((-0.5 :: Double) |<*> v),
--         y2g <+> ((r <-> y2g) </>| (4 :: Double)), -- Kinetic energy is scaled by 1/4  
--         y, tt]
--         where
--         y2g = (20 :: Double) |<*> y -- potential energy
--    eventDrop = HybSysEventKind "drop"
--    pruneDrop [x,v,r,y,_tt] = [x,v,r,y,tDrop]
--    resetDrop :: [f] -> [f]
--    resetDrop [x,v,r,y,tt] = 
--        [x, v, 
--         zP </\> r, -- include 0 to create a refinement fixed point (hack?)  
--         newConstFnFromSample y groundDrop, 
--         tt]
--        where
--        zP = newConstFnFromSample r z
--    eventSpecMap mode 
--        | mode == modeMove1 =
--            eventsBounce eventBounce1 `Map.union` eventsDrop
--        | mode == modeMove2 =
--            eventsBounce eventBounce2
--        where
--        eventsDrop =
--            Map.singleton eventDrop $
--                ([False, False, True, True, False], tDip, const (Just True), pruneDrop)
--            where
--            tDip [_x,_v,_r, _y,tt] = tDropP <-> tt
--                where
--                tDropP = newConstFnFromSample tt tDrop
--        eventsBounce eventBounce =
--            Map.singleton eventBounce $
--                ([True, True, True, False, False], xDip, vNegative, pruneBounce)
--            where
--            xDip [x,_v,_r,y,_tt] = x <-> y
--            vNegative [_x,v,_r,_y,_tt] = (v <? z)
--   
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove1 initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if t = " ++ show tDrop ++ " then post(y) = " ++ show groundDrop 
--        ++ "else (if x = y && v <= 0 then post(v) = -0.5*(pre(v)) else x'' = -10, y' = 0)" 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initV
--        ++ ", r(" ++ show tStart ++ ") ∊ " ++ show initR
--        ++ "; y(" ++ show tStart ++ ") = " ++ show initY
--    initValues@[initX, initV, initR, initY, _initTT] = 
--        [toD 5,toD 0,
--         energyWith initX initV,
--         groundInit,
--         toD 0] :: [Domain f]
----    initValues@[initX, initX'] = [0,0] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    [groundInit, tDrop, groundDrop, tEnd] = map toD [groundInitDbl, tDropDbl, groundDropDbl, tEndDbl]
--    z = toD 0
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--ivpTwoBouncingBallsDrop_AtTime :: 
--    (Var f ~ String,
--     HasConstFns f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedMixedAdd f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    f ->
--    HybridIVP f
--ivpTwoBouncingBallsDrop_AtTime 
--        groundInitDbl tDrop1Dbl tDrop2PreDbl groundDropDbl tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    g = 9.81 :: Double
--    c = 0.8 :: Double
--    tDrop2Dbl = tDrop2PreDbl + 1
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x1","v1","y1","x2","v2","y2","tt"],
--            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList 
--                    [
--                        (eventBounce1, (modeMove, resetBounce1))
--                        ,
--                        (eventDrop1, (modeMove, resetDrop1))
--                        ,
--                        (eventBounce2, (modeMove, resetBounce2))
--                        ,
--                        (eventDrop2, (modeMove, resetDrop2))
--                    ],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove = HybSysMode "move"
--    odeMove :: [f] -> [f]
--    odeMove [x1,v1,y1,x2,v2,y2,tt] = 
--        [v1, 
--         newConstFnFromSample x1 (toD (-g)), 
--         newConstFnFromSample y1 (toD 0),
--         v2, 
--         newConstFnFromSample x2 (toD (-g)), 
--         newConstFnFromSample y2 (toD 0),
--         newConstFnFromSample tt (toD 1)]
----    invariantMove = id
--    invariantMove [x1,v1,y1,x2,v2,y2,tt] = 
--        [y1 <+> (makeNonneg (x1 <-> y1)),v1,y1,
--         y2 <+> (makeNonneg (x2 <-> y2)),v2,y2,
--         tt]
--    eventBounce1 = HybSysEventKind "bounce1"
--    eventBounce2 = HybSysEventKind "bounce2"
--    pruneBounce1 [_x1,v1,y1,x2,v2,y2,tt] = 
--        [y1, neg (makeNonneg (neg v1)),y1,
--         x2,v2,y2,
--         tt]
--    pruneBounce2 [x1,v1,y1,_x2,v2,y2,tt] = 
--        [x1,v1,y1,
--         y2, neg (makeNonneg (neg v2)),y2,
--         tt]
--    resetBounce1 :: [f] -> [f]
--    resetBounce1 [x1,v1,y1,x2,v2,y2,tt] = 
--        [x1, ((-c) |<*> v1), y1, 
--         x2,v2,y2,
--         tt]
--    resetBounce2 [x1,v1,y1,x2,v2,y2,tt] = 
--        [x1,v1,y1,
--         x2, ((-c) |<*> v2), y2,
--         tt]
--    eventDrop1 = HybSysEventKind "drop1"
--    eventDrop2 = HybSysEventKind "drop2"
--    pruneDrop1 [x1,v1,y1,x2,v2,y2,_tt] = [x1,v1,y1,x2,v2,y2,tDrop1]
--    pruneDrop2 [x1,v1,y1,x2,v2,y2,_tt] = [x1,v1,y1,x2,v2,y2,tDrop2]
--    resetDrop1 :: [f] -> [f]
--    resetDrop1 [x1,v1,y1,x2,v2,y2,tt] = 
--        [x1, v1, newConstFnFromSample y1 groundDrop,
--         x2,v2,y2, 
--         tt <+>| (1 :: Double)] -- jump tt to avoid another drop event (hack!!)
--    resetDrop2 [x1,v1,y1,x2,v2,y2,tt] = 
--        [x1,v1,y1,
--         x2, v2, newConstFnFromSample y2 groundDrop, 
--         tt <+>| (1 :: Double)] -- jump tt to avoid another drop event (hack!!)
--    eventSpecMap _mode =
--        Map.unions [eventsBounce1, eventsDrop1, eventsBounce2, eventsDrop2] 
--        where
--        eventsDrop1 = 
--            Map.singleton eventDrop1
--                ([False, False, True, False, False, False, True], tDip1, tNearDrop1, pruneDrop1)
--            where
--            tDip1 [_x1,_v1,_y1,_x2,_v2,_y2,tt] = tDrop1P <-> tt
--                where
--                tDrop1P = newConstFnFromSample tt tDrop1
--            tNearDrop1 [_x1,_v1,_y1,_x2,_v2,_y2,tt] = tt <? (tDrop1 <+> (toD 0.5))
--        eventsDrop2 =
--            Map.singleton eventDrop2
--                ([False, False, False, False, False, True, True], tDip2, tNearDrop2, pruneDrop2)
--            where
--            tDip2 [_x1,_v1,_y1,_x2,_v2,_y2,tt] = tDrop2P <-> tt
--                where
--                tDrop2P = newConstFnFromSample tt tDrop2
--            tNearDrop2 [_x1,_v1,_y1,_x2,_v2,_y2,tt] = tt <? (tDrop2 <+> (toD 0.5))
--        eventsBounce1 =
--            Map.singleton eventBounce1 $
--                ([True, True, False, False, False, False, False], 
--                    x1Dip, v1Negative, pruneBounce1)
--            where
--            x1Dip [x1,_v1,y1,_x2,_v2,_y2,_tt] = x1 <-> y1
--            v1Negative [_x1,v1,_y1,_x2,_v2,_y2,_tt] = (v1 <? z)
--        eventsBounce2 =
--            Map.singleton eventBounce2 $
--                ([False, False, False, True, True, False, False], 
--                    x2Dip, v2Negative, pruneBounce2)
--            where
--            x2Dip [_x1,_v1,_y1,x2,_v2,y2,_tt] = x2 <-> y2
--            v2Negative [_x1,_v1,_y1,_x2,v2,_y2,_tt] = (v2 <? z)
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if t = " ++ show tDrop1 ++ " then post(y1) = " ++ show groundDrop 
--        ++ "else (if x1 = y1 && v1 <= 0 then post(v1) = -0.5*(pre(v1)) else x1'' = -10, y1' = 0)" 
--        ++ "if t = " ++ show tDrop2 ++ " then post(y2) = " ++ show groundDrop 
--        ++ "else (if x2 = y2 && v2 <= 0 then post(v2) = -0.5*(pre(v2)) else x2'' = -10, y2' = 0)" 
--        ++ "; x1(" ++ show tStart ++ ") = " ++ show initX1
--        ++ ", v1(" ++ show tStart ++ ") = " ++ show initV1
--        ++ "; y1(" ++ show tStart ++ ") = " ++ show initY1
--        ++ "; x2(" ++ show tStart ++ ") = " ++ show initX2
--        ++ ", v2(" ++ show tStart ++ ") = " ++ show initV2
--        ++ "; y2(" ++ show tStart ++ ") = " ++ show initY2
--    initValues@[initX1, initV1, initY1, initX2, initV2, initY2, _initTT] = 
--        [toD 30,toD 14,groundInit,
--         toD 30,toD 25,groundInit,
--         z] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    [groundInit, tDrop1, tDrop2, groundDrop, tEnd] = map toD [groundInitDbl, tDrop1Dbl, tDrop2Dbl, groundDropDbl, tEndDbl]
--    z = toD 0
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--
--ivpTwoBouncingBallsEnergyDrop_AtTime ::
--    (Var f ~ String,
--     HasConstFns f,
--     RefOrd.RoundedLattice f,
--     Neg f,
--     ArithInOut.RoundedSubtr f,
--     ArithInOut.RoundedMixedAdd f Double,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedMixedDivide f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     ArithInOut.RoundedSquareRoot (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    Double -> 
--    f ->
--    HybridIVP f
--ivpTwoBouncingBallsEnergyDrop_AtTime
--        groundInitDbl tDrop1Dbl tDrop2PreDbl groundDropDbl tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    tDrop2Dbl = tDrop2PreDbl + 1
--    g = 9.81 :: Double
--    c = 0.8 :: Double
--    energyWith x v = z </\> (v <*> v <+> (toD 20) <*> x)
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x1","v1","r1","y1","x2","v2","r2","y2","tt"],
--            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList 
--                    [
--                        (eventBounce1, (modeMove, resetBounce1)),
--                        (eventDrop1, (modeMove, resetDrop1))
--                        ,
--                        (eventBounce2, (modeMove, resetBounce2)),
--                        (eventDrop2, (modeMove, resetDrop2))
--                    ],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove = HybSysMode "move"
--    odeMove :: [f] -> [f]
--    odeMove [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [v1,
--         newConstFnFromSample x1 (toD (-g)), 
--         newConstFnFromSample r1 (toD 0), 
--         newConstFnFromSample y1 (toD 0), 
--         v2,
--         newConstFnFromSample x2 (toD (-g)), 
--         newConstFnFromSample r2 (toD 0), 
--         newConstFnFromSample y2 (toD 0),
--         newConstFnFromSample tt (toD 1)]
----    invariantMove = id
--    invariantMove [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [x1NN <\/> x1E, 
--         v1 <\/> ((neg absV1) </\> absV1), 
--         r1NN,
--         y1, 
--         x2NN <\/> x2E, 
--         v2 <\/> ((neg absV2) </\> absV2), 
--         r2NN,
--         y2, 
--         tt]
--        {- making use of the energy conservation law: 
--           (v)^2 + 2gx = r
--           
--           which implies 
--           |v| = sqrt(r - 2gx) 
--           x = (r - (v)^2) / 2g 
--        -}
--        where
--        r1NN = makeNonneg r1
--        x1NN = y1 <+> (makeNonneg (x1 <-> y1))
--        absV1 = sqrtOut $ makeNonneg $ r1NN <-> ((2*g) |<*> x1NN)
--        x1E = (r1NN <-> (makeNonneg $ v1 <*> v1)) </>| (2*g)
--        r2NN = makeNonneg r2
--        x2NN = y2 <+> (makeNonneg (x2 <-> y2))
--        absV2 = sqrtOut $ makeNonneg $ r2NN <-> ((2*g) |<*> x2NN)
--        x2E = (r2NN <-> (makeNonneg $ v2 <*> v2)) </>| (2*g)
--    eventBounce1 = HybSysEventKind "bounce1"
--    eventBounce2 = HybSysEventKind "bounce2"
--    pruneBounce1 [_x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [y1, neg (makeNonneg (neg v1)),r1,y1,
--         x2,v2,r2,y2,
--         tt]
--    pruneBounce2 [x1,v1,r1,y1,_x2,v2,r2,y2,tt] = 
--        [x1,v1,r1,y1,
--         y2, neg (makeNonneg (neg v2)),r2,y2,
--         tt]
--    resetBounce1 :: [f] -> [f]
--    resetBounce1 [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [x1, 
--         ((-c) |<*> v1),
--         yyg1 <+> ((c*c) |<*> (r1 <-> yyg1)), -- Kinetic energy is scaled by c^2  
--         y1, 
--         x2,v2,r2,y2,
--         tt]
--         where
--         yyg1 = (2*g) |<*> y1
--    resetBounce2 [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [x1,v1,r1,y1,
--         x2, 
--         ((-c) |<*> v2),
--         yyg2 <+> ((c*c) |<*> (r2 <-> yyg2)), -- Kinetic energy is scaled by c^2  
--         y2, 
--         tt]
--         where
--         yyg2 = (2*g) |<*> y2
--    eventDrop1 = HybSysEventKind "drop1"
--    eventDrop2 = HybSysEventKind "drop2"
--    pruneDrop1 [x1,v1,r1,y1,x2,v2,r2,y2,_tt] = 
--        [x1,v1,r1,y1,x2,v2,r2,y2,tDrop1]
--    pruneDrop2 [x1,v1,r1,y1,x2,v2,r2,y2,_tt] = 
--        [x1,v1,r1,y1,x2,v2,r2,y2,tDrop2]
--    resetDrop1 :: [f] -> [f]
--    resetDrop1 [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [x1, v1, 
--         zP </\> r1, -- include 0 to create a refinement fixed point (hack!!)  
--         newConstFnFromSample y1 groundDrop,
--         x2,v2,r2,y2, 
--         tt <+>| (1 :: Double)] -- move clock to avoid another drop event (hack!!)
--        where
--        zP = newConstFnFromSample tt z
--    resetDrop2 [x1,v1,r1,y1,x2,v2,r2,y2,tt] = 
--        [x1,v1,r1,y1, 
--         x2, v2, 
--         zP </\> r2, -- include 0 to create a refinement fixed point (hack!!)  
--         newConstFnFromSample y2 groundDrop,
--         tt <+>| (1 :: Double)] -- move clock to avoid another drop event (hack!!)
--        where
--        zP = newConstFnFromSample tt z
--    eventSpecMap _mode =
--        Map.unions [eventsBounce1, eventsDrop1, eventsBounce2, eventsDrop2] 
--        where
--        eventsDrop1 = 
--            Map.singleton eventDrop1
--                ([False, False, True, True, False, False, False, False, True], tDip1, tNearDrop1, pruneDrop1)
--            where
--            tDip1 [_x1,_v1,_r1,_y1,_x2,_v2,_r2,_y2,tt] = tDrop1P <-> tt
--                where
--                tDrop1P = newConstFnFromSample tt tDrop1
--            tNearDrop1 [_x1,_v1,_r1,_y1,_x2,_v2,_r2,_y2,tt] = tt <? (tDrop1 <+> (toD 0.5))
--        eventsDrop2 =
--            Map.singleton eventDrop2
--                ([False, False, False, False, False, False, True, True, True], tDip2, tNearDrop2, pruneDrop2)
--            where
--            tDip2 [_x1,_v1,_r1,_y1,_x2,_v2,_r2,_y2,tt] = tDrop2P <-> tt
--                where
--                tDrop2P = newConstFnFromSample tt tDrop2
--            tNearDrop2 [_x1,_v1,_r1,_y1,_x2,_v2,_r2,_y2,tt] = tt <? (tDrop2 <+> (toD 0.5))
--        eventsBounce1 =
--            Map.singleton eventBounce1 $
--                ([True, True, True, False, False, False, False, False, False], 
--                    x1Dip, v1Negative, pruneBounce1)
--            where
--            x1Dip [x1,_v1,_r1,y1,_x2,_v2,_r2,_y2,_tt] = x1 <-> y1
--            v1Negative [_x1,v1,_r1,_y1,_x2,_v2,_r2,_y2,_tt] = (v1 <? z)
--        eventsBounce2 =
--            Map.singleton eventBounce2 $
--                ([False, False, False, False, True, True, True, False, False], 
--                    x2Dip, v2Negative, pruneBounce2)
--            where
--            x2Dip [_x1,_v1,_r1,_y1,x2,_v2,_r2,y2,_tt] = x2 <-> y2
--            v2Negative [_x1,_v1,_r1,_y1,_x2,v2,_r2,_y2,_tt] = (v2 <? z)
--    
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if t = " ++ show tDrop1 ++ " then post(y1) = " ++ show groundDrop 
--        ++ "else (if x1 = y1 && v1 <= 0 then post(v1) = -" ++ show c ++ "*(pre(v1)) else x1'' = -" ++ show g ++ ", y1' = 0)" 
--        ++ "if t = " ++ show tDrop2 ++ " then post(y2) = " ++ show groundDrop 
--        ++ "else (if x2 = y2 && v2 <= 0 then post(v2) = -" ++ show c ++ "*(pre(v2)) else x2'' = -" ++ show g ++ ", y2' = 0)" 
--        ++ "; x1(" ++ show tStart ++ ") = " ++ show initX1
--        ++ ", v1(" ++ show tStart ++ ") = " ++ show initV1
--        ++ ", r1(" ++ show tStart ++ ") ∊ " ++ show initR1
--        ++ "; y1(" ++ show tStart ++ ") = " ++ show initY1
--        ++ "; x2(" ++ show tStart ++ ") = " ++ show initX2
--        ++ ", v2(" ++ show tStart ++ ") = " ++ show initV2
--        ++ ", r2(" ++ show tStart ++ ") ∊ " ++ show initR2
--        ++ "; y2(" ++ show tStart ++ ") = " ++ show initY2
--    initValues@[initX1, initV1, initR1, initY1, initX2, initV2, initR2, initY2, _initTT] = 
--        [toD 30,toD 14,energyWith initX1 initV1,groundInit,
--         toD 30,toD 25,energyWith initX2 initV2,groundInit,
--         z] :: [Domain f]
----    initValues@[initX, initX'] = [0,0] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    [groundInit, tDrop1, tDrop2, groundDrop, tEnd] = 
--        map toD [groundInitDbl, tDrop1Dbl, tDrop2Dbl, groundDropDbl, tEndDbl]
--    z = toD 0
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--ivpBouncingSpring_AtTime :: 
--    (Var f ~ String,
--     HasConstFns f,
--     ArithInOut.RoundedMixedAdd f Double,
--     ArithInOut.RoundedMixedMultiply f Double,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    f -> 
--    HybridIVP f
--ivpBouncingSpring_AtTime tEndDbl (sampleFn :: f) =
--    ivp
--    where
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x","v"],
--            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
--            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(eventBounce, (modeMove, resetBounce))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeMove = HybSysMode "move"
--    odeMove :: [f] -> [f]
--    odeMove [x,v] = [v, (-1 :: Double) |<*> x]
--    invariantMove [x,v] = [(toD 1) <+> (makeNonneg (x <-> (toD 1))), v]
--    eventBounce = HybSysEventKind "bounce"
--    pruneBounce [_x,v] = [toD 1, neg $ makeNonneg $ neg v]
--    resetBounce :: [f] -> [f]
--    resetBounce [x,v] = 
--        [x, (-0.5 :: Double) |<*> v]
----        [newConstFnFromSample v 0, (0 :: Double) |<*> v]
--    eventSpecMap _mode =
--        Map.singleton eventBounce $
--            ([True, True], xDip, vNegative, pruneBounce)
--        where
--        xDip [x,_v] = x <+>| (-1 :: Double)
--        vNegative [_x,v] = (v <? z)
--
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeMove initValues,
--            hybivp_maybeExactStateAtTEnd = Nothing
--        }
--    description =
--        "if x = 1 && v <= 0 then post(v) = -0.5*pre(v) else x'' = -10x" 
--        ++ "; x(" ++ show tStart ++ ") = " ++ show initX
--        ++ ", v(" ++ show tStart ++ ") = " ++ show initX'
--    initValues@[initX, initX'] = [toD 1,toD 1] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    z = toD 0
--    tEnd = toD tEndDbl
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn
--
--ivpTwoTanks_AfterZeno :: 
--    (Var f ~ String,
--     HasConstFns f,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     Show (Domain f)
--    )
--    => 
--    Double -> 
--    f -> 
--    HybridIVP f
--ivpTwoTanks_AfterZeno tEndMinusTZenoDbl (sampleFn :: f) =
--    ivp
--    where
--    v1 = toD 2
--    v2 = toD 3
--    w = toD 4
--    tZenoDbl = 2
--    tEndDbl = tEndMinusTZenoDbl + tZenoDbl
--    system =
--        HybridSystem
--        {
--            hybsys_componentNames = ["x1","x2"],
--            hybsys_modeFields = Map.fromList [(modeFill1, odeFill1), (modeFill2, odeFill2)],
--            hybsys_modeInvariants = Map.fromList [(modeFill1, invariant), (modeFill2, invariant)],
--            hybsys_eventModeSwitchesAndResetFunctions =
--                Map.fromList [(event1To2, (modeFill2, id)), (event2To1, (modeFill1, id))],
--            hybsys_eventSpecification = eventSpecMap
--        }
--    modeFill1 = HybSysMode "fill1"
--    modeFill2 = HybSysMode "fill2"
--    odeFill1 :: [f] -> [f]
--    odeFill1 [_x1,_x2] = [newConstFnFromSample _x1 (w <-> v1), newConstFnFromSample _x1 (neg v2)]
--    odeFill2 :: [f] -> [f]
--    odeFill2 [_x1,_x2] = [newConstFnFromSample _x1 (neg v1), newConstFnFromSample _x1 (w <-> v2)]
--    invariant [x1,x2] = [makeNonneg x1, makeNonneg x2]
--    event1To2 = HybSysEventKind "1To2"
--    event2To1 = HybSysEventKind "2To1"
--    prune1To2 [x1,_x2] = [x1, toD 0]
--    prune2To1 [_x1,x2] = [toD 0, x2]
--
--    eventSpecMap (HybSysMode "fill1") =
--        Map.singleton event1To2 $
--            ([True, True], x2Dip, const (Just True), prune1To2)
--        where
--        x2Dip [_x1,x2] = x2
--    eventSpecMap (HybSysMode "fill2") =
--        Map.singleton event2To1 $
--            ([True, True], x1Dip, const (Just True), prune2To1)
--        where
--        x1Dip [x1,_x2] = x1
--
--    ivp :: HybridIVP f
--    ivp =
--        HybridIVP
--        {
--            hybivp_description = description,
--            hybivp_system = system,
--            hybivp_tVar = "t",
--            hybivp_tStart = z,
--            hybivp_tEnd = tEnd,
--            hybivp_initialStateEnclosure = 
--                Map.singleton modeFill1 initValues,
--            hybivp_maybeExactStateAtTEnd = Just $
--                Map.fromList
--                [
--                    (modeFill1, [toD 0,toD 0]),
--                    (modeFill2, [toD 0,toD 0])
--                ]
--        }
--    description =
--        ""
--        ++    "if fill1 then (if x2 = 0 then fill2 else x1' = 4-2, x2' =  -3)"
--        ++ "\n if fill2 then (if x1 = 0 then fill1 else x1' =  -2, x2' = 4-3)"
--        ++ "\n ; x1(" ++ show tStart ++ ") = " ++ show initX1
--        ++    ", x2(" ++ show tStart ++ ") = " ++ show initX2
--    initValues@[initX1, initX2] = [toD 1,toD 1] :: [Domain f]
--    tStart = hybivp_tStart ivp
--    z = toD 0
--    tEnd = toD tEndDbl
--    toD = dblToReal sampleDom
--    sampleDom = getSampleDomValue sampleFn

ivpTwoTanks :: 
    (Var f ~ String,
     HasConstFns f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     Show (Domain f)
    )
    => 
    Double {-^ minimum level that triggers switch (>= 0) -} ->
    f -> 
    HybridIVP f
ivpTwoTanks rD (sampleFn :: f) =
    ivp
    where
    v1 = toD 2
    v2 = toD 3
    w = toD 4
    r = toD rD
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x1","x2"],
            hybsys_modeFields = Map.fromList 
                [(modeFill1, odeFill1), 
                 (modeFill2, odeFill2)
                ],
            hybsys_modeInvariants = Map.fromList 
                [(modeFill1, invariant), 
                 (modeFill2, invariant)
                ],
            hybsys_eventSpecification = eventSpecMap
        }
    modeFill1 = HybSysMode "fill1"
    modeFill2 = HybSysMode "fill2"
    odeFill1 :: [f] -> [f]
    odeFill1 [_x1,_x2] = 
        [newConstFnFromSample _x1 (w <-> v1), 
         newConstFnFromSample _x1 (neg v2)
        ]
    odeFill2 :: [f] -> [f]
    odeFill2 [_x1,_x2] = 
        [newConstFnFromSample _x1 (neg v1), 
         newConstFnFromSample _x1 (w <-> v2)
        ]
    invariant [x1,x2] =
        do
        let x1Limit = NumOrd.minOut x2 r  
        let x2Limit = NumOrd.minOut x1 r  
        --
        x1MlimNN <- makeNonneg (x1 <-> x1Limit)
        let x1AboveLim = x1MlimNN <+> x1Limit 
        x2MlimNN <- makeNonneg (x2 <-> x2Limit)
        let x2AboveLim = x2MlimNN <+> x2Limit 
        --
        x1New <- isect x1AboveLim x1
        x2New <- isect x2AboveLim x2
        return [x1New, x2New]

    eventSpecMap mode
        | mode == modeFill1 =
            Map.singleton event1To2 $
                (modeFill2, id, [True, True], prune1To2)
        | mode == modeFill2 =
            Map.singleton event2To1 $
                (modeFill1, id, [True, True], prune2To1)

    event1To2 = HybSysEventKind "1To2"
    event2To1 = HybSysEventKind "2To1"
    prune1To2 _ [x1,x2] =
        do
        _ <- isect x2 r
        return [x1, r]
        
    prune2To1 _ [x1,x2] =
        do 
        _ <- isect x1 r
        return [r, x2]

    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z, -- will be overridden
            hybivp_initialStateEnclosure = 
                Map.singleton modeFill1 initValues,
            hybivp_maybeExactStateAtTEnd =
                Nothing 
        }
    description =
        ""
        ++ "2T-S"
        ++ "\n ; x1(" ++ show tStart ++ ") = " ++ show initX1
        ++    ", x2(" ++ show tStart ++ ") = " ++ show initX2
    initValues@[initX1, initX2] = [toD 1,toD 1] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn




ivpTwoTanksSum :: 
    (Var f ~ String,
     HasConstFns f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     Show (Domain f)
    )
    => 
    Double {-^ minimum level that triggers switch (>= 0) -} ->
    f -> 
    HybridIVP f
ivpTwoTanksSum rD (sampleFn :: f) =
    ivp
    where
    v1 = toD 2
    v2 = toD 3
    w = toD 4
    r = toD rD
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x1","x2","x12"],
            hybsys_modeFields = Map.fromList 
                [(modeFill1, odeFill1), 
                 (modeFill2, odeFill2)
                ],
            hybsys_modeInvariants = Map.fromList 
                [(modeFill1, invariant), 
                 (modeFill2, invariant)
                ],
            hybsys_eventSpecification = eventSpecMap
        }
    modeFill1 = HybSysMode "fill1"
    modeFill2 = HybSysMode "fill2"
    odeFill1 :: [f] -> [f]
    odeFill1 [_x1,_x2,_x12] = 
        [newConstFnFromSample _x1 (w <-> v1), 
         newConstFnFromSample _x1 (neg v2), 
         newConstFnFromSample _x1 (w <-> v1 <-> v2)
        ]
    odeFill2 :: [f] -> [f]
    odeFill2 [_x1,_x2,_x12] = 
        [newConstFnFromSample _x1 (neg v1), 
         newConstFnFromSample _x1 (w <-> v2),
         newConstFnFromSample _x1 (w <-> v1 <-> v2)
        ]
    invariant [x1,x2,x12] =
        do
        let x1Limit = NumOrd.minOut x2 r  
        let x2Limit = NumOrd.minOut x1 r  
        --
        x1MlimNN <- makeNonneg (x1 <-> x1Limit)
        let x1AboveLim = x1MlimNN <+> x1Limit 
        x2MlimNN <- makeNonneg (x2 <-> x2Limit)
        let x2AboveLim = x2MlimNN <+> x2Limit 
        x12NN <- makeNonneg x12 -- $ NumOrd.maxOut (zero x12) x12
        --
        x12mx2 <- makeNonneg $ x12NN <-> x2AboveLim
        x1New <- isect x1AboveLim x12mx2
        x12mx1 <- makeNonneg $ x12NN <-> x1AboveLim
        x2New <- isect x2AboveLim x12mx1
        --
        let x1px2 = x1AboveLim <+> x2AboveLim
        x12New <- isect x12NN x1px2
        return [x1New, x2New, x12New]

    eventSpecMap mode
        | mode == modeFill1 =
            Map.singleton event1To2 $
                (modeFill2, id, [True, True, True], prune1To2)
        | mode == modeFill2 =
            Map.singleton event2To1 $
                (modeFill1, id, [True, True, True], prune2To1)

    event1To2 = HybSysEventKind "1To2"
    event2To1 = HybSysEventKind "2To1"
    prune1To2 _ [x1,x2, x12] =
        do
        _ <- isect x2 r
        return [x1, r, x12]
        
    prune2To1 _ [x1,x2, x12] =
        do 
        _ <- isect x1 r
        return [r, x2, x12]

    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z, -- will be overridden
            hybivp_initialStateEnclosure = 
                Map.singleton modeFill1 initValues,
            hybivp_maybeExactStateAtTEnd =
                Nothing 
--                Just $
--                HybridSystemUncertainState 
--                {
--                    hybstate_modes = Set.fromList [modeFlow],
--                    hybstate_values = [0, 0, 0]
--                }
        }
    description =
        ""
        ++ "2T-S"
        ++ "\n ; x1(" ++ show tStart ++ ") = " ++ show initX1
        ++    ", x2(" ++ show tStart ++ ") = " ++ show initX2
        ++    ", x12(" ++ show tStart ++ ") = " ++ show initX12
    initValues@[initX1, initX2, initX12] = [toD 1,toD 1,toD 2] :: [Domain f]
    tStart = hybivp_tStart ivp
    z = toD 0
    toD = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn


ivp2BeadColumnEnergy :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivp2BeadColumnEnergy (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        (v <*> v <+> (toD 20) <*> x)
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x1","v1","r1", "x2","v2","r2"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove vars = 
        odeMoveAux vars1 ++ odeMoveAux vars2
        where
        (vars1, vars2) = splitAt 3 vars
    odeMoveAux [x,v,r] = [v, newConstFnFromSample x (toD $ -10), newConstFnFromSample r (toD 0)]
    odeMoveAux _ = error "internal error in odeMoveAux"
    invariantMove vars =
        do
        [x1,v1,r1] <- invariantMoveAux vars1
        -- x1 <= x2:
        x2Mx1NN <- makeNonneg $ x2 <-> x1 
        x1New <- isect x1 (x2 <-> x2Mx1NN)
        x2New <- isect x2 (x2Mx1NN <+> x1)
        let vars2 = [x2New, v2, r2]
        res2 <- invariantMoveAux vars2
        return $ [x1New,v1,r1] ++ res2 
        where
        (vars1, [x2,v2,r2]) = splitAt 3 vars
    invariantMoveAux [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- |v| = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let absV = ArithInOut.sqrtOut vSqr1
        vNew <- isect v ((neg absV) </\> absV)
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]

    eventSpecMap _mode =
        Map.fromList $
            [
                (eventBounce1,
                    (modeMove, resetBounce1, 
--                        [True, True, True, True, True, True], 
                        [True, True, True, False, False, False], 
                            pruneBounce1))
            ,
                (eventBounce2,
                    (modeMove, resetBounce2, [True, True, True, True, True, True], pruneBounce2))
            ]
    
    eventBounce1 = HybSysEventKind "bc1"
    pruneBounce1 _ [x1,v1,r1,x2,v2,r2] =
        do
        _ <- isect z x1
        v1NP <- makeNonpos v1
        return $ [z,v1NP,r1,x2,v2,r2]
    pruneBounce1 _ _ = error "internal error in pruneBounce1"
    resetBounce1 [x1,v1,r1,x2,v2,r2] = 
        [x1,
         (-0.5 :: Double) |<*> v1, 
         (toDInterval 0 0.25) <*> r1,
         x2, v2, r2
        ]
    resetBounce1 _ = error "internal error in resetBounce1"

    eventBounce2 = HybSysEventKind "bc2"
    pruneBounce2 _ [x1,v1,r1,x2,v2,r2] =
        do
        xNew <- isect x1 x2
        v2Mv1NP <- makeNonpos $ v2 <-> v1
        v1New <- isect v1 $ v2 <-> v2Mv1NP
        v2New <- isect v2 $ v2Mv1NP <+> v1
        return $ [xNew,v1New,r1,xNew,v2New,r2]
    pruneBounce2 _ _ = error "internal error in pruneBounce2"
    resetBounce2 [x1,v1,r1,x2,v2,r2] = 
        [x1,
         v2 <+> vDiffRest, -- v1+ := v2 + c(v1-v2)
         r2 <+> vDiffRest <*> (vDiffRest <+> ((2 :: Int) |<*> v2)),
            -- r1+ = (v1+)^2 + 20x 
            --     = (v2 + c(v1-v2))^2 + 20x 
            --     = v2^2 + 20x + 2*v2*c*(v1-v2) + (c(v1-v2))^2
            --     = r2 + 2*v2*c*(v1-v2)  + (c(v1-v2))^2
            --     = r2 + c(v1-v2)*(c(v1-v2)+2*v2)
         x2,
         v1 <-> vDiffRest, -- v2+ := v1 - c(v1-v2)
         r1 <+> vDiffRest <*> (vDiffRest <-> ((2 :: Int) |<*> v1))
            -- r2+ = (v2+)^2 + 20x 
            --     = (v1 - c(v1-v2))^2 + 20x 
            --     = v1^2 + 20x - 2*v1*c*(v1-v2)  + (c(v1-v2))^2
            --     = r1 - 2*v1*c*(v1-v2)  + (c(v1-v2))^2
            --     = r1 + c(v1-v2)*(c(v1-v2)-2*v1)
        ]
        where
        vDiff = (v1 <-> v2)
        vDiffRest = restCoeff |<*> vDiff
        restCoeff = 0.25 :: Double -- has to be > 0 and  < 0.5
    resetBounce2 _ = error "internal error in resetBounce2"

    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "2BC+"
        ++ "; x1(" ++ show tStart ++ ") = " ++ show initX1
        ++ ", v1(" ++ show tStart ++ ") = " ++ show initX1'
        ++ ", r1(" ++ show tStart ++ ") ∊ " ++ show initR1
    tStart = hybivp_tStart ivp

    initValues = [initX1, initX1', initR1, initX2, initX2', initR2]
    initX1 = toD 2
    initX1' = toD 0
    initR1 = energyWith initX1 initX1'
    initX2 = toD 4
    initX2' = toD 0
    initR2 = energyWith initX2 initX2'
    
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn

ivp2BeadColumnEnergyVDiff :: 
    (Var f ~ String,
     HasConstFns f,
     Neg f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedMultiply f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasConsistency (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show (Domain f)
    )
    => 
    f -> 
    HybridIVP f
ivp2BeadColumnEnergyVDiff (sampleFn :: f) =
    ivp
    where
    energyWith x v = 
        (v <*> v <+> (toD 20) <*> x)
    system =
        HybridSystem
        {
            hybsys_componentNames = ["x1","v1","r1", "x2","v2","r2","v1Mv2"],
            hybsys_modeFields = Map.fromList [(modeMove, odeMove)],
            hybsys_modeInvariants = Map.fromList [(modeMove, invariantMove)],
            hybsys_eventSpecification = eventSpecMap
        }
    modeMove = HybSysMode "move"
    odeMove :: [f] -> [f]
    odeMove vars = 
        odeMoveAux vars1 ++ odeMoveAux vars2 ++ [newConstFnFromSample v1Mv2 (toD 0)]
        where
        (vars2, [v1Mv2]) = splitAt 3 vars2vDiff
        (vars1, vars2vDiff) = splitAt 3 vars
    odeMoveAux [x,v,r] = 
        [v, 
         newConstFnFromSample x (toD $ -10), 
         newConstFnFromSample r (toD 0)]
    odeMoveAux _ = error "internal error in odeMoveAux"
    invariantMove vars =
        do
        [x1,v1,r1] <- invariantMoveAux vars1
        -- x1 <= x2:
        x2Mx1NN <- makeNonneg $ x2 <-> x1 
        x1New <- isect x1 (x2 <-> x2Mx1NN)
        x2New <- isect x2 (x2Mx1NN <+> x1)
        -- v2 = v1 - v1Mv2:
        v2New <- isect v2 (v1 <-> v1Mv2)
        v1Mv2New <- isect v1Mv2 $ v1 <-> v2New
        let vars2 = [x2New, v2New, r2]
        res2@[_, v2New2, _] <- invariantMoveAux vars2
        -- v1 = v1Mv2 + v2:        
        v1New <- isect v1 (v1Mv2New <+> v2New2)
        v1Mv2New2 <- isect v1Mv2New $ v1New <-> v2New2
        return $ [x1New,v1New,r1] ++ res2 ++ [v1Mv2New2] 
        where
        (vars1, [x2,v2,r2,v1Mv2]) = splitAt 3 vars
    invariantMoveAux [x,v,r] =
        do
        -- x >= 0:
        xNN <- makeNonneg x
        -- r >= 0:
        rNN <- makeNonneg r
        -- |v| = sqrt(r - 2gx):
        vSqr1 <- makeNonneg $ rNN <-> ((toD 20) <*> xNN)
        let absV = ArithInOut.sqrtOut vSqr1
        vNew <- isect v ((neg absV) </\> absV)
        -- x = (r - (v)^2) / 2g:
        vSqr2 <- makeNonneg $ v <*> v
        let x2 = (rNN <-> vSqr2) </> (toD 20)
        xNew <- isect xNN x2
        return [xNew, vNew, rNN]

    eventSpecMap _mode =
        Map.fromList $
            [
                (eventBounce1,
                    (modeMove, resetBounce1, 
--                        [True, True, True, True, True, True, True], 
                        [True, True, True, False, False, False, True], 
                            pruneBounce1))
            ,
                (eventBounce2,
                    (modeMove, resetBounce2, [True, True, True, True, True, True, True], pruneBounce2))
            ]
    
    eventBounce1 = HybSysEventKind "bc1"
    pruneBounce1 _ [x1,v1,r1,x2,v2,r2,v1Mv2] =
        do
        _ <- isect z x1
        v1NP <- makeNonpos v1
        return $ [z,v1NP,r1,x2,v2,r2,v1Mv2]
    pruneBounce1 _ _ = error "internal error in pruneBounce1"
    resetBounce1 [x1,v1,r1,x2,v2,r2,v1Mv2] = 
        [x1,
         (-0.5 :: Double) |<*> v1, 
         (toDInterval 0 0.25) <*> r1,
         x2, v2, r2,
         v1Mv2 <+> ((-1.5 :: Double) |<*> v1)
        ]
    resetBounce1 _ = error "internal error in resetBounce1"

    eventBounce2 = HybSysEventKind "bc2"
    pruneBounce2 _ [x1,v1,r1,x2,v2,r2,v1Mv2] =
        do
        xNew <- isect x1 x2
        -- v1 >= v2
        v1Mv2New <- isect v1Mv2 (v1 <-> v2)
        v1Mv2NN <- makeNonneg $ v1Mv2New
        v1New <- isect v1 $ v1Mv2NN <+> v2
        v2New <- isect v2 $ v1 <-> v1Mv2NN
        return $ [xNew,v1New,r1,xNew,v2New,r2,v1Mv2NN]
    pruneBounce2 _ _ = error "internal error in pruneBounce2"
    resetBounce2 [x1,v1,r1,x2,v2,r2,v1Mv2] = 
        [x1,
         v2 <+> vDiffRest, -- v1+ := v2 + c(v1-v2)
         r2 <+> vDiffRest <*> (vDiffRest <+> ((2 :: Int) |<*> v2)),
            -- r1+ = (v1+)^2 + 20x 
            --     = (v2 + c(v1-v2))^2 + 20x 
            --     = v2^2 + 20x + 2*v2*c*(v1-v2) + (c(v1-v2))^2
            --     = r2 + 2*v2*c*(v1-v2)  + (c(v1-v2))^2
            --     = r2 + c(v1-v2)*(c(v1-v2)+2*v2)
         x2,
         v1 <-> vDiffRest, -- v2+ := v1 - c(v1-v2)
         r1 <+> vDiffRest <*> (vDiffRest <-> ((2 :: Int) |<*> v1)),
            -- r2+ = (v2+)^2 + 20x 
            --     = (v1 - c(v1-v2))^2 + 20x 
            --     = v1^2 + 20x - 2*v1*c*(v1-v2)  + (c(v1-v2))^2
            --     = r1 - 2*v1*c*(v1-v2)  + (c(v1-v2))^2
            --     = r1 + c(v1-v2)*(c(v1-v2)-2*v1)
         (2*restCoeff - 1) |<*> v1Mv2
        ]
        where
        vDiff = v1Mv2
        vDiffRest = restCoeff |<*> vDiff
        restCoeff = 0.25 :: Double -- has to be > 0 and  < 0.5
    resetBounce2 _ = error "internal error in resetBounce2"

    ivp :: HybridIVP f
    ivp =
        HybridIVP
        {
            hybivp_description = description,
            hybivp_system = system,
            hybivp_tVar = "t",
            hybivp_tStart = z,
            hybivp_tEnd = z,
            hybivp_initialStateEnclosure = 
                Map.singleton modeMove initValues,
            hybivp_maybeExactStateAtTEnd = Nothing
        }
    description =
        "2BC+"
        ++ "; x1(" ++ show tStart ++ ") = " ++ show initX1
        ++ ", v1(" ++ show tStart ++ ") = " ++ show initX1'
        ++ ", r1(" ++ show tStart ++ ") ∊ " ++ show initR1
    tStart = hybivp_tStart ivp

    initValues = [initX1, initX1', initR1, initX2, initX2', initR2, initV1MV2]
    initX1 = toD 2
    initX1' = toD 0
    initR1 = energyWith initX1 initX1'
    initX2 = toD 4
    initX2' = toD 0
    initR2 = energyWith initX2 initX2'
    initV1MV2 = initX1' <-> initX2'
    
    z = toD 0
    toD = dblToReal sampleDom
    toDInterval l r = (toD l) </\> (toD r)
    sampleDom = getSampleDomValue sampleFn



makeNonneg ::
    (HasZero d, NumOrd.PartialComparison d, RefOrd.IntervalLike d, Show d) 
    => 
    d -> Maybe d
makeNonneg r
    | rangeContainsZero =
        Just $ RefOrd.fromEndpointsOut (z, rR)
    | alreadyNonneg = Just $ r
    | otherwise = Nothing
    where
    alreadyNonneg =
        (z <=? rL) == Just True
    rangeContainsZero =
        ((rL <=? z) == Just True)
        &&
        ((z <=? rR) == Just True)
    z = zero r
    (rL, rR) = RefOrd.getEndpointsOut r
    
makeNonpos ::
    (HasZero d, Neg d, NumOrd.PartialComparison d, RefOrd.IntervalLike d, Show d) 
    => 
    d -> Maybe d
makeNonpos r =
    do
    rN <- makeNonneg $ neg r
    return $ neg rN
    
isect ::
    (RefOrd.RoundedLattice d, HasConsistency d)
    =>
    d -> d -> Maybe d
isect x1 x2 =
    case isConsistentEff (consistencyDefaultEffort x1) meet of
        Just False -> Nothing
        _ -> Just meet
    where
    meet = x1 <\/> x2
    

      
