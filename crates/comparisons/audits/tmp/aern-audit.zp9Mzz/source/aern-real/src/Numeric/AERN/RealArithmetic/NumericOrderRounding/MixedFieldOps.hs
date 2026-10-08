{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.NumericOrderRounding.MixedFieldOps
    Description :  rounded basic arithmetic operations mixing 2 types
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations mixing 2 types.
    
    This module is hidden and reexported via its parent NumericOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.NumericOrderRounding.MixedFieldOps where

import Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
import Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion
import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.Effort
import Numeric.AERN.RealArithmetic.Laws 
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.NumericOrder as NumOrd

import Control.Exception
import Data.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

infixr 6 |+., |+^
infixl 6 +.|, +^|
infixr 7 |*., |*^
infixl 7 *.|, *^|
infixl 7 /.|, /^|

class
    (EffortIndicator (MixedAddEffortIndicator t tn))
    => 
    RoundedMixedAddEffort t tn 
    where
    type MixedAddEffortIndicator t tn
    mixedAddDefaultEffort :: t -> tn -> MixedAddEffortIndicator t tn

class (RoundedMixedAddEffort t tn) => RoundedMixedAdd t tn where
    mixedAddUpEff :: MixedAddEffortIndicator t tn -> t -> tn -> t
    mixedAddDnEff :: MixedAddEffortIndicator t tn -> t -> tn -> t

mixedAddUp :: (RoundedMixedAdd t tn) => t -> tn -> t
mixedAddUp a b = mixedAddUpEff (mixedAddDefaultEffort a b) a b

(+^|) :: (RoundedMixedAdd t tn) => t -> tn -> t
(+^|) = mixedAddUp
(|+^) :: (RoundedMixedAdd t tn) => tn -> t -> t
(|+^) = flip mixedAddUp

mixedAddDn :: (RoundedMixedAdd t tn) => t -> tn -> t
mixedAddDn a b = mixedAddDnEff (mixedAddDefaultEffort a b) a b

(+.|) :: (RoundedMixedAdd t tn) => t -> tn -> t
(+.|) = mixedAddDn
(|+.) :: (RoundedMixedAdd t tn) => tn -> t -> t
(|+.) = flip mixedAddDn

--{- 
--    The following would lead to overlapping instances, eg
--    with "e (Interval e)" or "(Interval (IntPoly String (Interval e)) (Interval e)".
---} 
--
--instance (RoundedAddEffort t) => (RoundedMixedAddEffort t t)
--    where
--    type MixedAddEffortIndicator t t = AddEffortIndicator t
--    mixedAddDefaultEffort _ sample = addDefaultEffort sample 
--instance (RoundedAdd t) => (RoundedMixedAdd t t)
--    where
--    mixedAddDnEff = addDnEff
--    mixedAddUpEff = addUpEff


{- tools to easily make a RoundedMixedAdd instance 
   via the composition of conversion and homogeneous addition -}

type MixedAddEffortIndicatorByConversion t tn =
        (AddEffortIndicator t, ConvertEffortIndicator tn t)

mixedAddDefaultEffortByConversion d n = 
        (addDefaultEffort d, convertDefaultEffort n d)

mixedAddUpEffByConversion ::
    (Convertible tn t, RoundedAdd t, Show tn) =>
    (AddEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedAddUpEffByConversion (effAdd, effConv) d n = 
    addUpEff effAdd nUp d
    where
    nUp = 
        case convertUpEff effConv sample n of
            (Just nUp) -> nUp
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed addition: n = " ++ show n
    sample = d

mixedAddDnEffByConversion ::
    (Convertible tn t, RoundedAdd t, Show tn) =>
    (AddEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedAddDnEffByConversion (effAdd, effConv) d n = 
    addDnEff effAdd nDn d
    where
    nDn = 
        case convertDnEff effConv sample n of
            (Just nDn) -> nDn
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed addition: n = " ++ show n
    sample = d

{- properties of mixed addition -}

propMixedAddEqualsConvert ::
    (NumOrd.PartialComparison t, Convertible tn t,
     RoundedMixedAdd t tn, RoundedAdd t,
     Show t, HasLegalValues t) 
    =>
    t -> tn ->
    (NumOrd.PartialCompareEffortIndicator t,
     (MixedAddEffortIndicator t tn,      
      AddEffortIndicator t,
      ConvertEffortIndicator tn t)) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    tn -> 
    Bool
propMixedAddEqualsConvert sample sampleN initEffort 
        (NumOrd.UniformlyOrderedSingleton d) n =
    equalRoundingUpDn "mixed addition by conversion"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMAdd,_,_) =
        let (+^|) = mixedAddUpEff effMAdd in d +^| n
    expr1Dn (effMAdd,_,_) =
        let (+.|) = mixedAddDnEff effMAdd in d +.| n
    expr2Up (_,effAdd,effConv) =
        let (+^) = addUpEff effAdd in d +^ nUp
        where
        Just nUp = convertUpEff effConv sample n
    expr2Dn (_,effAdd,effConv) =
        let (+.) = addDnEff effAdd in d +. nDn
        where
        Just nDn = convertDnEff effConv sample n

class
    (EffortIndicator (MixedMultEffortIndicator t tn))
    => 
    RoundedMixedMultiplyEffort t tn 
    where
    type MixedMultEffortIndicator t tn
    mixedMultDefaultEffort :: t -> tn -> MixedMultEffortIndicator t tn

class (RoundedMixedMultiplyEffort t tn) => RoundedMixedMultiply t tn where
    mixedMultUpEff :: MixedMultEffortIndicator t tn -> t -> tn -> t
    mixedMultDnEff :: MixedMultEffortIndicator t tn -> t -> tn -> t


mixedMultUp :: (RoundedMixedMultiply t tn) => t -> tn -> t
mixedMultUp a b = mixedMultUpEff (mixedMultDefaultEffort a b) a b

(*^|) :: (RoundedMixedMultiply t tn) => t -> tn -> t
(*^|) = mixedMultUp
(|*^) :: (RoundedMixedMultiply t tn) => tn -> t -> t
(|*^) = flip mixedMultUp

mixedMultDn :: (RoundedMixedMultiply t tn) => t -> tn -> t
mixedMultDn a b = mixedMultDnEff (mixedMultDefaultEffort a b) a b

(*.|) :: (RoundedMixedMultiply t tn) => t -> tn -> t
(*.|) = mixedMultDn
(|*.) :: (RoundedMixedMultiply t tn) => tn -> t -> t
(|*.) = flip mixedMultDn

--{- 
--    The following would lead to overlapping instances, eg
--    with "e (Interval e)" or "(Interval (IntPoly String (Interval e)) (Interval e)".
---} 
--
--instance (RoundedMultiplyEffort t) => (RoundedMixedMultiplyEffort t t)
--    where
--    type MixedMultEffortIndicator t t = MultEffortIndicator t
--    mixedMultDefaultEffort _ sample = multDefaultEffort sample 
--instance (RoundedMultiply t) => (RoundedMixedMultiply t t)
--    where
--    mixedMultDnEff = multDnEff
--    mixedMultUpEff = multUpEff


{- tools to easily make a RoundedMixedMultiply instance 
   via the composition of conversion and homogeneous addition -}

type MixedMultEffortIndicatorByConversion t tn =
        (MultEffortIndicator t, 
         ConvertEffortIndicator tn t,
         NumOrd.MinmaxEffortIndicator t)

mixedMultDefaultEffortByConversion d n = 
        (addDefaultEffort d, 
         convertDefaultEffort n d,
         NumOrd.minmaxDefaultEffort d)

mixedMultUpEffByConversion ::
    (Convertible tn t, RoundedMultiply t, NumOrd.RoundedLattice t, Show tn) =>
    (MultEffortIndicator t, 
     ConvertEffortIndicator tn t,
     NumOrd.MinmaxEffortIndicator t) ->
    t -> tn -> t
mixedMultUpEffByConversion (effMult, effConv, effMinmax) d n =
    NumOrd.maxUpEff effMinmax
    (multUpEff effMult d nDn)
    (multUpEff effMult d nUp)
    where
    (nUp, nDn) = 
        case (convertUpEff effConv sample n, convertDnEff effConv sample n) of
            (Just nUp, Just nDn) -> (nUp, nDn)
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed multiplication: n = " ++ show n
    sample = d

mixedMultDnEffByConversion ::
    (Convertible tn t, RoundedMultiply t, NumOrd.RoundedLattice t, Show tn) =>
    (MultEffortIndicator t, 
     ConvertEffortIndicator tn t,
     NumOrd.MinmaxEffortIndicator t) ->
    t -> tn -> t
mixedMultDnEffByConversion (effMult, effConv, effMinmax) d n =
    NumOrd.minDnEff effMinmax
    (multDnEff effMult d nDn)
    (multDnEff effMult d nUp)
    where
    (nUp, nDn) = 
        case (convertUpEff effConv sample n, convertDnEff effConv sample n) of
            (Just nUp, Just nDn) -> (nUp, nDn)
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed multiplication: n = " ++ show n
    sample = d


{- properties of mixed multiplication -}

propMixedMultEqualsConvert ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t, 
     Convertible tn t,
     RoundedMixedMultiply t tn, RoundedMultiply t,
     Show t, HasLegalValues t) 
    =>
    t -> tn ->
    (NumOrd.PartialCompareEffortIndicator t,
     (MixedMultEffortIndicator t tn,      
      (MultEffortIndicator t,
       ConvertEffortIndicator tn t,
       NumOrd.MinmaxEffortIndicator t))) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    tn -> Bool
propMixedMultEqualsConvert sample _sampleN initEffort 
        (NumOrd.UniformlyOrderedSingleton d) n =
    equalRoundingUpDn "mixed multiplication by conversion"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMMult,_) =
        let (*^|) = mixedMultUpEff effMMult in d *^| n
    expr1Dn (effMMult,_) =
        let (*.|) = mixedMultDnEff effMMult in d *.| n
    expr2Up (_,(effMult,effConv,effMinmax)) =
        let (*^) = multUpEff effMult in
        NumOrd.maxUpEff effMinmax  
            (d *^ (fromJust $ convertUpEff effConv sample n))
            (d *^ (fromJust $ convertDnEff effConv sample n))
    expr2Dn (_,(effMult,effConv,effMinmax)) =
        let (*.) = multDnEff effMult in
        NumOrd.minDnEff effMinmax  
            (d *. (fromJust $ convertUpEff effConv sample n))
            (d *. (fromJust $ convertDnEff effConv sample n))

class
    (EffortIndicator (MixedDivEffortIndicator t tn))
    =>
    RoundedMixedDivideEffort t tn 
    where
    type MixedDivEffortIndicator t tn
    mixedDivDefaultEffort :: t -> tn -> MixedDivEffortIndicator t tn

class (RoundedMixedDivideEffort t tn) => RoundedMixedDivide t tn where
    mixedDivUpEff :: MixedDivEffortIndicator t tn -> t -> tn -> t
    mixedDivDnEff :: MixedDivEffortIndicator t tn -> t -> tn -> t

mixedDivUp :: (RoundedMixedDivide t tn) => t -> tn -> t
mixedDivUp a b = mixedDivUpEff (mixedDivDefaultEffort a b) a b

(/^|) :: (RoundedMixedDivide t tn) => t -> tn -> t
(/^|) = mixedDivUp

mixedDivDn :: (RoundedMixedDivide t tn) => t -> tn -> t
mixedDivDn a b = mixedDivDnEff (mixedDivDefaultEffort a b) a b

(/.|) :: (RoundedMixedDivide t tn) => t -> tn -> t
(/.|) = mixedDivDn

--{- 
--    The following would lead to overlapping instances, eg
--    with "e (Interval e)" or "(Interval (IntPoly String (Interval e)) (Interval e)".
---} 
--
--instance (RoundedDivideEffort t) => (RoundedMixedDivideEffort t t)
--    where
--    type MixedDivEffortIndicator t t = DivEffortIndicator t
--    mixedDivDefaultEffort _ sample = divDefaultEffort sample 
--instance (RoundedDivide t) => (RoundedMixedDivide t t)
--    where
--    mixedDivDnEff = divDnEff
--    mixedDivUpEff = divUpEff


{- tools to easily make a RoundedMixedDivide instance 
   via the composition of conversion and homogeneous addition -}

type MixedDivEffortIndicatorByConversion t tn =
        (DivEffortIndicator t, 
         ConvertEffortIndicator tn t,
         (NumOrd.MinmaxEffortIndicator t,
          NumOrd.PartialCompareEffortIndicator t))

mixedDivDefaultEffortByConversion d n = 
        (addDefaultEffort d, 
         convertDefaultEffort n d,
         (NumOrd.minmaxDefaultEffort d,
          NumOrd.pCompareDefaultEffort d))

mixedDivUpEffByConversion ::
    (Convertible tn t, 
     RoundedDivide t, 
     HasZero t,  HasInfinities t,
     NumOrd.PartialComparison t,
     NumOrd.RoundedLattice t,
     Show tn) =>
    (DivEffortIndicator t, 
     ConvertEffortIndicator tn t,
     (NumOrd.MinmaxEffortIndicator t, 
      NumOrd.PartialCompareEffortIndicator t)) ->
    t -> tn -> t
mixedDivUpEffByConversion (effDiv, effConv, (effMinmax, effComp)) d n =
    let (<=?) = NumOrd.pLeqEff effComp in
    case ((zero d) <=? nDn, nUp <=? (zero d)) of
        (Just True, _) -> normalResult 
        (_, Just True) -> normalResult
        _ -> plusInfinity d -- b is too close to zero
    where
    normalResult =
        NumOrd.maxDnEff effMinmax  -- we do not know the sign of a
            (divUpEff effDiv d nDn)
            (divUpEff effDiv d nUp)
    (nUp, nDn) = 
        case (convertUpEff effConv sample n, convertDnEff effConv sample n) of
            (Just nUp, Just nDn) -> (nUp, nDn)
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed division: n = " ++ show n
    sample = d
    
mixedDivDnEffByConversion ::
    (Convertible tn t, 
     RoundedDivide t, 
     HasZero t,  HasInfinities t,
     NumOrd.PartialComparison t,
     NumOrd.RoundedLattice t,
     Show tn) =>
    (DivEffortIndicator t, 
     ConvertEffortIndicator tn t,
     (NumOrd.MinmaxEffortIndicator t, 
      NumOrd.PartialCompareEffortIndicator t)) ->
    t -> tn -> t
mixedDivDnEffByConversion (effDiv, effConv, (effMinmax, effComp)) d n = 
    let (<=?) = NumOrd.pLeqEff effComp in
    case ((zero d) <=? nDn, nUp <=? (zero d)) of
        (Just True, _) -> normalResult 
        (_, Just True) -> normalResult
        _ -> minusInfinity d -- b is too close to zero
    where
    normalResult =
        NumOrd.minDnEff effMinmax  -- we do not know the sign of a
            (divDnEff effDiv d nDn)
            (divDnEff effDiv d nUp)
    (nUp, nDn) = 
        case (convertUpEff effConv sample n, convertDnEff effConv sample n) of
            (Just nUp, Just nDn) -> (nUp, nDn)
            _ -> throw $ AERNException $ 
                        "conversion failed during mixed division: n = " ++ show n
    sample = d

{- properties of mixed multiplication -}

propMixedDivEqualsConvert ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t, 
     Convertible tn t,
     RoundedMixedDivide t tn, RoundedDivide t,
     Show t, HasLegalValues t,
     HasZero t) 
    =>
    t -> tn ->
    (NumOrd.PartialCompareEffortIndicator t,
     (MixedDivEffortIndicator t tn,      
      (DivEffortIndicator t,
       ConvertEffortIndicator tn t,
       NumOrd.MinmaxEffortIndicator t))) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    tn -> Bool
propMixedDivEqualsConvert sample _sampleN initEffort@(effComp,(_,(_,effConv,_))) 
        (NumOrd.UniformlyOrderedSingleton d) n
    =
    equalRoundingUpDn "mixed division by conversion"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMDiv,_) =
        let (/^|) = mixedDivUpEff effMDiv in d /^| n
    expr1Dn (effMDiv,_) =
        let (/.|) = mixedDivDnEff effMDiv in d /.| n
    expr2Up (_,(effDiv,effConv,effMinmax)) =
        let (/^) = divUpEff effDiv in
        NumOrd.maxUpEff effMinmax  
            (d /^ (fromJust $ convertUpEff effConv sample n))
            (d /^ (fromJust $ convertDnEff effConv sample n))
    expr2Dn (_,(effDiv,effConv,effMinmax)) =
        let (/.) = divDnEff effDiv in
        NumOrd.minDnEff effMinmax  
            (d /. (fromJust $ convertUpEff effConv sample n))
            (d /. (fromJust $ convertDnEff effConv sample n))
    
testsUpDnMixedFieldOps (name, sample) (nameN, sampleN) =
    testGroup (name ++ " with " ++ nameN ++ ": mixed up/dn rounded ops") $
        [
            testProperty "addition" (propMixedAddEqualsConvert sample sampleN)
        ,
            testProperty "multiplication" (propMixedMultEqualsConvert sample sampleN)
        ,
            testProperty "division" (propMixedDivEqualsConvert sample sampleN)
        ]

class 
    (RoundedMixedAddEffort t tn, RoundedMixedMultiplyEffort t tn,
     EffortIndicator (MixedRingOpsEffortIndicator t tn)) 
    => 
    RoundedMixedRingEffort t tn
    where
    type MixedRingOpsEffortIndicator t tn
    mixedRingOpsDefaultEffort :: t -> tn -> MixedRingOpsEffortIndicator t tn
    mxringEffortAdd :: t -> tn -> MixedRingOpsEffortIndicator t tn -> MixedAddEffortIndicator t tn
    mxringEffortMult :: t -> tn -> MixedRingOpsEffortIndicator t tn -> MixedMultEffortIndicator t tn

class (RoundedMixedAdd t tn, RoundedMixedMultiply t tn, RoundedMixedRingEffort t tn) => 
    RoundedMixedRing t tn

--{- 
--    The following would lead to overlapping instances, eg
--    with "e (Interval e)" or "(Interval (IntPoly String (Interval e)) (Interval e)".
---} 
--
--instance (RoundedRingEffort t) => (RoundedMixedRingEffort t t)
--    where
--    type MixedRingOpsEffortIndicator t t = RingOpsEffortIndicator t
--    mixedRingOpsDefaultEffort _ sample = ringOpsDefaultEffort sample
--    mxringEffortAdd sample _ eff = ringEffortAdd sample eff
--    mxringEffortMult sample _ eff = ringEffortMult sample eff
--
--instance (RoundedRing t) => (RoundedMixedRing t t)


class 
    (RoundedMixedRingEffort t tn, RoundedMixedDivideEffort t tn,
     EffortIndicator (MixedFieldOpsEffortIndicator t tn)) 
    => 
    RoundedMixedFieldEffort t tn
    where
    type MixedFieldOpsEffortIndicator t tn
    mixedFieldOpsDefaultEffort :: t -> tn -> MixedFieldOpsEffortIndicator t tn
    mxfldEffortAdd :: t -> tn -> MixedFieldOpsEffortIndicator t tn -> MixedAddEffortIndicator t tn
    mxfldEffortMult :: t -> tn -> MixedFieldOpsEffortIndicator t tn -> MixedMultEffortIndicator t tn
    mxfldEffortDiv :: t -> tn -> MixedFieldOpsEffortIndicator t tn -> MixedDivEffortIndicator t tn

class (RoundedMixedRing t tn, RoundedMixedDivide t tn, RoundedMixedFieldEffort t tn) => 
    RoundedMixedField t tn
    
--{- 
--    The following would lead to overlapping instances, eg
--    with "e (Interval e)" or "(Interval (IntPoly String (Interval e)) (Interval e)".
---} 
--    
--instance (RoundedFieldEffort t) => (RoundedMixedFieldEffort t t)
--    where
--    type MixedFieldOpsEffortIndicator t t = FieldOpsEffortIndicator t
--    mixedFieldOpsDefaultEffort _ sample = fieldOpsDefaultEffort sample
--    mxfldEffortAdd sample _ eff = fldEffortAdd sample eff
--    mxfldEffortMult sample _ eff = fldEffortMult sample eff
--    mxfldEffortDiv sample _ eff = fldEffortDiv sample eff
--
--instance (RoundedField t) => (RoundedMixedField t t)
    