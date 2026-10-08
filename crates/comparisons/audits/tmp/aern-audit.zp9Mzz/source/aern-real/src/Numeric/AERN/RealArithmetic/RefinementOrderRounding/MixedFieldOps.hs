{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RefinementOrderRounding.MixedFieldOps
    Description :  rounded basic arithmetic operations mixing 2 types
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations mixing 2 types.
    
    This module is hidden and reexported via its parent RefinementOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.RefinementOrderRounding.MixedFieldOps where

import Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion
import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception (HasLegalValues)
import Numeric.AERN.RealArithmetic.Laws
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

infixr 6 |<+>, |>+<, |+
infixl 6 <+>|, >+<|, +|
infixr 7 |<*>, |>*<, |*
infixl 7 <*>|, >*<|, *|
infixl 7 </>|, >/<|, /|

(|+) :: (RoundedMixedAdd t tn) => tn -> t -> t
(|+) = (|<+>)
(+|) :: (RoundedMixedAdd t tn) => t -> tn -> t
(+|) = (<+>|)
(|*) :: (RoundedMixedMultiply t tn) => tn -> t -> t
(|*) = (|<*>)
(*|) :: (RoundedMixedMultiply t tn) => t -> tn -> t
(*|) = (<*>|)
(/|) :: (RoundedMixedDivide t tn) => t -> tn -> t
(/|) = (</>|)

class
    (EffortIndicator (MixedAddEffortIndicator t tn))
    => 
    RoundedMixedAddEffort t tn 
    where
    type MixedAddEffortIndicator t tn
    mixedAddDefaultEffort :: t -> tn -> MixedAddEffortIndicator t tn

class (RoundedMixedAddEffort t tn) => RoundedMixedAdd t tn where
    mixedAddInEff :: MixedAddEffortIndicator t tn -> t -> tn -> t
    mixedAddOutEff :: MixedAddEffortIndicator t tn -> t -> tn -> t

-- | Inward rounded additive scalar left action with default effort
mixedAddIn :: (RoundedMixedAdd t tn) => t -> tn -> t
mixedAddIn a b = mixedAddInEff (mixedAddDefaultEffort a b) a b

-- | Inward rounded additive scalar left action with default effort
(|>+<) :: (RoundedMixedAdd t tn) => tn -> t -> t
(|>+<) = flip mixedAddIn

-- | Inward rounded additive scalar right action with default effort
(>+<|) :: (RoundedMixedAdd t tn) => t -> tn -> t
(>+<|) = mixedAddIn

-- | Outward rounded additive scalar left action with default effort
mixedAddOut :: (RoundedMixedAdd t tn) => t -> tn -> t
mixedAddOut a b = mixedAddOutEff (mixedAddDefaultEffort a b) a b

-- | Outward rounded additive scalar left action with default effort
(|<+>) :: (RoundedMixedAdd t tn) => tn -> t -> t
(|<+>) = flip mixedAddOut

-- | Outward rounded additive scalar right action with default effort
(<+>|) :: (RoundedMixedAdd t tn) => t -> tn -> t
(<+>|) = mixedAddOut

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
--    mixedAddOutEff = addOutEff
--    mixedAddInEff = addInEff

mixedAddDefaultEffortByConversion d n = 
        (addDefaultEffort d, convertDefaultEffort n d)

mixedAddInEffByConversion ::
    (Convertible tn t, RoundedAdd t) =>
    (AddEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedAddInEffByConversion (effAdd, effConv) d n = 
    addInEff effAdd d (convertInEff effConv sample n)
    where
    sample = d

mixedAddOutEffByConversion ::
    (Convertible tn t, RoundedAdd t) =>
    (AddEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedAddOutEffByConversion (effAdd, effConv) d n = 
    addOutEff effAdd d (convertOutEff effConv sample n)
    where
    sample = d


propMixedAddEqualsConvert ::
    (RefOrd.PartialComparison t, Convertible tn t,
     RoundedMixedAdd t tn, RoundedAdd t,
     Show t, HasLegalValues t) 
    =>
    t -> tn ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t,
     (MixedAddEffortIndicator t tn,      
      AddEffortIndicator t,
      ConvertEffortIndicator tn t)) -> 
    tn -> Bool
propMixedAddEqualsConvert sample _sampleN 
        (RefOrd.UniformlyOrderedSingleton d) 
        initEffort 
        n 
    =
    equalRoundingUpDn "mixed addition by conversion"
        expr1In expr1Out expr2In expr2Out
        RefOrd.pLeqEff initEffort
    where
    expr1In (effMAdd,_,_) =
        let (>+<|) = mixedAddInEff effMAdd in d >+<| n
    expr1Out (effMAdd,_,_) =
        let (<+>|) = mixedAddOutEff effMAdd in d <+>| n
    expr2In (_,effAdd,effConv) =
        let (>+<) = addInEff effAdd in  d >+< (convertInEff effConv sample n)
    expr2Out (_,effAdd,effConv) =
        let (<+>) = addOutEff effAdd in  d <+> (convertOutEff effConv sample n)


class
    (EffortIndicator (MixedMultEffortIndicator t tn))
    =>
    RoundedMixedMultiplyEffort t tn 
    where
    type MixedMultEffortIndicator t tn
    mixedMultDefaultEffort :: t -> tn -> MixedMultEffortIndicator t tn

class (RoundedMixedMultiplyEffort t tn) =>  RoundedMixedMultiply t tn where
    mixedMultInEff :: MixedMultEffortIndicator t tn -> t -> tn -> t
    mixedMultOutEff :: MixedMultEffortIndicator t tn -> t -> tn -> t


-- | Inward rounded scaling left action with default effort
mixedMultIn :: (RoundedMixedMultiply t tn) => t -> tn -> t
mixedMultIn a b = mixedMultInEff (mixedMultDefaultEffort a b) a b

-- | Inward rounded scaling left action with default effort
(|>*<) :: (RoundedMixedMultiply t tn) => tn -> t -> t
(|>*<) = flip mixedMultIn

-- | Inward rounded scaling right action with default effort
(>*<|) :: (RoundedMixedMultiply t tn) => t -> tn -> t
(>*<|) = mixedMultIn

-- | Outward rounded scaling left action with default effort
mixedMultOut :: (RoundedMixedMultiply t tn) => t -> tn -> t
mixedMultOut a b = mixedMultOutEff (mixedMultDefaultEffort a b) a b

-- | Outward rounded scaling left action with default effort
(|<*>) :: (RoundedMixedMultiply t tn) => tn -> t -> t
(|<*>) = flip mixedMultOut

-- | Outward rounded scaling right action with default effort
(<*>|) :: (RoundedMixedMultiply t tn) => t -> tn -> t
(<*>|) = mixedMultOut

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
--    mixedMultOutEff = multOutEff
--    mixedMultInEff = multInEff

mixedMultDefaultEffortByConversion d n = 
        (multDefaultEffort d, convertDefaultEffort n d)

mixedMultInEffByConversion ::
    (Convertible tn t, RoundedMultiply t) =>
    (MultEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedMultInEffByConversion (effMult, effConv) d n = 
    multInEff effMult d (convertInEff effConv sample n)
    where
    sample = d

mixedMultOutEffByConversion ::
    (Convertible tn t, RoundedMultiply t) =>
    (MultEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedMultOutEffByConversion (effMult, effConv) d n = 
    multOutEff effMult d (convertOutEff effConv sample n)
    where
    sample = d


propMixedMultEqualsConvert ::
    (RefOrd.PartialComparison t, Convertible tn t,
     RoundedMixedMultiply t tn, RoundedMultiply t,
     Show t, HasLegalValues t) 
    =>
    t -> tn ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t,
      (MixedMultEffortIndicator t tn,      
       MultEffortIndicator t,
       ConvertEffortIndicator tn t)) -> 
    tn -> Bool
propMixedMultEqualsConvert sample _sampleN 
        (RefOrd.UniformlyOrderedSingleton d) 
        initEffort 
        n 
    =
    equalRoundingUpDn "mixed multiplication by conversion"
        expr1In expr1Out expr2In expr2Out
        RefOrd.pLeqEff initEffort
    where
    expr1In (effMMult,_,_) =
        let (>*<|) = mixedMultInEff effMMult in d >*<| n
    expr1Out (effMMult,_,_) =
        let (<*>|) = mixedMultOutEff effMMult in d <*>| n
    expr2In (_,effMult,effConv) =
        let (>*<) = multInEff effMult in d >*< (convertInEff effConv sample n)
    expr2Out (_,effMult,effConv) =
        let (<*>) = multOutEff effMult in d <*> (convertOutEff effConv sample n)

class
    (EffortIndicator (MixedDivEffortIndicator t tn))
    => 
    RoundedMixedDivideEffort t tn 
    where
    type MixedDivEffortIndicator t tn
    mixedDivDefaultEffort :: t -> tn -> MixedDivEffortIndicator t tn

class (RoundedMixedDivideEffort t tn) => RoundedMixedDivide t tn where
    mixedDivInEff :: MixedDivEffortIndicator t tn -> t -> tn -> t
    mixedDivOutEff :: MixedDivEffortIndicator t tn -> t -> tn -> t

-- | Inward rounded reciprocal scaling right action with default effort
mixedDivIn :: (RoundedMixedDivide t tn) => t -> tn -> t
mixedDivIn a b = mixedDivInEff (mixedDivDefaultEffort a b) a b

-- | Inward rounded reciprocal scaling right action with default effort
(>/<|) :: (RoundedMixedDivide t tn) => t -> tn -> t
(>/<|) = mixedDivIn

-- | Outward rounded reciprocal scaling right action with default effort
mixedDivOut :: (RoundedMixedDivide t tn) => t -> tn -> t
mixedDivOut a b = mixedDivOutEff (mixedDivDefaultEffort a b) a b

-- | Outward rounded reciprocal scaling right action with default effort
(</>|) :: (RoundedMixedDivide t tn) => t -> tn -> t
(</>|) = mixedDivOut

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
--    mixedDivOutEff = divOutEff
--    mixedDivInEff = divInEff

mixedDivDefaultEffortByConversion d n = 
        (divDefaultEffort d, convertDefaultEffort n d)

mixedDivInEffByConversion ::
    (Convertible tn t, RoundedDivide t) =>
    (DivEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedDivInEffByConversion (effDiv, effConv) d n = 
    divInEff effDiv d (convertInEff effConv sample n)
    where
    sample = d

mixedDivOutEffByConversion ::
    (Convertible tn t, RoundedDivide t) =>
    (DivEffortIndicator t, ConvertEffortIndicator tn t) ->
    t -> tn -> t
mixedDivOutEffByConversion (effDiv, effConv) d n = 
    divOutEff effDiv d (convertOutEff effConv sample n)
    where
    sample = d


propMixedDivEqualsConvert ::
    (RefOrd.PartialComparison t, Convertible tn t,
     RoundedMixedDivide t tn, RoundedDivide t,
     Show t, HasZero t, HasLegalValues t) 
    =>
    t -> tn ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t,
     (MixedDivEffortIndicator t tn,      
      DivEffortIndicator t,
      ConvertEffortIndicator tn t)) -> 
    tn -> Bool
propMixedDivEqualsConvert sample _sampleN 
        (RefOrd.UniformlyOrderedSingleton d) 
        initEffort@(effComp,(_,_,effConv)) 
        n
    =
    equalRoundingUpDn "mixed division by conversion"
        expr1In expr1Out expr2In expr2Out
        RefOrd.pLeqEff initEffort
    where
    expr1In (effMDiv,_,_) =
        let (>/<|) = mixedDivInEff effMDiv in d >/<| n
    expr1Out (effMDiv,_,_) =
        let (</>|) = mixedDivOutEff effMDiv in d </>| n
    expr2In (_,effDiv,effConv) =
        let (>/<) = divInEff effDiv in d >/< (convertInEff effConv sample n)
    expr2Out (_,effDiv,effConv) =
        let (</>) = divOutEff effDiv in d </> (convertOutEff effConv sample n)

    
testsInOutMixedFieldOps (name, sample) (nameN, sampleN) area =
    testGroup (name ++ " with " ++ nameN ++ ": mixed in/out rounded ops") $
        [
            testProperty "addition" (area, propMixedAddEqualsConvert sample sampleN)
        ,
            testProperty "multiplication" (area, propMixedMultEqualsConvert sample sampleN)
        ,
            testProperty "division" (area, propMixedDivEqualsConvert sample sampleN)
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
        