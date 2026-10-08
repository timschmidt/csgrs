{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE DefaultSignatures, FlexibleInstances, UndecidableInstances #-}
{-# LANGUAGE IncoherentInstances #-}
module AuditFnRepComparison where

import FunctionAbstraction

import Numeric.AERN.MPFRBasis.Interval
    
main :: IO ()
main =
    do
    putStrLn "runFnRepComparison not implemented yet"
        
zeroUsingTrisection ::
    (RF fn)
    => 
    fn ->
    Rational ->
    Rational ->
    R
zeroUsingTrisection fn lOrig rOrig n =
    trisect lOrig rOrig initialPrecision
    where
    initialPrecision = 100
    trisect l r prec
        | resPreciseEnough = res
        | sign1 * sign2 < 0 = trisect m1 m2 prec 
        | sign0 * sign1 < 0 = trisect l m1 prec 
        | sign0 * sign2 < 0 = trisect l m2 prec 
        | sign1 * sign3 < 0 = trisect m1 r prec 
        | sign2 * sign3 < 0 = trisect m2 r prec 
        | otherwise = trisect l r (2*prec)
        where
        res = p0 <\/> p3
        resPreciseEnough = (width res <=? 2^^(-n)) == Just True 
        p0 = fromRationalWithPrec prec l
        p1 = fromRationalWithPrec prec m1
        p2 = fromRationalWithPrec prec m2
        p3 = fromRationalWithPrec prec r
        m1 = (l * 0.25) + (r * 0.75)
        m2 = (l * 0.75) + (r * 0.25)
        [value0, value1, value2, value3] = map (evalMI fn) [p0,p1,p2,p3]
        [sign0, sign1, sign2, sign3] = map getPseudoSign [value0,value1,value2,value3]
            where
            getPseudoSign :: MI -> Int
            getPseudoSign v =
                case (v <? 0, 0 <? v) of -- TODO fix MI comparison to allow mixed precisions
                    (Just True, _) -> 1
                    (_, Just True) -> -1
                    _ -> 0
    


--preciseNearby :: MI -> MI
--preciseNearby mi =
--    fst $ getEndpoints mi
