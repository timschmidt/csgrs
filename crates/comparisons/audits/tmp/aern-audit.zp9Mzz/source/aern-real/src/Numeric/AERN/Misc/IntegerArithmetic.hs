{-|
    Module      :  Numeric.AERN.Misc.IntegerArithmetic
    Description :  miscellaneous integer arithmetic functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Miscellaneous integer arithmetic functions.
-}

module Numeric.AERN.Misc.IntegerArithmetic where

intLogDown b n = fst $ intLog b n 
intLogUp b n = snd $ intLog b n 
    
intLog ::
    (Show n1, Num n1, Num n2, Ord n1, Integral n2) => 
    n1 {-^ base -} -> 
    n1 {-^ x -} -> 
    (n2, n2)
intLog b n
    | n == 1 = (0,0)
    | n > 1 && n < b = (0,1)
    | n >= b =
        bisect (lgDn, pwDn) (lgUp, pwUp)
    | otherwise = 
        error $ "Numeric.ER.Misc: intLog: illegal argument n = " ++ show n
    where
    ((lgDn, pwDn), (lgUp, pwUp)) = 
        findBounds (1, b) 
        -- lgDn <= log_b n < lgUp; pwDn = b^lgDn; pwUp = b^lgUp
    findBounds (lg, pw)
        | n < pwNext = ((lg, pw), (lgNext, pwNext))
        | otherwise = findBounds (lgNext, pwNext)
        where
        lgNext = 2 * lg
        pwNext = pw * pw
    bisect (lgDn, pwDn) (lgUp, pwUp)
        | pwDn == n = (lgDn, lgDn)
        | pwUp == n = (lgUp, lgUp)
        | lgDn == lgMid = (lgDn, lgUp)
        | lgUp == lgMid = (lgDn, lgUp)
        | n < pwMid =
            bisect (lgDn, pwDn) (lgMid, pwMid)
        | otherwise =
            bisect (lgMid, pwMid) (lgUp, pwUp)
        where
        lgMid = (lgDn + lgUp) `div` 2
        pwMid = pwDn * (b ^ (lgMid - lgDn))

binomial :: 
    (Integral n) =>
    n -> n -> n
binomial n p | p >= 0 =
    (product [(n-p+1)..n])
    `div`
    (product [1..p])
    
        