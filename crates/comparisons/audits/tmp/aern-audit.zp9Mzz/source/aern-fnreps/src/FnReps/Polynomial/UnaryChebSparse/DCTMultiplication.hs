{-|
    This module implements the fast polynomial multiplication algorithm presented in:
    [BT97] G. Baszenski and M. Tasche, 
    “Fast polynomial multiplication and convolutions related to the discrete cosine transform,” 
    Linear Algebra Appl., vol. 252, no. 1-3, pp. 1–25, Feb. 1997
    http://dx.doi.org/10.1016/0024-3795(95)00696-6
        
    

-}
module FnReps.Polynomial.UnaryChebSparse.DCTMultiplication 
--(multiplyDCT_terms)
where

--import Numeric.AERN.MPFRBasis.Interval
import AuditDCTCompat (DI, piOut, (/|), (|*))

import Math.NumberTheory.Logarithms (intLog2)

import qualified Data.HashMap.Strict as HM
import Data.List (sortBy)

import AuditDCTTrace (trace)

--type RA = MI
type RA = DI

testDirect =
    multiplyDirect_terms p1 p2
    
testDCT =
    multiplyDCT_terms p1 p2
    
p1 :: HM.HashMap Int RA
p1 = HM.fromList [(i,1) | i <- [0..500]]

p2 :: HM.HashMap Int RA
p2 = HM.fromList [(i,2) | i <- [0..500]]

multiplyDirect_terms
     :: HM.HashMap Int RA -> HM.HashMap Int RA -> HM.HashMap Int RA
multiplyDirect_terms terms1 terms2 =
    terms
    where
    terms =
        foldl addTerm HM.empty newTerms
        where
        addTerm prevTerms (i,a) = 
            HM.insertWith (+) i a prevTerms 
        newTerms =
            concat
            [   let c = a*b/2 in [(i+j, c), (abs (i-j), c)]
                | 
                (i,a) <- HM.toList terms1,
                (j,b) <- HM.toList terms2
            ]
            
multiplyDCT_terms :: HM.HashMap Int RA -> HM.HashMap Int RA -> HM.HashMap Int RA
multiplyDCT_terms termsA termsB =
    trace
    (
        "multiplyDCT_terms:"
        ++ "\n dA = " ++ show dA
        ++ "\n dB = " ++ show dB
        ++ "\n cN = " ++ show cN
        ++ "\n a = " ++ show a
        ++ "\n b = " ++ show b
        ++ "\n aT = " ++ show aT
        ++ "\n bT = " ++ show bT
        ++ "\n cT = " ++ show cT
        ++ "\n c = " ++ show c
    ) $
    HM.fromList $ zip [0..] (c0Double / 2 : c)
    where
    (c0Double : c) = map (* (2 /| cN)) (tDCT_I_nlogn cT) -- interpolate the values using a polynomial 
     
    cT = zipWith (*) aT bT -- multiplication of the cN+1 values of the polynomials on the grid
    
    aT = tDCT_I_nlogn a -- compute the values of the polynomial termsA on a grid
    bT = tDCT_I_nlogn b -- compute the values of the polynomial termsB on a grid
    
    -- convert from sparse to dense representation:
    a = pad0 $ (2 * a0) : [HM.lookupDefault 0 i termsA | i <- [1..dA]]
    a0 = HM.lookupDefault 0 0 termsA
    b = pad0 $ (2 * b0) : [HM.lookupDefault 0 i termsB | i <- [1..dB]]
    b0 = HM.lookupDefault 0 0 termsB
    pad0 list = take (cN + 1) $ list ++ (repeat 0)
    
    cN = 2 ^ (1 + (intLog2 $ max 1 (dA + dB)))
    dA = maximum $ HM.keys termsA
    dB = maximum $ HM.keys termsB

{-|
    DCT-I computed directly from its definition in
    [BT97, page 18, display (6.1)].
    
    This is quite inefficient for large N.
    It is to be used only for N<8 and as a reference in tests.
-}
tDCT_I_reference :: 
    [RA] {-^ @a@ a vector of validated real numbers -} -> 
    [RA] {-^ @a~@ a vector of validated real numbers -}
tDCT_I_reference a =
    [sum [ (eps cN k) * (a !! k) * cos ( ((mu * k) |* rPi) /| cN)
            | k <- [0..cN] 
         ] 
        | mu <- [0..cN]
    ]
    where
    cN = length a - 1
    rPi = piOut -- rSample
--    (rSample : _) = g

{-| An auxiliary family of constants, frequently used in Chebyshev-basis expansions. -}
eps :: Int -> Int -> RA
eps n k 
    | k == 0 = 0.5
    | k == n = 0.5
    | otherwise = 1 


{-|
    DCT-I computed by splitting N and via DCT-III as described in
    [BT97, page 18, Proposition 6.1].
    
    Precondition: (length a) = 1+2^{t+1} where t > 1
-}
tDCT_I_nlogn :: 
    [RA] {-^ @a@ a vector of validated real numbers -} -> 
    [RA] {-^ @a~@ a vector of validated real numbers -}

tDCT_I_nlogn a 
    | cN < 8 = tDCT_I_reference a
    | otherwise = map aTilde [0..cN]
    where
    aTilde i 
        | even i = fTilde !! (i `div` 2)
        | otherwise = gTilde !! ((i - 1) `div` 2)
    fTilde = tDCT_I_nlogn f
    gTilde = tDCT_III_nlogn g
    f = [ (a !! ell) + (a !! (cN - ell)) | ell <- [0..cN1]]
    g = [ (a !! ell) - (a !! (cN - ell)) | ell <- [0..cN1-1]]
    cN = (length a) - 1
    cN1 = cN `div` 2

{-|
    DCT-III computed directly from its definition in
    [BT97, page 18, display (6.2)].
    
    This is quite inefficient.  It is to be used only as a reference in tests.
-}
tDCT_III_reference :: 
    [RA] {-^ g a vector of validated real numbers -} -> 
    [RA] {-^ g~ a vector of validated real numbers -}
tDCT_III_reference g =
    [sum [ (eps cN1 k) * (g !! k) * cos ( (((2*j+1)*k) |* rPi) /| cN)
            | k <- [0..(cN1-1)] 
         ] 
        | j <- [0..(cN1-1)]
    ]
    where
    cN = cN1 * 2
    cN1 = length g
    rPi = piOut -- rSample
--    (rSample : _) = g

{-|
    DCT-III computed via SDCT-III.  The reduction is described on page 20. 
    
    Precondition: length g is a power of 2
-}
tDCT_III_nlogn :: 
    [RA] {-^ g a vector of validated real numbers -} -> 
    [RA] {-^ g~ a vector of validated real numbers -}
tDCT_III_nlogn g =
    h2g $ tSDCT_III_nlogn $ map g2h $ zip [0..] g 
    where
    g2h (i,gi) = (eps cN1 i) * gi
    h2g h = map get_g [0..cN1-1]
        where
        get_g i  
            | even i = h !! (i `div` 2)
            | otherwise = h !! ((2*cN1 - i - 1) `div` 2)   
    cN1 = length g


{-|
    Simplified DCT-III computed directly from its definition in
    [BT97, page 20, display (6.3)].
    
    This is quite inefficient.  It is to be used only as a reference in tests.
-}
tSDCT_III_reference :: 
    [RA] {-^ h a vector of validated real numbers -} -> 
    [RA] {-^ h~ a vector of validated real numbers -}
tSDCT_III_reference h =
--    trace (
--        "rPi = " ++ show rPi
--        ++ "\ncN = " ++ show cN
--        ++ "\ncN1 = " ++ show cN1
--        ++ "\n( (((4*3+1)*3 :: Int) |* rPi) /| cN) = " ++ show (( (((4*3+1)*3 :: Int) |* rPi) /| cN))
--        ++ "\ncos ( (((4*3+1)*3 :: Int) |* rPi) /| cN) = " ++ show (cos ( (((4*3+1)*3 :: Int) |* rPi) /| cN))
--    ) $
    [sum [ (h !! ell) * cos ( (((4*j+1)*ell) |* rPi) /| cN)
            | ell <- [0..(cN1-1)] 
         ] 
        | j <- [0..(cN1-1)]
    ]
    where
    cN = cN1 * 2
    cN1 = length h
    rPi = piOut -- rSample
--    (rSample : _) = h

{-|
    Simplified DCT-III computed as described in 
    [BT97, page 21, Algorithm 1].
    
    Changed many occurrences of N1 with N1-1 because the indices were out of range.
    This is part of a trial and error process.
    
    Precondition: length h is a power of 2
-}
tSDCT_III_nlogn :: 
    [RA] {-^ h a vector of validated real numbers -} -> 
    [RA] {-^ h~ a vector of validated real numbers -}
tSDCT_III_nlogn h =
    map (\ (_,[a],_) -> a) $
        sortBy (\ (i,_,_) (j,_,_) -> compare i j) $ 
        splitUntilSingletons $ [(0, h, 1)]
    where
    splitUntilSingletons :: [(Int, [RA], Int)] -> [(Int, [RA], Int)]
    splitUntilSingletons groups
        | allSingletons = groups
        | otherwise =
            splitUntilSingletons $
                concat $ map splitGroup groups
        where
        allSingletons = and $ map isSingleton groups
        isSingleton (_, [_], _) = True
        isSingleton _ = False
    splitGroup :: (Int, [RA], Int) -> [(Int, [RA], Int)]
    splitGroup (c_Itau_minus_1, hItau_minus_1, two_pow_tau_minus_1) =
        [subgroup 0, subgroup 1]
        where
        subgroup bit_iTauMinus1 =
            (c_Itau_minus_1 + bit_iTauMinus1 * two_pow_tau_minus_1, 
             map hItau [0..c_Ntau_plus_1-1], 
             2 * two_pow_tau_minus_1)
            where
            hItau 0 = 
                (hItau_minus_1 !! 0) 
                + 
                (minusOnePow bit_iTauMinus1) * (hItau_minus_1 !! (c_Ntau_plus_1)) * gamma
            hItau n = 
                (hItau_minus_1 !! n)
                -
                (hItau_minus_1 !! (c_Ntau - n))
                + 
                (((2 :: Int) |* (minusOnePow bit_iTauMinus1)) * (hItau_minus_1 !! (c_Ntau_plus_1+n)) * gamma)
            gamma =
                cos $ (((4 * c_Itau_minus_1) + 1) |* rPi) /| (4*two_pow_tau_minus_1)
        c_Ntau = length hItau_minus_1
        c_Ntau_plus_1 
            | even c_Ntau = c_Ntau `div` 2
            | otherwise = error "tSDCT_III_nlogn: precondition violated: (length h) has to be a power of 2"
        
    minusOnePow :: Int -> RA
    minusOnePow 0 = 1
    minusOnePow 1 = -1 
    minusOnePow _ = error "tSDCT_III_nlogn: minusOnePow called with a value other than 0,1"

    rPi = piOut -- rSample
--    (rSample : _) = h

{-
allBits :: Int -> [[Int]]
allBits n 
    | n > 0 =
        (map (0:) allBits_n_minus_1) ++ (map (1:) allBits_n_minus_1)
    | n == 0 = [[]]
    | otherwise = error "allBits: precondition violated: n >= 0"
    where
    allBits_n_minus_1 = allBits (n - 1)
-}        
  
  
{-
{-| 
    The formula at the bottom of page 20, fixed by Eike. 
    
    This should be equivalent to tSDCT_III_reference.
-}
aux1 h =
    [htld j 
        | j <- [0..(cN1-1)]
    ]
    where
    htld j =
        (h !! 0) 
        +
        (sum 
            [
                ((h !! n) - (h !! (cN1 - n)))
                *
                (cos ((((4*j + 1)*n) |* rPi)/| cN))
                | n <- [1..(cN2 - 1)]
            ]
        ) 
        +
        2 * cos(((4*j + 1) |* rPi)/ 4) *
        (
            0.5 * (h !! cN2)
            +
            (sum
                [
                    (h !! (cN2 + n))
                    *
                    (cos ((((4*j + 1)*(n)) |* rPi)/| cN))
                    | n <- [1..(cN2 - 1)]
                ]
            )
        )
    cN = cN1 * 2
    cN1 = length h
    cN2 = cN1 `div` 2
    rPi = piOut -- rSample
    (rSample : _) = h
    
-}
