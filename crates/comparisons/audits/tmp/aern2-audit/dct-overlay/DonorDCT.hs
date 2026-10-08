-- Audit-only transform slice of AERN2.Poly.Cheb.DCT, pinned d1ac3664.
-- Original copyright (c) Michal Konecny; BSD3, as in upstream LICENSE.
-- Function bodies unchanged except obsolete /! spelling becomes /.
-- Full ChPoly lifting, normalization and cached extrema are not included.
module DonorDCT where
import MixedTypesNumPrelude
import qualified Prelude as P
import qualified Data.List as List
import AERN2.MP
import AERN2.Real
tDCT_I_reference ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ @a@ a vector of validated real numbers -} ->
  [c] {-^ @a~@ a vector of validated real numbers -}
tDCT_I_reference a =
    [sum [ (eps cN k) * (a !! k) * cos ( ((mu * k) * pi) / cN)
            | k <- [0..cN]
         ]
        | mu <- [0..cN]
    ]
    where
    cN = (length a) - 1

{-| An auxiliary family of constants, frequently used in Chebyshev-basis expansions. -}
eps :: Integer -> Integer -> Rational
eps n k
    | k == 0 = 0.5
    | k == n = 0.5
    | otherwise = 1.0

{-|
    DCT-I computed by splitting N and via DCT-III as described in
    [BT97, page 18, Proposition 6.1].

    Precondition: (length a) = 1+2^{t+1} where t > 1
-}
tDCT_I_nlogn ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ @a@ a vector of validated real numbers -} ->
  [c] {-^ @a~@ a vector of validated real numbers -}
tDCT_I_nlogn a
    | cN < 8 = tDCT_I_reference a
    | otherwise = map aTilde [0..cN]
    where
    aTilde i
        | even i = fTilde !! (floor (i/2))
        | otherwise = gTilde !! (floor ((i - 1)/2))
    fTilde = tDCT_I_nlogn f
    gTilde = tDCT_III_nlogn g
    f = [ (a !! ell) + (a !! (cN - ell)) | ell <- [0..cN1]]
    g = [ (a !! ell) - (a !! (cN - ell)) | ell <- [0..cN1-1]]
    cN = (length a) - 1
    cN1 = floor (cN / 2)

{-|
    DCT-III computed directly from its definition in
    [BT97, page 18, display (6.2)].

    This is quite inefficient.  It is to be used only as a reference in tests.
-}
_tDCT_III_reference ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ g a vector of validated real numbers -} ->
  [c] {-^ g~ a vector of validated real numbers -}
_tDCT_III_reference g =
    [sum [ (eps cN1 k) * (g !! k) * cos ( (((2*j+1)*k) * pi) / cN)
            | k <- [0..(cN1-1)]
         ]
        | j <- [0..(cN1-1)]
    ]
    where
    cN = cN1 * 2
    cN1 = (length g)

{-|
    DCT-III computed via SDCT-III.  The reduction is described on page 20.

    Precondition: (length g) is a power of 2
-}
tDCT_III_nlogn ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ g a vector of validated real numbers -} ->
  [c] {-^ g~ a vector of validated real numbers -}
tDCT_III_nlogn g =
    h2g $ tSDCT_III_nlogn $ map g2h $ zip [0..] g
    where
    g2h (i,gi) = (eps cN1 i) * gi
    h2g h = map get_g [0..cN1-1]
        where
        get_g i
            | even i = h !! (floor (i/2 :: Rational))
            | otherwise = h !! (floor $ (2*cN1 - i - 1)/2)
    cN1 = (length g)

{-|
    Simplified DCT-III computed directly from its definition in
    [BT97, page 20, display (6.3)].

    This is quite inefficient.  It is to be used only as a reference in tests.
-}
_tSDCT_III_reference ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ h a vector of validated real numbers -} ->
  [c] {-^ h~ a vector of validated real numbers -}
_tSDCT_III_reference h =
    [sum [ (h !! ell) * cos ( (((4*j+1)*ell) * pi) / cN)
            | ell <- [0..(cN1-1)]
         ]
        | j <- [0..(cN1-1)]
    ]
    where
    cN = cN1 * 2
    cN1 = (length h)

{-|
    Simplified DCT-III computed as described in
    [BT97, page 21, Algorithm 1].

    Changed many occurrences of N1 with N1-1 because the indices were out of range.
    This is part of a trial and error process.

    Precondition: length h is a power of 2
-}
tSDCT_III_nlogn ::
  -- (Field c, CanMulBy c CauchyReal) =>
  -- _ =>
  (c ~ MPBall) =>
  [c] {-^ h a vector of validated real numbers -} ->
  [c] {-^ h~ a vector of validated real numbers -}
tSDCT_III_nlogn (h :: [c]) =
    map (\ (_,[a],_) -> a) $
        List.sortBy (\ (i,_,_) (j,_,_) -> P.compare i j) $
        splitUntilSingletons $ [(0, h, 1)]
    where
    splitUntilSingletons :: [(Integer, [c], Integer)] -> [(Integer, [c], Integer)]
    splitUntilSingletons groups
        | allSingletons = groups
        | otherwise =
            splitUntilSingletons $
                concat $ map splitGroup groups
        where
        allSingletons = and $ map isSingleton groups
        isSingleton (_, [_], _) = True
        isSingleton _ = False
    splitGroup :: (Integer, [c], Integer) -> [(Integer, [c], Integer)]
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
                ((2 * (minusOnePow bit_iTauMinus1)) * (hItau_minus_1 !! (c_Ntau_plus_1+n)) * gamma)
            gamma =
                cos $ (((4 * c_Itau_minus_1) + 1) * pi) / (4*two_pow_tau_minus_1)
        c_Ntau = length hItau_minus_1
        c_Ntau_plus_1
            | even c_Ntau = floor (c_Ntau/2)
            | otherwise = error "tSDCT_III_nlogn: precondition violated: (length h) has to be a power of 2"

    minusOnePow :: Integer -> Integer
    minusOnePow 0 = 1
    minusOnePow 1 = -1
    minusOnePow _ = error "tSDCT_III_nlogn: minusOnePow called with a value other than 0,1"

