-- Audit compatibility copy; original pinned AERN2 source is unchanged.
-- Legacy unchecked arithmetic spellings are mapped to current raw arithmetic;
-- only exact Integer polynomials on well-formed nonnegative degrees qualified.
module AERN2.Poly.Power.SignedSubresultantVector where

import MixedTypesNumPrelude
import AERN2.MP.Ball
import AERN2.Poly.Power.Type
import AERN2.Poly.Basics

import qualified Prelude as P
import Data.List as List

import Data.Map (Map)
import qualified Data.Map as Map

import Data.Maybe

import Debug.Trace

type IntPoly = PowPoly Integer

separablePart :: IntPoly -> IntPoly
separablePart p =
  if degree p <= 0 || degree (derivative p) <= 0 then
    p
  else
    snd $ gcdAndgcdFreePart p (derivative p)

{- For this algorithm, compare Basu, Pollack, Roy, Prop 10.14 -}
gcdAndgcdFreePart :: IntPoly -> IntPoly -> (IntPoly, IntPoly)
gcdAndgcdFreePart p q =
  (justLookup deggcd sResP, justLookup (deggcd - 1) sResV)
  where
  (sResP, _, sResV) = extendedSignedSubresultant p q
  deggcd = aux 0
  aux i =
    if not $ isZero $ justLookup i sResP then
      i
    else
      aux (i + 1)

extendedSignedSubresultant ::
  IntPoly -> IntPoly ->
  (Map.Map Integer IntPoly, Map.Map Integer IntPoly, Map.Map Integer IntPoly)
extendedSignedSubresultant p@(PowPoly (Poly pts)) q@(PowPoly (Poly qts)) =
  aux1 $ aux0 $ initialiseESS
  where

  epsilon k = (-1)^(k*(k - 1) `P.div` 2)
  a k = justLookup k pts
  b k = justLookup k qts

  degp = integer $ degree p
  degq = integer $ degree q

  initialiseESS =
    (sResP0, sResU0, sResV0, s0, t0, i0, j0)
    where
      sResP0 =
        foldl'
        (\m k -> Map.insert k zero m)
        ( Map.insert degq ((epsilon $ degp - degq) * ((b degq)^(degp - degq - 1)) * q) $
          Map.fromList [(degp - 1, q), (degp, p)])
        [degq + 1 .. degp - 2]
      sResU0 =
        foldl'
        (\m k -> Map.insert k zero m)
        (Map.insert degq zero $
         Map.fromList [(degp - 1, zero), (degp, zero)])
        [degq + 1 .. degp - 2]
      sResV0 =
         foldl'
         (\m k -> Map.insert k zero m)
         (Map.insert degq (fromList [(0,(epsilon $ degp - degq) * (b degq)^(degp - degq))]) $
          Map.fromList [(degp - 1, one), (degp, zero)])
         [degq + 1 .. degp - 2]
      s0 =
          foldl'
             (\m k -> Map.insert k 0 m)
             (Map.insert degq ((epsilon $ degp - degq) * ((b degq)^(degp - degq))) $
              Map.fromList [(degp - 1, b degq) , (degp, 1)])
             [degq + 1 .. degp - 2]
      t0 = Map.fromList [(degp, 1), (degp - 1, b degq)]
      i0 = degp + 1
      j0 = degp

  aux1 (sResP, sResU, sResV, s, t, i, j) =
    (sResP', sResU, sResV)
    where
    sResP' = foldl'
             (\m k -> Map.insert k zero m)
             sResP
             [0 .. j - 2]

  aux0 (sResP, sResU, sResV, s, t, i, j) =
    {-trace
    (
      "sResP: "++ show sResP ++"\n"++
      "sResU: "++ show sResU ++"\n"++
      "sResV: "++ show sResV ++"\n"++
      "s: "++ show s ++"\n"++
      "t: "++ show t ++"\n"++
      "j - 1: "++ show (j - 1) ++"\n"++
      "i - 1: "++ show (i - 1) ++"\n"
    )
    $-}
    if isZero $ justLookup (j - 1) sResP then
      (sResP, sResU, sResV, s, t, i, j)
    else
      let
        k = integer $ degree $ justLookup (j - 1) sResP
      in
        if k == j - 1 then
          let
            s' = Map.insert (j - 1) (justLookup (j - 1) t) s
            s2 = (justLookup (j - 1) s')^2
            quot = (justLookup j s') * (justLookup (i - 1) t)
            ck   = quo (s2 * (justLookup (i - 1) sResP)) (justLookup (j - 1) sResP)
            sResP' = Map.insert (k - 1) (sweepLeadingZeroes $ (-s2 * (justLookup (i - 1) sResP) + ck * (justLookup (j - 1) sResP))/quot) sResP
            sResU' = Map.insert (k - 1) (sweepLeadingZeroes $ (-s2 * (justLookup (i - 1) sResU) + ck * (justLookup (j - 1) sResU))/quot) sResU
            sResV' = Map.insert (k - 1) (sweepLeadingZeroes $ (-s2 * (justLookup (i - 1) sResV) + ck * (justLookup (j - 1) sResV))/quot) sResV
            t' = Map.insert (k - 1) (leadingCoefficient $ justLookup (k - 1) sResP') t
            i' = j
            j' = k
          in
            aux0 (sResP', sResU', sResV', s', t', i', j')
        else  -- defective case
          let
            s' = Map.insert (j - 1) 0 s
            (sResP', sResU', sResV', t', s'') = trace("s j is "++(show $  (justLookup j s))) $ aux01 sResP sResU sResV s' t j k
            (sResP'', sResU'', sResV'', s''') = aux02 sResP' sResU' sResV' s'' t' j k
            (sResP''', sResU''', sResV''') = aux03 sResP'' sResU'' sResV'' s''' t' i j k
            t'' = Map.insert (k - 1) (leadingCoefficient $ justLookup (k - 1) sResP''') t'
            i' = j
            j' = k
          in
            aux0 (sResP''', sResU''', sResV''', s''', t'', i', j')

  aux01 sResP sResU sResV s t j k =
    (sResP', sResU', sResV', t', s')
    where
    t' =
      foldl'
      (\m d -> Map.insert (j - d - 1) (((-1)^d * justLookup (j - 1) m * justLookup (j - d) m) `P.div` (justLookup j s)) m)
      t
      [1 .. j - k - 1]
    s' = Map.insert k (justLookup k t') s
    sResP' = Map.insert k (sweepLeadingZeroes $  ((justLookup k s') * justLookup (j - 1) sResP) / justLookup (j - 1) t') sResP
    sResU' = Map.insert k (sweepLeadingZeroes $  ((justLookup k s') * justLookup (j - 1) sResU) / justLookup (j - 1) t') sResU
    sResV' = Map.insert k (sweepLeadingZeroes $  ((justLookup k s') * justLookup (j - 1) sResV) / justLookup (j - 1) t') sResV

  aux02 sResP sResU sResV s t j k =
    (sResP', sResU', sResV', s')
    where
    sResP' =
      foldl'
      (\m l -> Map.insert l zero m)
      sResP
      [j - 2 .. k + 1]
    sResU' =
      foldl'
      (\m l -> Map.insert l zero m)
      sResU
      [j - 2 .. k + 1]
    sResV' =
      foldl'
      (\m l -> Map.insert l zero m)
      sResV
      [j - 2 .. k + 1]
    s' =
      foldl'
      (\m l -> Map.insert l 0 m)
      s
      [j - 2 .. k + 1]

  aux03 sResP sResU sResV s t i j k =
    (sResP', sResU', sResV')
    where
    scale = (justLookup k s) * (justLookup (j - 1) t)
    quot  =
      trace("first: "++(show $ (justLookup j s))) $
      trace("second: "++(show $ (justLookup (i - 1) t))) $
      trace("s: "++(show $ (s))) $
      ((justLookup j s) * (justLookup (i - 1) t))
    c = quo (scale * justLookup (i - 1) sResP) (justLookup (j - 1) sResP)
    sResP' = Map.insert (k - 1) ((-scale * (justLookup (i - 1) sResP) + c * (justLookup (j - 1) sResP))/quot) sResP
    sResU' = Map.insert (k - 1) ((-scale * (justLookup (i - 1) sResU) + c * (justLookup (j - 1) sResU))/quot) sResU
    sResV' = Map.insert (k - 1) ((-scale * (justLookup (i - 1) sResV) + c * (justLookup (j - 1) sResV))/quot) sResV

justLookup :: (P.Ord k, Show k, Show a) => k -> Map.Map k a -> a
justLookup x m = (fromJust . Map.lookup x) m


instance CanDiv IntPoly Integer where
  type DivType IntPoly Integer = IntPoly
  divide (PowPoly (Poly ts)) c =
    PowPoly $ Poly $ Map.map (`P.div` c) ts

isZero :: IntPoly -> Bool
isZero (PowPoly (Poly ts)) =
  terms_degree ts <= 0
  && terms_lookupCoeff ts 0 == 0

zero :: IntPoly
zero = PowPoly $ Poly $ terms_fromList [(0,0)]

one :: IntPoly
one = PowPoly $ Poly $ terms_fromList [(0,1)]

coeff :: Integer -> IntPoly -> Integer
coeff j (PowPoly (Poly ts)) =
  terms_lookupCoeff ts j

leadingCoefficient :: IntPoly -> Integer
leadingCoefficient (PowPoly (Poly ts)) =
  terms_lookupCoeff ts (terms_degree ts)

sweepLeadingZeroes :: IntPoly -> IntPoly
sweepLeadingZeroes (PowPoly (Poly ts)) =
  PowPoly $ Poly $ terms_filterKeepConst (\_k c -> c /= 0) ts


{-| Quotient of Euclidean division. Only valid if result is again an integer polynomial. -}
quo :: IntPoly -> IntPoly -> IntPoly
quo p q =
  aux zero p dp
  where
  dp = degree p
  dq = degree q
  xPow k = fromList [(k,1)]
  aux c r j =
    if j == dq - 1 then
      c
    else
      let
      lt = (coeff j r `P.div` leadingCoefficient q) * (xPow (j - dq))
      c' = c + lt
      r' = r - lt * q
      in
      aux c' r' (j - 1)

