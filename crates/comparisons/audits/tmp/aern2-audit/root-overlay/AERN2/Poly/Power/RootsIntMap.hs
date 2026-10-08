-- Audit compatibility copy; original pinned AERN2 source is unchanged.
-- Legacy unchecked arithmetic spellings are mapped to current raw arithmetic;
-- only exact Integer polynomials on well-formed nonnegative degrees qualified.
module AERN2.Poly.Power.RootsIntMap
(
    initialBernsteinCoefs
  , bernsteinCoefs
  , signVars
  , reflect
  , contract
  , translate
  , transform
--  , findRoots
  , reduce
  , Terms
)
where

import MixedTypesNumPrelude
import Data.Map (Map)
import qualified Data.Map as Map
import qualified Prelude
import AERN2.MP.Ball hiding (iterateUntilAccurate)
import AERN2.MP.Dyadic
import Data.Maybe
import Data.Ratio

import AERN2.Poly.Power.Type
import AERN2.Poly.Basics hiding (Terms)
import AERN2.Poly.Power.Eval

import Debug.Trace

import qualified Prelude as P

shouldTrace :: Bool
shouldTrace = False

maybeTrace :: String -> a -> a
maybeTrace
    | shouldTrace = trace
    | otherwise = const id

type Terms = (ErrorBound, Integer, Map Integer Integer)

reduce :: Terms -> Terms
reduce (e, c, ts) =
  ((e / tsGcd) / tsCGcd, c, ts'')
  where
  tsGcd = Map.foldl' gcd 0 ts
  ts' = if tsGcd > 1 then trace("reduction 1 successful") $ Map.map (`P.div` tsGcd) ts else ts
  tsCGcd = Map.foldl' gcd c ts'
  ts'' = if tsCGcd > 1 then trace("reduction 2 successful") $ Map.map (`P.div` tsCGcd) ts' else ts'

{-instance (HasAccuracy c) => HasAccuracy (Terms c) where
  getAccuracy ts = Map.foldl' min (Exact) $ Map.map getAccuracy ts-}

ts_deg :: Terms -> Integer
ts_deg (_, _, ts) = (fst . Map.findMax) ts

signVars :: Terms -> Maybe Integer
signVars ts@(e, c, cfs) =
  aux 0 0 (ts_deg ts)
  where
  ce = c * rational e
  eZero = ce == 0
  sgn x
   | x == 0 && eZero  = Just 0
   | x >  ce          = Just 1
   | x < -ce          = Just (-1)
   | otherwise        = Nothing
  aux vrs _ (-1) = Just vrs
  aux vrs sg d =
    case sgn (fromJust $ Map.lookup d cfs) of
      Nothing   -> Nothing
      Just sgnx ->
        if sgnx == 0 || sg == 0 || sgnx == sg then
          aux vrs (if sgnx /= 0 then sgnx else sg) (d - 1)
        else
          aux (vrs + 1) sgnx (d - 1)

-- Input: l,m and Polynomial P.
-- Output: some positive integer constant c and coefficients of c*P in Bernstein basis on [l,r].
initialBernsteinCoefs ::  PowPoly Integer -> ErrorBound -> Rational -> Rational -> Terms
initialBernsteinCoefs p e l r =
  (e, lambda, bs)
  where
  lI = if l == 1.0 then 2 else 1
  d = degree p
  PowPoly (Poly csI) = transform (-1) (integer lI) p
  binoms = Map.fromList [(k, binom d (d - k)) | k <- [0.. d]]
  bsFrac = Map.mapWithKey (\k c -> (toRational c) / (toRational $ fromJust $ Map.lookup k binoms)) csI
  lambdaI = Map.foldl' lcm 1 (Map.map denominator bsFrac)
  bsI = Map.mapKeys (\k -> d - k) $ Map.mapWithKey (\k c -> lambdaI*c `Prelude.div` (fromJust $ Map.lookup k binoms)) csI
  (_, (_,lambdaL, bsL)) = bernsteinCoefs (-1.0) (rational lI) l (e, lambdaI, bsI)
  ( (_, lambda, bs), _) = bernsteinCoefs l (rational lI) r (e, lambdaL, bsL) -- TODO: error bound correct?


-- Input: (l,r,m,c, List of coefficients of c*P in Bernstein basis on [l,r])
-- Output: c' and Lists of coefficients of c'*P in Bernstein basis on [l,m] and [m,r].
-- Note that m does not have to lie between l and r.
-- This is Algorithm 10.3 [Special Bernstein Coefficients] in
-- Basu, Pollack, Roy: Algorithms in Real Algebraic Geometry
bernsteinCoefs :: Rational -> Rational -> Rational -> Terms -> (Terms, Terms)
bernsteinCoefs l r m (e, c, bs) =
  ((e, c', buildLeft p 1 Map.empty),
   (e, c', buildRight p 1 Map.empty))
  where
  c' = diff^p*c
  d  = toRational $ foldl1 lcm $ map denominator [l,r,m]
  l' = numerator $ d*l
  m' = numerator $ d*m
  r' = numerator $ d*r
  diff = r' - l'
  p = terms_degree bs
  mp = buildMap 0 0 Map.empty
  buildRight j prd cs =
    if j < 0 then
      cs
    else
      buildRight (j - 1) (prd * diff) (Map.insert (p - j) (prd * (fromJust $ Map.lookup (j,p - j) mp)) cs)
  buildLeft j prd cs =
    if j < 0 then
      cs
    else
      buildLeft (j - 1) (prd*diff) (Map.insert j (prd * (fromJust $ Map.lookup (j,0) mp)) cs)
  buildMap i j tri =
    if i > p then
      tri
    else if j > p - i then
      buildMap (i + 1) 0 tri
    else if i == 0 then
      buildMap i (j + 1) (Map.insert (i,j) (terms_lookupCoeff bs j) tri)
    else
      buildMap i (j + 1) (Map.insert (i,j) ((r' - m')*(fromJust $ Map.lookup (i - 1, j) tri) + (m' - l')*(fromJust $ Map.lookup (i - 1,j + 1) tri)) tri)

reflect :: PowPoly c -> PowPoly c
reflect poly@(PowPoly (Poly ts)) =
  PowPoly $ Poly ts'
  where
  ts' = Map.mapKeys (\p -> deg - p) ts
  deg = degree poly

translate :: Integer -> PowPoly Integer -> PowPoly Integer
translate t poly@(PowPoly (Poly ts)) =
    translateAcc ((degree poly) - 1) $
      (fromList [(0,terms_lookupCoeff ts (degree poly))])
    where
    translateAcc (-1) poly' = poly'
    translateAcc n poly' =
      let
        c = terms_lookupCoeff ts n
      in
        translateAcc (n - 1) $ c + (shiftRight 1 poly') - (t*poly')

contract :: Integer -> PowPoly Integer -> PowPoly Integer
contract l (PowPoly (Poly ts)) =
  PowPoly $ Poly $ Map.mapWithKey (\p c -> c*(l^p)) ts

transform :: Integer -> Integer -> PowPoly Integer -> PowPoly Integer
transform l r =
  (translate (-1)) .
    (reflect) .
    (contract (r - l)) .
    (translate (-l))

{- root finding algorithm: -}

data HasRoot =
  Yes | DontKnow
  deriving (Prelude.Eq)

instance HasEqAsymmetric HasRoot HasRoot where
  type EqCompareType HasRoot HasRoot = Bool

{-
findRoots :: PowPoly Integer -> MPBall -> MPBall -> ((MPBall, MPBall) -> Bool) -> [(MPBall, MPBall)] -- TODO: if signVars == Nothing - recompute or give up? (currently giving up)
findRoots p l r intervalOK =      -- TODO: remove intervalOK and iterate until nix mehr geht
  splitUntilAccurate [(l, r, DontKnow, bsI)] []
  where
  lDown = rational $ dyadic (ball_value l) - dyadic (ball_error l)
  rUp   = rational $ dyadic (ball_value r) + dyadic (ball_error r)
  bsI = initialBernsteinCoefs p lDown rUp
  sgn x
   | x == 0 = 0
   | x >  0 = 1
   | x < 0  = -1
  splitUntilAccurate ::
    [(Rational, Rational, HasRoot, (Integer, Terms))]
    -> [(Rational, Rational)]
    -> [(Rational, Rational)]
  splitUntilAccurate [] res = res
  splitUntilAccurate ((a, b, hasRoot, (c, ts)) : is) res =
    let
      vars = signVars ts
    in
      maybeTrace (
      "interval " ++ (show (a, b)) ++ "\n" ++
       "coefs: " ++(show ts)++"\n"++
       "vars: "++(show vars)
      ) $
      if vars == 0 then
        splitUntilAccurate is res
      else if intervalOK (a, b) then
        splitUntilAccurate is ((a, b):res)
      else if hasRoot == Yes || vars == 1 then
        let
          m = 0.5*(a + b)
          sgnM     = sgn $ evalDirect p m  -- TODO: improve data structure to avoid redundant computations
          sgnLeft  = sgn $ evalDirect p a
          sgnRight = sgn $ evalDirect p b
        in
          maybeTrace (
           "m: "++ show m++"\n"++
           "sgnM: "++ show sgnM ++"\n"++
           "value: "++ show (evalDirect p m) ++"\n"++
           "comparison: "++ show (evalDirect p m == 0)
          ) $
          if (evalDirect p m == 0) == Just True then
            splitUntilAccurate is ((m,m):res)
          else if isJust sgnM && isJust sgnLeft && isJust sgnRight then
            if sgnLeft /= sgnM then
              splitUntilAccurate ((a, m, Yes, ts):is) res
            else
              splitUntilAccurate ((m, b, Yes, ts):is) res
          else
            splitUntilAccurate is ((a,b):res)
      else
        let
          findM cm =
            let
              sgnM = sgn $ evalDirect p cm
            in
              if isJust sgnM
                && fromJust sgnM /= 0
              then
                cm
              else
                findM $ 0.5*(cm + b) -- TODO: find a better perturbation function
          m = findM $ 0.5*(a + b)
          (bsL, bsR)  = bernsteinCoefs a b m ts
        in
          splitUntilAccurate
            ((a,m, DontKnow, bsL) : (m,b, DontKnow, bsR) : is) res-}

{- auxiliary functions -}

binom :: Integer -> Integer -> Integer
binom _ 0 = 1
binom n k =  binom n (k - 1) * (n - k + 1) `Prelude.div` k

