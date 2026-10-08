-- Audit-only exact Integer slice of the donor's PowPoly representation.
module AERN2.Poly.Power.Type where
import MixedTypesNumPrelude
import qualified Data.Map as Map
import AERN2.Poly.Basics
data PowPoly c = PowPoly {powPoly_poly :: Poly c} deriving Show
fromList :: HasIntegers c => [(Integer,c)] -> PowPoly c
fromList = PowPoly . Poly . terms_fromList
degree :: PowPoly c -> Integer
degree (PowPoly (Poly ts)) = terms_degree ts
shiftRight :: Integer -> PowPoly c -> PowPoly c
shiftRight n (PowPoly (Poly ts)) = PowPoly (Poly (Map.mapKeys (+n) ts))
derivative :: PowPoly Integer -> PowPoly Integer
derivative (PowPoly (Poly ts)) = PowPoly (Poly
  (Map.mapKeys (\k -> k-1) (Map.mapWithKey (*) (Map.delete 0 ts))))
instance CanNeg (PowPoly Integer) where
  type NegType (PowPoly Integer) = PowPoly Integer
  negate (PowPoly (Poly p)) = PowPoly (Poly (Map.map negate p))
instance CanAddAsymmetric (PowPoly Integer) (PowPoly Integer) where
  type AddType (PowPoly Integer) (PowPoly Integer) = PowPoly Integer
  add (PowPoly p) (PowPoly q) = PowPoly (p+q)
instance CanSub (PowPoly Integer) (PowPoly Integer) where
  type SubType (PowPoly Integer) (PowPoly Integer) = PowPoly Integer
  sub p q = add p (negate q)
instance CanAddAsymmetric Integer (PowPoly Integer) where
  type AddType Integer (PowPoly Integer) = PowPoly Integer
  add c (PowPoly (Poly p)) = PowPoly (Poly (Map.insertWith (+) 0 c p))
instance CanMulAsymmetric Integer (PowPoly Integer) where
  type MulType Integer (PowPoly Integer) = PowPoly Integer
  mul c (PowPoly p) = PowPoly (c*p)
instance CanMulAsymmetric (PowPoly Integer) (PowPoly Integer) where
  type MulType (PowPoly Integer) (PowPoly Integer) = PowPoly Integer
  mul (PowPoly (Poly p)) (PowPoly (Poly q)) = PowPoly
    (Map.foldl' (+) (Poly (terms_fromList [(0,0)]))
      (Map.mapWithKey (\k c -> c*(Poly (Map.mapKeys (+k) q))) p))

