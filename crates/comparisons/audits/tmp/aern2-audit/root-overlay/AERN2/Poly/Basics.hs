-- Audit-only exact-polynomial compatibility slice, preserving the donor's
-- Map storage, zero-constant convention and exact Integer operations.
-- Not a replacement/build of the full obsolete generic function package.
module AERN2.Poly.Basics where
import MixedTypesNumPrelude
import qualified Data.Map as Map
newtype Poly c = Poly { poly_terms :: Map.Map Integer c } deriving Show
type Degree = Integer
type Terms c = Map.Map Integer c
terms_degree :: Terms c -> Integer
terms_degree = fst . Map.findMax
terms_lookupCoeff :: HasIntegers c => Terms c -> Integer -> c
terms_lookupCoeff ts k = Map.findWithDefault (convertExactly 0) k ts
terms_fromList :: HasIntegers c => [(Integer,c)] -> Terms c
terms_fromList xs = Map.insertWith (\_ old -> old) 0 (convertExactly 0) (Map.fromList xs)
terms_filterKeepConst :: (Integer -> c -> Bool) -> Terms c -> Terms c
terms_filterKeepConst f = Map.filterWithKey (\k c -> k == 0 || f k c)
instance CanAddAsymmetric (Poly Integer) (Poly Integer) where
  type AddType (Poly Integer) (Poly Integer) = Poly Integer
  add (Poly p) (Poly q) = Poly (Map.unionWith (+) p q)
instance CanMulAsymmetric Integer (Poly Integer) where
  type MulType Integer (Poly Integer) = Poly Integer
  mul c (Poly p) = Poly (Map.map (c*) p)

