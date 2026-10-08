module Main where
import Control.Monad (unless)
import qualified Data.IntMap as IM
import Numeric.AERN.Poly.IntPoly.IntPoly

main :: IO ()
main = do
  let original = IntPolyV "x" (IM.fromList [(0,IntPolyC (1::Integer)),(1,IntPolyC 1)])
      collect = termsCollectCoeffsWith (,)
      changed = collect (termsMapConstCoeff (+2) original)
      expected = [([0],3),([1],1)]
  print ("original",collect original)
  print ("constant-only +2 actual",changed)
  print ("constant-only +2 expected",expected)
  print ("changes nonconstant coefficient",changed /= expected)
  unless (collect original == [([0],1),([1],1)]) (error "coefficient traversal mismatch")
  unless (changed == [([0],3),([1],3)]) (error "source suspicion not reproduced")
