-- Donor dense Horner recurrence; only unrelated interval APIs omitted.
module AERN2.Poly.Power.Eval where
import MixedTypesNumPrelude
import AERN2.Poly.Basics
import AERN2.Poly.Power.Type
evalDirect :: (CanAddAsymmetric b c, b ~ AddType b c,
               Ring b, HasIntegers b, HasIntegers c) => PowPoly c -> b -> b
evalDirect (PowPoly (Poly ts)) (x :: b) =
  evalHornerAcc (terms_degree ts) (convertExactly 0)
  where
  evalHornerAcc :: Integer -> b -> b
  evalHornerAcc 0 sm = x*sm + terms_lookupCoeff ts 0
  evalHornerAcc k sm = evalHornerAcc (k-1) (x*sm + terms_lookupCoeff ts k)
