-- Independently evaluated arithmetic expressions from the obsolete univariate
-- source, NOT a build/run of the complete Frac/PPoly/AnalyticMV packages.
module Main where
import Prelude
import Data.Ratio ((%))

contains :: (Rational,Rational) -> Rational -> Bool
contains (l,u) x = l<=x && x<=u
report label enclosure values =
  putStrLn (label ++ ": output=" ++ show enclosure ++ "; exact=" ++ show values ++
            "; contains=" ++ show (map (contains enclosure) values))

main :: IO ()
main = do
  -- ShiftScale scales the two cached extrema in place instead of swapping.
  -- f(x)=x on [0,4], then scalar -2. Cached maximum is -8, but max is 0.
  let oldMin=0::Rational; oldMax=4; scalar = -2
  print ("negative scale cached min/max vs correct",(oldMin*scalar,oldMax*scalar),(-8::Rational,0::Rational))
  -- Frac division by 1/2 changes q=1 to q=1/2 but keeps m=1. For p=x,
  -- evalLip at x=0 +/-1/2 uses lp=1,lq=0,fc=0 and the stale m.
  let m=1::Rational; lp=1; lq=0; fc=0; eps=1%2
      fracErr=m*(lp+lq*abs fc)*eps
  report "Frac stale reciprocal bound" (fc-fracErr,fc+fracErr) [-1,1]
  -- Frac.updateRadius identity sends numerator radius e to qmax*(e-e).
  let e=1%8; qmax=1::Rational; updated=qmax*(id e-e)
  report "Frac identity radius update" (-updated,updated) [-e,e]
  -- PPoly.evalDI physical f=x on [0,4] has unit polynomial 2+2u.
  -- It scales unit derivative 2 by 2/4, then passes radius(x_unit)=1/4
  -- to the Cheb Lipschitz evaluator, so the physical scaling occurs twice.
  let center=2::Rational; physicalScale=2%4; unitDerivative=2
      unitRadius=1%4; pieceErr=physicalScale*unitDerivative*unitRadius
  report "PPoly unit-radius derivative scale" (center-pieceErr,center+pieceErr) [3%2,5%2]
  -- Final PPoly reciprocal correction adds only input radius. Even granting
  -- an exact inverse of the center, c=1/2 +/-1/8 needs a larger output radius.
  let c=1%2; rad=1%8
  report "PPoly reciprocal input-radius correction" (1/c-rad,1/c+rad) [1/(c+rad),1/(c-rad)]
  -- PowS invariant in sum1/sigma is |a_m|<=A*k^sum(m). Valid all-one
  -- series with A=k=1 are not closed under the donor's metadata updates.
  print ("PowS product coefficients respect retained A=k=1",
         [sum [1::Rational | _<-[0..n]]<=1 | n<-[0..8::Int]])
  print ("PowS partial substitution x=1/2 respects retained A=1", (2::Rational)<=1)
