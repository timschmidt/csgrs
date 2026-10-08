{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Real.CReal as C
import qualified Data.Real.ICReal as I
import qualified Data.Real.Base as B
import qualified Data.Real.Complete as M
import qualified Data.Interval as V
import qualified Data.Multinomial as P
import Data.Ratio
import Data.Bits
import Data.List (foldl')
import Control.Exception
import Control.Monad
import System.Environment
import System.IO

two :: Int -> Rational
two n | n>=0 = fromInteger (1 `shiftL` n)
      | otherwise = 1 % (1 `shiftL` negate n)
report :: String -> [(Bool,String)] -> IO ()
report label cases = do
  let (!n,!bad,ex)=foldl' step (0::Int,0::Int,[]) cases
      step (!n,!b,ex) (ok,msg)=(n+1,b+if ok then 0 else 1,
                                if not ok && b<3 then ex++[msg] else ex)
  putStrLn $ label++"\tchecks="++show n++"\tfailures="++show bad
  mapM_ (putStrLn . ("  "++)) ex
guarded :: String -> IO () -> IO ()
guarded label action = catch action $ \e -> putStrLn(label++"\tEXCEPTION "++displayException(e::SomeException))
ends :: V.Interval Rational -> [Rational]
ends (V.Interval (a,b))=[a,b]
encloses :: V.Interval Rational -> [Rational] -> Bool
encloses z xs = head(ends z)<=minimum xs && maximum xs<=last(ends z)
fuzzReal :: Rational -> Rational -> C.CReal
fuzzReal q s=C.unsafeMkCReal(\e->q+s*e)
poly :: [Rational] -> P.Polynomial Rational
poly=foldr (\a p -> P.constP a+P.xP*p) 0
oraclePoly :: [Rational] -> Rational -> Rational
oraclePoly cs x=sum [c*x^k|(c,k)<-zip cs [0::Int ..]]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args<-getArgs
  case args of
    ["sqrt-zero"] -> print(C.approx (sqrt (C.inject 0)) (two (-16)))
    ["rational-sqrt-zero"] -> print(C.approx (C.rationalSqrt 0) (two (-16)))
    ["asin-one"] -> print(C.approx (asin (C.inject 1)) (two (-16)))
    ["acos-one"] -> print(C.approx (acos (C.inject 1)) (two (-16)))
    ["sqrt-perturbed"] -> print(C.approx (sqrt(fuzzReal 0 (-1))) (two (-16)))
    ["poly-zero-derivative"] -> print(P.evalP (P.dx (0::P.Polynomial Rational)) 3)
    ["poly-zero-lift"] -> print(C.approx(C.realBasePolynomialBound 0 (C.compact(C.inject 3))) (two (-16)))
    _ -> pureChecks

pureChecks :: IO ()
pureChecks=do
  report "approxBase-exact-error-and-dyadic"
    [(abs(z-q)<=e && let d=denominator z in d .&. (d-1)==0,show(q,e,z))|
     m<-[-127..127],d<-[1,3,7,17,32,1024],k<-[-8,-1,0,1,2,8,32,128],
     let q=m%d,let e=two(-k),let z=B.approxBase q e]
  report "sumBase-and-powers"
    [(B.sumBase xs==sum [q^k|k<-[0..n-1]],show(q,n))|
     m<-[-15..15],d<-[1,3,7,16,257],n<-[0,1,2,5,16,64],
     let q=m%d,let xs=take n(B.powers q)]
  let intervals=[V.Interval(a%8,b%8)|a<-[-16..16],b<-[a..16]]
  forM_ [("plus",V.intervalPlus,(+)),("mult",V.intervalMult,(*))] $ \(name,f,op)->
    report ("interval-"++name)
      [(encloses z qs,show(ends a,ends b,ends z))|a<-intervals,b<-intervals,
       let z=f a b,let qs=[op x y|x<-ends a,y<-ends b]]
  report "interval-recip"
    [(maybe False (\z->encloses z qs) (V.intervalRecip a),show(ends a))|
     a<-intervals,head(ends a)*last(ends a)>0,let qs=map recip(ends a)]
  report "interval-power"
    [(encloses (V.intervalPower a n) qs,show(ends a,n,ends(V.intervalPower a n)))|
     a<-intervals,n<-[0..8],
     let qs=[x^n|x<-ends a++[0|head(ends a)<=0&&last(ends a)>=0]]]
  forM_ [("add",(+),(+)),("sub",(-),(-)),("mult",(*),(*)),("div",(/),(/))] $ \(name,f,op) -> guarded name $
    report ("CReal-"++name++"-regular-inputs")
      [(abs(z-op a b)<=e,show(a,b,sa,sb,k,z,op a b))|
       a<-[-7%3,-1%8,0,1%8,7%3],b<-[-5%7,-1%4,0,1%4,5%7],name/="div"||b/=0,
       sa<-[-1,0,1],sb<-[-1,0,1],k<-[0,2,8,32],let e=two(-k),
       let z=C.approx(f(fuzzReal a sa)(fuzzReal b sb)) e]
  report "CReal-abs"
    [(abs(z-abs q)<=e,show(q,s,k,z))|q<-[-7%3,-1%8,0,1%8,7%3],s<-[-1,0,1],
     k<-[0,2,8,32,128],let e=two(-k),let z=C.approx(abs(fuzzReal q s)) e]
  report "CReal-positive-sqrt-exact-square"
    [(z+e>=0 && (z-e<=0 || (z-e)^2<=q) && q<=(z+e)^2,show(q,k,z))|
     q<-[1%1024,1%16,1%4,1,2,3,4,17,1024],k<-[0,2,8,32,128],
     let e=two(-k),let z=C.approx(C.rationalSqrt q) e]
  let polys=[[a,b,c,d]|a<-[-2,0,2],b<-[-1,0,1],c<-[0,1],d<-[-1,1]]
  report "polynomial-evaluation-derivative"
    [(P.evalP p x==oraclePoly cs x && P.evalP(P.dx p) x==oraclePoly ds x,show(cs,x))|
     cs<-polys,x<-[-2,-1%3,0,1%3,2],let p=poly cs,
     let ds=zipWith (*) (tail cs) [1..]]
  report "Bernstein-bounds-exact-samples"
    [(encloses bound [oraclePoly cs x],show(cs,ends iv,x,ends bound))|
     cs<-polys,a<-[-2,-1,0],b<-[a+1,a+2],let iv=V.Interval(a,b),j<-[0..16],
     let x=a+(b-a)*(j%16),let bound=P.boundPolynomial iv(poly cs)]
  report "completion-unary-moduli"
    [(abs(M.mapC f (\e->q+s*e) eps - op q)<=eps,show(q,s,eps))|
     q<-[-7%3,0,7%3],s<-[-1,0,1],eps<-map(two . negate)[0,8,32],
     (f,op)<-[(M.mkUniformCts id negate,negate),(M.mkUniformCts id abs,abs),
              (M.mkUniformCts (/7) (*7),(*7))]]
  report "constant-map-laziness"
    [(M.mapC (M.constCts q) (error "constant map evaluated input") eps==q,show(q,eps))|
     q<-[-3,0,7],eps<-map(two . negate)[0,8,32]]
