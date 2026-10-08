{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Data.Number.IReal
import Data.Number.IReal.IReal (appr,IReal(..),ir)
import Data.Number.IReal.IntegerInterval
import qualified Data.Number.IReal.FAD as D
import Data.Number.IReal.Generators (isCauchy)
import Data.Ratio
import Data.Bits
import Control.Monad
import Control.Exception
import Data.IORef
import System.IO

rat q = show (numerator q)++"/"++show (denominator q)
encl x p = let I(l,u)=appr x p in (l%bit p,u%bit p)
check stats name expected x p = do
  let (l,u)=encl x p; (a,b)=expected
      good=l<=a&&b<=u; strict=l<a&&b<u
  modifyIORef' stats $ \(n,f,s)->(n+1,f+if good then 0 else 1,s+if strict then 0 else 1)
  unless good $ putStrLn $ unwords ["FAIL",name,show p,show (map rat [a,b,l,u])]
summary name ref = readIORef ref >>= \(n,f,s)->putStrLn (unwords ["TOTAL",name,show n,"FAIL",show f,"NONSTRICT",show s])
ps=[0,1,2,8,32,128]
points=[n%d | n<-[-12..12],d<-[1,2,3,7,16]]
intervals=[(m-r,m+r) | m<-[-3,-1,0,1,3],r<-[0,1%16,1%2,2]]
mk (l,u)=((l+u)/2)+-((u-l)/2)
mulBounds (a,b) (c,d)=let xs=[a*c,a*d,b*c,b*d] in (minimum xs,maximum xs)
absBounds (a,b)=(if a<=0&&b>=0 then 0 else min (abs a) (abs b),max (abs a) (abs b))
bin n k=product [n-k+1..n] `div` product [1..k]
convolution xs ys=[sum [xs!!j * ys!!(k-j)*bin (toInteger k) (toInteger j)
                     | j<-[max 0 (k-length ys+1)..min k (length xs-1)]]
                   | k<-[0..length xs+length ys-2]]
main = do
  hSetBuffering stdout LineBuffering
  pointStats<-newIORef (0::Int,0::Int,0::Int)
  forM_ points $ \a->forM_ points $ \b->forM_ ps $ \p->do
    let x=fromRational a; y=fromRational b
    check pointStats "point-add" (a+b,a+b) (x+y) p
    check pointStats "point-mul" (a*b,a*b) (x*y) p
    when (b/=0) $ check pointStats "point-div" (a/b,a/b) (x/y) p
  summary "point-arithmetic" pointStats
  intervalStats<-newIORef (0::Int,0::Int,0::Int)
  forM_ intervals $ \ab@(a,b)->forM_ ps $ \p-> do
    check intervalStats "constructor" ab (mk ab) p
    check intervalStats "abs" (absBounds ab) (abs (mk ab)) p
    forM_ [0..8] $ \n->do
      let expected=if even n then let (l,u)=absBounds ab in (l^n,u^n) else (a^n,b^n)
      check intervalStats "power" expected (pow (mk ab) n) p
    forM_ intervals $ \cd@(c,d)->do
      let inputs=show (map rat [a,b,c,d])
      check intervalStats ("interval-add "++inputs) (a+c,b+d) (mk ab+mk cd) p
      check intervalStats ("interval-mul "++inputs) (mulBounds ab cd) (mk ab*mk cd) p
      when (c*d>0) $ check intervalStats ("interval-div "++inputs) (mulBounds ab (1/d,1/c)) (mk ab/mk cd) p
  summary "interval-arithmetic" intervalStats
  sumStats<-newIORef (0::Int,0::Int,0::Int)
  forM_ [0,1,2,3,7,16,63,128] $ \n->forM_ ps $ \p->do
    let qs=take n (cycle points); exact=sum qs
    forM_ [("sum",sum),("bsum",bsum),("isum",isumN' (toInteger n))] $ \(name,f)->
      check sumStats name (exact,exact) (f (map fromRational qs)) p
  summary "sums" sumStats
  let derivativeCases=[(xs,ys) | m<-[1..12],n<-[1..12],a<-[-2..2],b<-[-2..2],
        let xs=[a*i*i+1 | i<-[1..m]],let ys=[b*i-2 | i<-[1..n]]]
      bad=[(xs,ys,D.convs xs ys,convolution xs ys) | (xs,ys)<-derivativeCases,D.convs xs ys/=convolution xs ys]
  putStrLn $ "TOTAL convolution "++show (length derivativeCases)++" FAIL "++show (length bad)
  mapM_ print (take 3 bad)
  let powerCases=[(n,k,x) | n<-[0..16],k<-[0..20],x<-[-3..3]]
      expected n k x=if k>n then 0 else product [toInteger(n-k+1)..toInteger n]*x^(n-k)
      powerBad=[(n,k,x,got,expected n k x) | (n,k,x)<-powerCases,let got=deriv k (\t->pow t n) x,got/=expected n k x]
  putStrLn $ "TOTAL derivatives "++show (length powerCases)++" FAIL "++show (length powerBad)
  mapM_ print (take 3 powerBad)
  putStrLn $ "ordered-fold "++show [(n,foldb (++) "" xs,foldb' (++) "" xs) | n<-[2..8],let xs=map (:[]) (take n ['a'..])]
  let badApprox p=fromInteger (if p<10 then 0 else bit p)
  putStrLn $ "Cauchy raw/memoized "++show (isCauchy (IR badApprox) 2 10,isCauchy (ir badApprox) 2 10)
  forM_ [("abs-zero",abs 0),("closed-zero",0+-0),("square-zero",sq 0)] $ \(label,x)->do
    result<-try (evaluate (let I(l,u)=appr (signum x) 8 in (l+u) `seq` (l,u))) :: IO (Either SomeException (Integer,Integer))
    putStrLn $ "signum "++label++" "++show result
  let lazyEndpoint=IR (\_->I(error "endpoint evaluated",0))
  forced<-try (evaluate (force 30 lazyEndpoint) >> return ()) :: IO (Either SomeException ())
  putStrLn $ "force endpoint "++show forced
  putStrLn $ "finite polynomial prefixes "++show (take 10 (D.fromDif (con (1::Integer)+con 2)))
