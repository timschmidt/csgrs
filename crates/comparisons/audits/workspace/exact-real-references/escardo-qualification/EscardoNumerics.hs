module EscardoNumerics where
import qualified Escardo as E
import qualified EscardoVariants as V
import EscardoProbe (digits,prefix,streams,check,report)
import Data.Ratio
import Control.Monad
import Control.Exception
import System.Environment
import System.IO

emit label q n x = let p=prefix n x in putStrLn $ concat
  [label,"\t",show(numerator q),"\t",show(denominator q),"\t",show n,
   "\t",show(numerator p),"\t",show(denominator p)]

main=do
 hSetBuffering stdout LineBuffering
 args<-getArgs
 case args of
  ["elementary"] -> do
   forM_ [("mexp",E.mexp),("msin",E.msin),("mcos",E.mcos),
          ("marctan",E.marctan),("marcsin",E.marcsin),("mlni",E.mlni),
          ("mln",E.mln),("inv",E.inv)] $ \(label,f) ->
    forM_ [-1,-15%16,-3%4,-1%2,-1%16,0,1%16,1%2,3%4,15%16,1] $ \q ->
     forM_ [4,8,16,32,64] $ \n -> emit label q n (f(digits q))
   forM_ [("pi/32",E.piDividedBy32),("pi/4",E.piDividedBy4)] $ \(label,x) ->
    forM_ ([4,8,16,32,64,128,512]++[2048|label=="pi/32"]) $ \n -> emit label 0 n x
  ["naive"] -> print(take 1(V.naiveBigMid(repeat E.one)))
  ["extra"] -> do
   let ss=streams 3
   report "fixed-mul-audit-only" [(check n(V.fixedMul x y)(a*b),show(a,b,n))|
     (x,a)<-ss,(y,b)<-ss,n<-[2,5,12,32]]
   report "wider-alphabet-normalization" [(check k(E.divideBy n x)(q/fromIntegral n),show(n,q,k))|
     n<-[2,3,4,7],ds<-replicateM 3 [-n..n],t<-[-n,0,n],
     let x=ds++repeat t,let q=prefix 3 ds+fromIntegral t/8,k<-[2,5,12]]
   report "large-machine-divisor" [(check k(E.divByInt x n)(q/fromIntegral n),show(q,n,k,prefix k(E.divByInt x n)))|
     (x,q)<-[(E.one,1),(E.minusOne,-1),(digits(3%4),3%4)],
     n<-[maxBound `div` 4,maxBound `div` 2,maxBound-1,maxBound],k<-[64,80,128]]
   report "large-machine-normalizer" [(check k(E.divideBy n x)(q/fromIntegral n),show(n,q,k))|
     n<-[maxBound `div` 4,maxBound `div` 2,maxBound-1,maxBound],
     let a=n-1,let x=repeat a,let q=fromIntegral a,k<-[2,8,64]]
   report "domain-restricted-root-audit-only" [(check n(V.domainRoot(digits(q*q)))q,show(q,n))|
     q<-[0,1%1024,1%8,1%4,1%2,3%4,1],n<-[2,8,16,32,64]]
   report "gluing-at-unresolved-equal-branches"
     [(check n(E.ppifz z x y)q,show(q,n))|
      (z,0)<-ss,(x,q)<-ss,(y,r)<-ss,q==r,n<-[2,8,32]]
   report "old-normalizer-reference" [(check n(V.twoDigitDivide2 x)(q/2),show(q,n))|
     ds<-replicateM 3 [-2..2],t<-[-2,0,2],let x=ds++repeat t,
     let q=prefix 3 ds+fromIntegral t/8,n<-[2,8,32]]
   -- A forced error beyond the documented lookahead is an observable boundary.
   forM_ [("zero-one-lookahead",E.divideBy2 [0],True),
          ("endpoint-one-lookahead",E.divideBy2 [2],True),
          ("ordinary-needs-two",E.divideBy2 [1],False),
          ("old-zero-needs-two",V.twoDigitDivide2 [0],False)] $ \(label,x,ok) -> do
     r<-try(evaluate(head x)) :: IO (Either SomeException Int)
     let passed=case r of Right _ -> ok;Left _ -> not ok
     putStrLn(label++"\tpass="++show passed)
  _ -> error "mode: elementary | extra | naive"
