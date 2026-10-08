{-# LANGUAGE BangPatterns #-}
module EscardoBench where
import qualified Escardo as E
import qualified EscardoVariants as V
import EscardoProbe (prefix,check)
import Data.Ratio
import Data.List (foldl',intercalate)
import Control.Monad
import Control.Exception
import System.Environment
import System.CPUTime
import System.Mem
import GHC.Stats

rotate n xs=drop k xs++take k xs where k=n `mod` length xs
periodic ds=(cycle ds, prefix (length ds) ds/(1-1/(2^length ds)))
input family bits index = case family of
 "dense" -> periodic(rotate index [-1,1,1,-1,1,1,1])
 "zero" -> let (x,q)=periodic(rotate index [-1,1,1,-1,1])
               n=bits `div` 3 in (replicate n 0++x,q/(2^n))
 "finite" -> let ds=take 7(cycle(rotate index [-1,1,1,0]))
              in (ds++E.zero,prefix 7 ds)
 "norm-dense" -> periodic(rotate index [-1,1,1])
 "norm-zero" -> periodic(rotate index [0,0,0,1])
 "norm-endpoint" -> periodic(rotate index [2,-2,2])
 _ -> error "family"
main=do
 [variant,family,bs,ss,rs]<-getArgs
 let bits=read bs;seed=read ss;reps=read rs
     run j=let (x,a)=input family bits(seed+j)
               (y,b)=input family bits(seed+j+3)
               (z,q)=case variant of
                "mul0" -> (E.mul_version0 x y,a*b)
                "mul1" -> (E.mul_version1 x y,a*b)
                "mul3" -> (E.mul_version3 x y,a*b)
                "fixed-mul2" -> (V.fixedMul x y,a*b)
                "mul1-self" -> (E.mul_version1 x x,a*a)
                "fixed-mul2-self" -> (V.fixedMul x x,a*a)
                "sqr" -> (E.sqr x,a*a)
                "normal" -> (E.divideBy2 x,a/2)
                "two-digit" -> (V.twoDigitDivide2 x,a/2)
                _ -> error "variant"
           in (take bits z,q)
 performGC
 before<-getRTSStats
 t0<-getCPUTime
 let results=map run [0..reps-1]
 checksum<-evaluate(foldl' (\a (ds,_)->foldl' (\b d->b*3+d) a ds) (0::Int) results)
 t1<-getCPUTime
 performGC
 after<-getRTSStats
 unless(all(\(ds,q)->length ds==bits && check bits ds q) results)(error "exact Rational enclosure")
 putStrLn(intercalate "\t" [variant,family,bs,ss,rs,
   show(fromIntegral(t1-t0)/1e12::Double),show(allocated_bytes after-allocated_bytes before),
   show(max_live_bytes after),show checksum])
