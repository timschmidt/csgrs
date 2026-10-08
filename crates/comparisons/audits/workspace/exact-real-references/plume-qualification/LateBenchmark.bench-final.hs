module Main where
import Control.Exception (evaluate)
import Control.Monad (unless)
import Data.Ratio
import GHC.Stats
import System.CPUTime (getCPUTime)
import System.Environment (getArgs)
import System.Mem (performGC)
import qualified SBinDec as D
import qualified Tests as T

pow2 e | e>=0 = fromInteger (2^e)
       | otherwise = 1 % (2^(-e))
prefix = foldr (\d r -> (fromIntegral d+r)/2) 0
main = do
  [variant,input,kText,pText]<-getArgs
  let k=read kText::Int; p=read pText::Int
      initial=case input of "0.1"->1%10; "0.5467"->5467%10000; _->error "input"
      op=case variant of "sb-float"->T.log_mapSB; "dy-float"->T.log_mapDyWrap; _->error "variant"
  performGC
  before<-getRTSStats
  begin<-getCPUTime
  let (e,m)=op (D.decSbf input) k
      n=fromInteger (max 1 (toInteger p+e))
      ds=take n m
  checksum<-evaluate (e+toInteger (length ds+sum ds))
  end<-getCPUTime
  performGC
  after<-getRTSStats
  let want=iterate (\x->4*x*(1-x)) initial!!k
      center=pow2 e*prefix ds
      radius=pow2 (e-toInteger n)
  unless (n<=4096&&length ds==n&&all (\d->abs d<=1) ds&&abs(center-want)<=radius) $
    error "numerical qualification failed"
  putStrLn (unwords [variant,input,show k,show p,show(end-begin),
    show(allocated_bytes after-allocated_bytes before),show checksum,"PASS"])
