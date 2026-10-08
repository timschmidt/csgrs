{-# LANGUAGE DataKinds #-}
module Main where
import Data.CReal.Internal
import qualified Data.Bits as B
import Data.Ratio
import Control.Exception
import Control.Monad
import Data.IORef

type R = CReal 30
decimal :: String -> Rational
decimal s = let (negative,t)=case s of '-':xs -> (True,xs); _ -> (False,s)
                (whole,rest)=break (=='.') t
                fraction=case rest of []->[]; '.':xs->xs; _->error "decimal"
                n=read (whole++fraction)
            in (if negative then -n else n) % (10^length fraction)

main :: IO ()
main = do
  counts<-newIORef (0::Int,0::Int)
  forM_ [-50..50] $ \n -> forM_ [1,3,7,32,1024] $ \d -> forM_ [0,1,2,3,4,5,6,8,16,32,128] $ \p -> do
    let q=n%d
    forM_ [("nearest",fromRational q :: R),
           ("upward",crMemoize (\bits -> ceiling (q*fromInteger (B.bit bits))) :: R),
           ("downward",crMemoize (\bits -> floor (q*fromInteger (B.bit bits))) :: R)] $ \(mode,x) -> do
      let actual=decimal (showAtPrecision p x)
          ok=abs(actual-q)<=1%B.bit p
      (total,failures)<-readIORef counts
      writeIORef counts (total+1,failures+if ok then 0 else 1)
      when (not ok && failures<12) $ print ("SHOW FAIL",mode,q,p,actual,abs(actual-q))
  readIORef counts >>= print . (,) "SHOW total,failures"
  let point=crMemoize (\p -> ceiling ((1%1024)*fromInteger (B.bit p))) :: R
  print ("valid upward Cauchy input 1/1024",showAtPrecision 6 point)
  -- Evaluate rationalToDecimal separately against the exact decimal rounding
  -- error bound, independent of the CReal approximation preceding display.
  errors<-forM [(n%d,p) | n<-[-99..99],d<-[1,3,7,32,1024],p<-[0..10]] $ \(q,p) -> do
    actual<-evaluate (decimal (rationalToDecimal p q))
    pure (abs(actual-q) > 1%(2*10^p))
  print ("RATIONAL DECIMAL total,failures",length errors,length (filter id errors))
