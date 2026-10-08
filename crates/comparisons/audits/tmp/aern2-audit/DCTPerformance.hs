{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM_, unless, when)
import Data.IORef
import Data.List (sort)
import Data.Ratio ((%))
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified DonorDCT as D
import GHC.Clock (getMonotonicTimeNSec)
import System.Environment (getArgs)
import System.Exit (exitFailure,exitSuccess)

sample :: Int -> Int -> [Rational]
sample n s=[toInteger ((17*s+11*k) `mod` 23-11)%8 | k<-[0..n]]
{-# NOINLINE kernel #-}
kernel :: String -> [B.MPBall] -> [B.MPBall]
kernel method a = if method=="ref" then D.tDCT_I_reference a else D.tDCT_I_nlogn a
bounds :: B.MPBall -> (Rational,Rational)
bounds b = (c-e,c+e) where c=M.rational (B.ball_value b); e=M.rational (B.ball_error b)
has b x = let (l,u)=bounds b in l<=x && x<=u
main :: IO ()
main = do
  [mode,method,sp,sn,sc]<-getArgs
  let p=read sp; n=read sn; count=read sc::Int
      originals=[sample n s | s<-[0..12]]
      inputs=map (map (B.mpBallP (B.prec p))) originals
  when (mode=="validate") $ do
    let checks=[and (zipWith has (map (M.mul (2%toInteger n))
                    (kernel meth (kernel meth x))) exact)
                | (x,exact)<-zip inputs originals,meth<-["ref","fast"]]
        radiusRat y = let (l,u)=bounds y in (u-l)/2
        radii meth=concatMap (map radiusRat . kernel meth) inputs
        ratios=[fromRational (f/r)::Double | (r,f)<-zip (radii "ref") (radii "fast"),r>0]
    print (length (filter id checks),length checks)
    print ("fast/ref radius median/max",sort ratios!!(length ratios `div` 2),maximum ratios)
    print [(meth, minimum (concatMap (map B.getAccuracy . kernel meth) inputs)) | meth<-["ref","fast"]]
    unless (and checks) exitFailure
    exitSuccess
  evaluate (force inputs)
  state<-newIORef (cycle inputs)
  let one=do
        x:rest<-readIORef state
        writeIORef state rest
        evaluate (force (kernel method x))
  forM_ [1..13::Int] (const one)
  start<-getMonotonicTimeNSec
  forM_ [1..count] (const one)
  stop<-getMonotonicTimeNSec
  print (fromIntegral (stop-start)/fromIntegral count::Double)
