module Main where
import Data.Real.Constructible (Construct,fromConstruct)
import Data.Ratio (numerator,denominator)
import Control.Exception (evaluate)
import System.IO (hSetBuffering,BufferMode(..),stdout)

emit :: (String,Int,Integer,Integer,Integer,Construct) -> IO ()
emit (family,index,sgn,p,q,x) = do
  a <- evaluate (fromConstruct (fromInteger sgn*x) :: Double)
  if isNaN a || isInfinite a then error "nonfinite donor view" else pure ()
  let r=toRational a
  putStrLn (unwords [family,show index,show sgn,show p,show q,show(numerator r),show(denominator r),show a])
main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let close=sum(map sqrt [7,14,39,70,72,76,85])-sum(map sqrt [13,16,46,55,67,73,79]) :: Construct
      pell=take 96 (iterate (\(p,q)->(p+2*q,p+q)) (1,1))
      rows=[("close",0,s,0,0,close) | s<-[-1,1]]++
           [("pell",i,s,p,q,fromInteger q*sqrt 2-fromInteger p) | (i,(p,q))<-zip [1..] pell,s<-[-1,1]]
  mapM_ emit rows
  putStrLn ("SUMMARY "++show(length rows))
