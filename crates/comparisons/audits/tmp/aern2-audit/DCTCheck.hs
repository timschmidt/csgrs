{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Control.Monad (forM_, unless)
import Data.Ratio ((%), numerator, denominator)
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified DonorDCT as D
import System.Environment (getArgs)
import System.Exit (exitFailure)
import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))

sample :: Int -> Int -> [Rational]
sample n s = [toInteger ((17*s+11*k) `mod` 23-11)%8 | k <- [0..n-1]]
ball :: Integer -> Rational -> B.MPBall
ball p = B.mpBallP (B.prec p)
bounds :: B.MPBall -> (Rational,Rational)
bounds b = (c-e,c+e) where c=M.rational (B.ball_value b); e=M.rational (B.ball_error b)
has :: B.MPBall -> Rational -> Bool
has b x = let (l,u)=bounds b in l<=x && x<=u
report :: String -> [Bool] -> IO ()
report label bs = do
  putStrLn (label ++ " " ++ show (length (filter id bs),length bs))
  unless (and bs) exitFailure

-- Same input endpoint convention and interpolation scaling as lift2_DCT;
-- operate only on coefficient lists, not the obsolete ChPoly/range wrapper.
grid :: Integer -> Int -> [Rational] -> [B.MPBall]
grid p n cs = D.tDCT_I_nlogn (map (ball p) (take (n+1) (doubled++repeat 0)))
  where doubled=case cs of []->[]; c:rest -> 2*c:rest
interpolate :: Int -> [B.MPBall] -> [B.MPBall]
interpolate n ys = case map (M.mul (2%toInteger n)) (D.tDCT_I_nlogn ys) of
  c:rest -> M.divide c (2::Integer):rest
  [] -> []
productOracle :: [Rational] -> [Rational] -> [Rational]
productOracle a b = [sum [x*y/2 | (i,x)<-zip [0..] a, (j,y)<-zip [0..] b,
                                  k'<-[i+j,abs(i-j)],k'==k]
                     | k<-[0..length a+length b-2]]

numerical :: IO ()
numerical = forM_ [24,53,100] $ \p -> forM_ [2,4,8,16,32,64] $ \n -> do
  let identityCases=[(sample n s, s) | s<-[0..6]]
      productCases=[(sample (n `div` 2) s,sample (n `div` 2) (s+7)) | s<-[0..6]]
      -- Positive padding leaves the Nyquist term outside the asserted degree.
      ids=[and (zipWith has (take n (interpolate n (grid p n a))) a) | (a,_)<-identityCases]
      products=[and (zipWith has (interpolate n (zipWith M.mul (grid p n a) (grid p n b)))
                        (productOracle a b)) | (a,b)<-productCases]
  report ("DCT roundtrip p/n="++show (p,n)) ids
  report ("DCT product p/n="++show (p,n)) products

emit :: IO ()
emit = forM_ [24,53,100] $ \p -> forM_ [2,4,8,16,32] $ \n -> forM_ [0..4] $ \s -> do
  let inputs m=map (ball p) (sample m s)
  forM_ [("I-fast",D.tDCT_I_nlogn (inputs (n+1))),
         ("I-ref",D.tDCT_I_reference (inputs (n+1))),
         ("III-fast",D.tDCT_III_nlogn (inputs n)),
         ("III-ref",D._tDCT_III_reference (inputs n)),
         ("SDIII-fast",D.tSDCT_III_nlogn (inputs n)),
         ("SDIII-ref",D._tSDCT_III_reference (inputs n))] $ \(kind,ys) ->
    forM_ (zip [0::Int ..] ys) $ \(j,y) -> do
      let (l,u)=bounds y
      putStrLn (unwords [kind,show p,show n,show s,show j,
                         show (numerator l),show (denominator l),
                         show (numerator u),show (denominator u)])
main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args<-getArgs
  if args==["emit"] then emit else numerical
