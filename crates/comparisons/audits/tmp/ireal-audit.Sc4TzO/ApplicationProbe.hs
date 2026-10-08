{-# LANGUAGE DataKinds #-}
module Main where
import Data.Number.IReal
import Data.Number.IReal.IReal (appr)
import Data.Number.IReal.IntegerInterval
import qualified Data.Number.IReal.Rounded as R
import qualified FFT
import qualified FFTRounded as FR
import qualified ClenshawCurtis as C
import qualified ClenshawRounded as CR
import qualified Integrals as G
import qualified IntegralsRounded as GR
import qualified Newton as N
import qualified LinAlg as L
import Data.Complex
import Data.Bits
import Data.Ratio
import Control.Exception
import Control.Monad
import System.Timeout
import System.IO
import Data.IORef

check stats label expected x = do
  result<-try (timeout 3000000 $ do
     let I(l,u)=appr x 128
     evaluate (l+u)
     return (l%bit 128<=expected&&expected<=u%bit 128,l%bit 128,u%bit 128))
     :: IO (Either SomeException (Maybe (Bool,Rational,Rational)))
  let status=case result of
        Right (Just (True,_,_))->"PASS"
        Right (Just (False,_,_))->"FAIL"
        Right Nothing->"TIMEOUT"
        Left _->"EXCEPTION"
  modifyIORef' stats (status:)
  when (status/="PASS") $ putStrLn $ unwords [status,label,show expected,show result]
main = do
  hSetBuffering stdout LineBuffering
  stats<-newIORef []
  forM_ [2,4,8,16,32] $ \n-> do
    let inputs=[fromIntegral k :+ fromIntegral (k*k-3) | k<-[1..n]] :: [Complex IReal]
        output=FFT.ifft 80 (FFT.fft 80 inputs)
    forM_ (zip [1..n] output) $ \(k,r:+i)->do
      check stats ("fft-real "++show (n,k)) (fromIntegral k) r
      check stats ("fft-imag "++show (n,k)) (fromIntegral (k*k-3)) i
  putStrLn "DONE FFT round trips"
  forM_ [0..5] $ \n->forM_ [0..min (bit n) 12] $ \k->do
    let expected=if odd k then 0 else 2%fromIntegral (k+1)
    check stats ("quadrature "++show (n,k)) expected (C.quad (\x->pow x k) (C.chebpts n) (C.weights n))
  putStrLn "DONE quadrature monomials"
  forM_ [0..6] $ \k->do
    let expected=if odd k then 0 else 2%fromIntegral (k+1)
    check stats ("integral-polynomial "++show k) expected (G.integral 8 20 (\x->pow x k) (0+-1))
  putStrLn "DONE polynomial integrals"
  let roundedIntegral=GR.integral 4 10 (const (2 :: Dif (R.Rounded 40))) (0 R.+- 1 :: R.Rounded 40) :: R.Rounded 40
  putStrLn $ "Rounded integral constant 2 over [-1,1], expected 4: "++show roundedIntegral
  putStrLn $ "DCT length2 Double round trip "++show (FFT.idct 50 (FFT.dct 50 [1,2::Double]))
  putStrLn $ "rounded DCT length2 Double round trip "++show (FR.idct (FR.dct [1,2::Double]))
  forM_ [1..4] $ \n->do
    let matrix=[[if i==j then 4 else 1 | i<-[0..n-1]] | j<-[0..n-1]] :: [[IReal]]
        expected=[1..n]
        rhs=[fromIntegral (sum expected+3*k) | k<-expected]
        output=L.solve matrix rhs
    forM_ (zip expected output) $ \(k,x)->check stats ("linear-system "++show (n,k)) (fromIntegral k) x
  putStrLn "DONE linear systems"
  forM_ [-2,-1,0,1,2] $ \root->case N.newton 30 (\x->x-fromInteger root) ((fromInteger root-1) -+- (fromInteger root+1)) of
    Nothing->putStrLn "FAIL Newton missed affine root"
    Just x->check stats ("Newton affine "++show root) (fromInteger root) x
  counts<-readIORef stats
  forM_ ["PASS","FAIL","TIMEOUT","EXCEPTION"] $ \s->putStrLn ("TOTAL "++s++" "++show (length (filter (==s) counts)))
