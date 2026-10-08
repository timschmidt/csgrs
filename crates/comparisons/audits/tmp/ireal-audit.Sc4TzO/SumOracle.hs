{-# LANGUAGE ForeignFunctionInterface #-}
module Main where
import Data.Number.IReal
import Data.Number.IReal.IReal (appr)
import Data.Number.IReal.IntegerInterval
import Data.Ratio
import Data.Bits
import Foreign.C.String
import Foreign.C.Types
import Control.Monad
import Control.Exception
import System.IO
foreign import ccall unsafe "ireal_sum_oracle" oracle :: CInt -> CULong -> CULong -> CString -> CString -> IO CInt
rat q=show(numerator q)++"/"++show(denominator q)
main=do
  hSetBuffering stdout LineBuffering
  forM_ [16,128,512] $ \n->forM_ [False,True] $ \dependent->do
    let xs=map (sin . fromRational) [((k+1) `mod` 97+1) % (k `mod` 29+31) | k<-[1..n]]
        inputs=if dependent then tail(scanl (+) 0 xs) else xs
    forM_ [("sum",sum),("bsum",bsum),("isum",isumN' n)] $ \(name,f)->do
      let I(l,u)=appr (f inputs) 128
      evaluate(l+u)
      status<-withCString (rat(l%bit 128)) $ \lo->withCString (rat(u%bit 128)) $ \hi->
        oracle (if dependent then 1 else 0) (fromInteger n) 1 lo hi
      putStrLn $ unwords [if status==0 then "PASS" else "FAIL",name,show dependent,show n,"width",show(u-l)]
      when (status/=0||u-l/=2) $ error "independent finite-sum qualification failed"
