module Main where
import qualified Data.Real.CReal as C
import qualified Data.Real.ICReal as I
import Data.Ratio
import Control.Monad
import System.IO
main :: IO ()
main=do
  hSetBuffering stdout LineBuffering
  forM_ [("exp",exp), ("sin",sin), ("cos",cos), ("atan",atan),
         ("sinh",sinh), ("cosh",cosh), ("asinh",asinh),
         ("erf",C.realErrorFunction), ("ln",log), ("sqrt",sqrt),
         ("asin",asin), ("acos",acos), ("atanh",atanh), ("acosh",acosh)] $ \(name,f) ->
    forM_ (filter (domain name) [-2,-1,-1/4,0,1/4,1,2]) $ \q ->
      forM_ [16,64,128] $ \bits -> do
        let z=C.approx(f(C.inject q)) (1%(2^bits))
        putStrLn $ name++"\t"++show(numerator q)++"\t"++show(denominator q)++"\t"++show bits++"\t"++show(numerator z)++"\t"++show(denominator z)
  where
    domain "ln" q=q>0
    domain "sqrt" q=q>0
    domain "acosh" q=q>1
    domain "asin" q=abs q<1
    domain "acos" q=abs q<1
    domain "atanh" q=abs q<1
    domain _ _=True
