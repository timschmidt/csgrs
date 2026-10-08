{-# LANGUAGE ScopedTypeVariables #-}
module Main where

import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM, forM_, unless)
import Data.List (nub)
import System.Environment (getArgs)
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)
import Numeric.IEEE.RoundMode
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps ()
import Numeric.AERN.Basics.Interval (Interval(..))
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import qualified Data.Number.MPFR as M
import Data.Number.MPFR.Instances.Up ()

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  let mode = if args == ["up"] then Upward else ToNearest
      raw = [encodeFloat n e | n <- [-3,-1,0,1,3],
              e <- [-1074,-1022,-53,-1,0,1,5,9,53,500,1022]]
            ++ [-1100,-1000,-750,-710,-709,-100,-10,-3.141592653589793,
                -1.5707963267948966,0.1,0.5,1,1.5707963267948966,
                3.141592653589793,10,100,709,710,750,1000,1100]
      operations = [("exp",R.expOut,M.exp,\x -> abs x <= 1100),
                    ("sqrt",R.sqrtOut,M.sqrt,\x -> x >= 0),
                    ("sin",R.sinOut,M.sin,const True),
                    ("cos",R.cosOut,M.cos,const True)]
  values <- evaluate (force (nub (filter (not . isInfinite) raw)))
  forM_ operations $ \(name,op,oracle,admit) -> do
    results <- forM (filter admit values) $ \x -> do
      -- MPFR explicit rounding does not depend on the hardware mode. Its
      -- 512-bit enclosure is used to distinguish proven failure from ambiguity.
      let input = M.fromDouble M.Near 512 x
          refL = toRational (oracle M.Down 512 input)
          refU = toRational (oracle M.Up 512 input)
      _ <- evaluate (force (refL,refU))
      ok <- setRound mode
      unless ok (error "rounding mode unavailable")
      result <- timeout 1000000
        (try (evaluate (force (let Interval l u = op (Interval x x) in (l,u))))
          :: IO (Either SomeException (Double,Double)))
      let detail = case result of
            Nothing -> "timeout"
            Just (Left e) -> "exception " ++ show e
            Just (Right (l,u))
              | isNaN l || isNaN u || l > u -> "invalid endpoints " ++ show (l,u)
              | lowerAbove l refU || upperBelow u refL -> "proven violation " ++ show (l,u)
              | lowerBelow l refL && upperAbove u refU -> "pass"
              | otherwise -> "oracle ambiguity " ++ show (l,u)
      unless (detail == "pass") (putStrLn (name ++ " x=" ++ show x ++ " " ++ detail))
      pure detail
    putStrLn (name ++ " initial=" ++ show mode ++ " cases=" ++ show (length results)
      ++ " passed=" ++ show (length (filter (=="pass") results)))
  where
    lowerAbove l q = if isInfinite l then l > 0 else toRational l > q
    upperBelow u q = if isInfinite u then u < 0 else toRational u < q
    lowerBelow l q = if isInfinite l then l < 0 else toRational l <= q
    upperAbove u q = if isInfinite u then u > 0 else toRational u >= q
