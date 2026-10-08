{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}

import Control.Exception (catch, SomeException)
import Data.Proxy (Proxy(..))
import System.Exit (exitSuccess, exitFailure)

import Numeric.LongDouble (LongDouble)

import Numeric.Rounded

filename :: String
filename = "test.txt"

main :: IO ()
main = do
  golden <- readFile filename `Control.Exception.catch` update
  if golden == test then exitSuccess else exitFailure

update :: SomeException -> IO String
update _ = writeFile filename test >> readFile filename

pf :: RealFrac a => a -> (Integer, a)
pf = properFraction

test :: String
test = unlines
  [ show (exp pi :: Rounded TowardZero 512)
  , show (pi :: Rounded TowardZero Double)
  , show (pi :: Rounded AwayFromZero Double)
  , show (kCatalan :: Rounded TowardZero 128)
  , (reifyPrecision 512 (\(_ :: Proxy p) -> show (logBase 10 2 :: Rounded TowardNearest p)))
  , (reifyRounding TowardZero (\(_ :: Proxy r) -> show (logBase 10 2 :: Rounded r 512)))
  , show (fromDouble pi - pi :: Rounded TowardNearest 64)
  , show (fromInt 100000000 :: Rounded TowardNearest Float)
  , show (fromInt 123456789 :: Rounded TowardNearest Float)
  , show (realToFrac (pi :: Rounded TowardNearest 512) :: Double)
  , show . pf $ (-2.5 :: Rational)
  , show . pf $ (-1.5 :: Rational)
  , show . pf $ (-0.5 :: Rational)
  , show . pf $ ( 0.5 :: Rational)
  , show . pf $ ( 1.5 :: Rational)
  , show . pf $ ( 2.5 :: Rational)
  , show . pf $ (-2.5 :: Rounded TowardNearest Float)
  , show . pf $ (-1.5 :: Rounded TowardNearest Float)
  , show . pf $ (-0.5 :: Rounded TowardNearest Float)
  , show . pf $ ( 0.5 :: Rounded TowardNearest Float)
  , show . pf $ ( 1.5 :: Rounded TowardNearest Float)
  , show . pf $ ( 2.5 :: Rounded TowardNearest Float)
  , show . pf $ (-(2^23 + 0.5) :: Rounded TowardNearest Float)
  , show . pf $ (-(2^22 + 0.5) :: Rounded TowardNearest Float)
  , show . pf $ (-(2^21 + 0.5) :: Rounded TowardNearest Float)
  , show . pf $ ( (2^21 + 0.5) :: Rounded TowardNearest Float)
  , show . pf $ ( (2^22 + 0.5) :: Rounded TowardNearest Float)
  , show . pf $ ( (2^23 + 0.5) :: Rounded TowardNearest Float)
  , show (fromLongDouble pi == (pi :: Rounded TowardNearest LongDouble))
  , show (pi == toLongDouble (pi :: Rounded TowardNearest LongDouble))
  ]
