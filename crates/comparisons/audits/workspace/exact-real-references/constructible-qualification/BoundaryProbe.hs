module Main where

import Control.Exception (SomeException, evaluate, try)
import Data.Real.Constructible (Construct, fromConstruct)
import Data.Ratio ((%))
import System.Exit (exitWith, ExitCode(..))
import System.IO (hSetBuffering, BufferMode(..), stdout)

-- This is a boundary probe, not the donor's full native qualification.
-- Each claim is an exact identity/inequality or a finite-range requirement;
-- no floating approximation is used as the oracle for an exact decision.
cases :: [(String, Bool)]
cases =
  [("square-" ++ show n, (sqrt (fromInteger (n*n)) :: Construct) == fromInteger n) | n <- [0..40]] ++
  [("joined-field-" ++ show n, (sqrt (fromInteger (4*n)) :: Construct) == 2*sqrt (fromInteger n)) | n <- [1..40]] ++
  [("radical-denest", (sqrt (5+2*sqrt 6) :: Construct) == sqrt 2+sqrt 3),
   ("radical-sign", (sqrt (5-2*sqrt 6) :: Construct) == sqrt 3-sqrt 2)] ++
  [("fraction-" ++ show s ++ "-" ++ show n,
    let x = fromInteger s * sqrt (fromInteger n) :: Construct
        (whole, fraction) = properFraction x :: (Integer, Construct)
        reconstruction = fromInteger whole + fraction == x
        properSign = fraction == 0 || signum fraction == signum x
    in reconstruction && abs fraction < 1 && properSign)
   | s <- [-1,1], n <- [2,3,5,7,10]] ++
  [("rational-fraction-" ++ show s,
    let q = (3*s)%2; x = fromRational q :: Construct
        (whole,fraction) = properFraction x :: (Integer,Construct)
        (expected,rest) = properFraction q :: (Integer,Rational)
    in whole == expected && fraction == fromRational rest) | s <- [-1,1]] ++
  concat [[
    ("scaled-identity-" ++ show k,
      let scale = 10^k :: Integer
          x = sqrt (fromInteger (2*scale*scale)) / fromInteger scale :: Construct
      in x == sqrt 2),
    ("scaled-finite-" ++ show k,
      let scale = 10^k :: Integer
          x = sqrt (fromInteger (2*scale*scale)) / fromInteger scale :: Construct
          approx = fromConstruct x :: Double
      in not (isNaN approx || isInfinite approx) && approx > 1 && approx < 2)]
    | k <- [0,100,154,155,160,200,500]]

run :: (String, Bool) -> IO Bool
run (label, verdict) = do
  result <- try (evaluate verdict) :: IO (Either SomeException Bool)
  case result of
    Left e -> putStrLn ("ERROR\t" ++ label ++ "\t" ++ show e) >> pure False
    Right ok -> putStrLn ((if ok then "PASS\t" else "FAIL\t") ++ label) >> pure ok

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  good <- mapM run cases
  putStrLn ("SUMMARY\t" ++ show (length (filter id good)) ++ "\t" ++ show (length good))
  if and good then pure () else exitWith (ExitFailure 2)
