module Main where
import Control.Monad (unless)
import Data.Bits (testBit)
import Numeric.AERN.RealArithmetic.Interval.FieldOps (multiplyIntervals)

run :: Bool -> Int -> (Int,Int) -> (Int,Int) -> (Int,Int)
run inward mask = multiplyIntervals
    (signs 0) (consistent 2) (signs 4) (consistent 6)
    (*) (*) min min max max
    (if inward then max else min) (if inward then min else max)
  where
    known bit truth = if testBit mask bit then Just truth else Nothing
    signs bit x = (known bit (x >= 0), known (bit+1) (x <= 0))
    consistent bit (l,r) = (known bit (l <= r), known (bit+1) (r <= l))

oracle :: (Int,Int) -> (Int,Int) -> (Int,Int)
oracle (a,b) (c,d) = (minimum corners, maximum corners)
  where corners = [a*c,a*d,b*c,b*d]
ensure name ok = unless ok (error name)
main :: IO ()
main = do
    let intervals = [(l,r) | l <- [-2..2], r <- [-2..2], l <= r]
        cases = [(m,a,b) | m <- [0..255], a <- intervals, b <- intervals]
        badOut = [(m,a,b,out,exact) | (m,a,b) <- cases,
                   let out@(lo,hi) = run False m a b,
                   let exact@(elo,ehi) = oracle a b, lo > elo || hi < ehi]
        badIn = [(m,a,b,inn,exact) | (m,a,b) <- cases,
                  let inn@(lo,hi) = run True m a b,
                  let exact@(elo,ehi) = oracle a b, lo < elo || hi > ehi]
        known = [(a,b) | a <- intervals, b <- intervals]
    ensure "all-known product table agrees with four-corner oracle"
      (all (\(a,b) -> run False 255 a b == oracle a b && run True 255 a b == oracle a b) known)
    putStrLn ("all-unknown sample result: " ++ show (run False 0 (-2,-1) (-2,-1)))
    putStrLn ("full-information cases: " ++ show (length known) ++ " pass outward/inward")
    putStrLn ("partial-information cases: " ++ show (length cases)
      ++ "; outward violations: " ++ show (length badOut)
      ++ "; inward violations: " ++ show (length badIn))
    print (take 4 badOut)
    print (take 4 badIn)
    ensure "unknown-sign four-corner product is correct" (run False 0 (-2,-1) (-2,-1) == (1,4))
    ensure "all outward enclosures contain the exact product" (null badOut)
    ensure "all inward enclosures lie within the exact product" (null badIn)
    putStrLn "All consistent-interval checks pass; no product defect reproduced."
