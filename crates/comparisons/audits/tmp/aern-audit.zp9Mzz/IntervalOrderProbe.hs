{-# LANGUAGE TypeFamilies #-}
module Main where
import Control.Monad (unless)
import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.Interval.RefinementOrder (intervalApproxIncludedIn)
import qualified Numeric.AERN.NumericOrder as N
import qualified Numeric.AERN.RefinementOrder as R
import qualified Numeric.AERN.Basics.PartialOrdering as P

newtype Endpoint = E Int deriving (Eq, Ord, Show)
instance N.PartialComparison Endpoint where
    type PartialCompareEffortIndicator Endpoint = ()
    pCompareDefaultEffort _ = ()
    pCompareEff _ (E a) (E b) = N.pCompareEff () a b
instance N.RoundedLatticeEffort Endpoint where
    type MinmaxEffortIndicator Endpoint = ()
    minmaxDefaultEffort _ = ()
interval (a,b) = Interval (E a) (E b)
inside (l1,r1) (l2,r2) = l2 <= l1 && r1 <= r2
ensure name ok = unless ok (error name)

main :: IO ()
main = do
    let endpoints = [-2..2]
        allIntervals = [(a,b) | a <- endpoints, b <- endpoints]
        pairs = [(a,b) | a <- allIntervals, b <- allIntervals]
        refOrder a b
          | a == b = P.EQ
          | inside b a = P.LT
          | inside a b = P.GT
          | otherwise = P.NC
        wrongRef = [(a,b) | (a,b) <- pairs,
            R.pCompare (interval a) (interval b) /= Just (refOrder a b)]
    ensure "exact endpoint refinement order" (null wrongRef)
    putStrLn ("refinement order: " ++ show (length pairs) ++ " exact checks passed")
    let validIntervals = [(a,b) | (a,b) <- allIntervals, a <= b]
        approximations = [(o,i) | o <- validIntervals, i <- validIntervals, inside i o]
        value (o,i) = IntervalApprox (interval o) (interval i)
        possibilities ((ol,or'),(il,ir)) = [(l,r) | l <- [ol..il], r <- [ir..or']]
        claims = [(a,b,intervalApproxIncludedIn (value a) (value b))
                 | a <- approximations, b <- approximations]
        unsound = [(a,b,claim) | (a,b,Just claim) <- claims,
                    or [inside x y /= claim | x <- possibilities a, y <- possibilities b]]
    ensure "false inclusion certificate is reproduced" (not (null unsound))
    putStrLn ("interval approximation inclusion: " ++ show (length unsound)
       ++ " unsound certificates among " ++ show (length claims) ++ " pairs")
    print (take 3 unsound)
    let a = IntervalApprox (interval (0,10)) (interval (1,2))
        b = IntervalApprox (interval (0,5)) (interval (0,5))
    ensure "false refutation from only loose outer bound"
        (intervalApproxIncludedIn a b == Just False && inside (1,2) (0,5))
    putStrLn "loose outer [0,10], inner [1,2], compared with exact [0,5]: falsely refuted"
