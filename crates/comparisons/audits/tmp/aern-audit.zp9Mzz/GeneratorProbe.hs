module Main where

import Prelude hiding (EQ, LT, GT)
import Control.Monad (unless)
import Test.QuickCheck (Gen, elements)
import Test.QuickCheck.Gen (unGen)
import Test.QuickCheck.Random (mkQCGen)
import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.NumericOrder

ensure :: String -> Bool -> IO ()
ensure label ok = unless ok (error label)

sampleAt :: Gen a -> Int -> a
sampleAt gen seed = unGen gen (mkQCGen seed) 30

main :: IO ()
main = do
    let lower = areaLinearAddLowerBound (5,False) $
            areaLinearAddLowerBound (0,False) (areaLinearWhole [] :: AreaLinear Int)
        upper = areaLinearAddUpperBound (5,False) $
            areaLinearAddUpperBound (10,False) (areaLinearWhole [] :: AreaLinear Int)
    ensure "expected lower-bound merge observation" (areaLinLowerBound lower == Just 0)
    ensure "expected upper-bound merge observation" (areaLinUpperBound upper == Just 10)
    putStrLn ("adding x>=5 to x>=0 retains " ++ show (areaLinLowerBound lower))
    putStrLn ("adding x<=5 to x<=10 retains " ++ show (areaLinUpperBound upper))
    let area strict = AreaLinear (Just 0) strict (Just 5) strict (areaWholeOnlyWhole [])
        pick chooseFn strict = sampleAt
            (arbitraryLinear (minBound,maxBound) succ pred chooseFn (area strict)) 0 :: Int
        strictEndpoints = (pick (pure . fst) True, pick (pure . snd) True)
        inclusiveEndpoints = (pick (pure . fst) False, pick (pure . snd) False)
    ensure "expected reversed strict flags" (strictEndpoints == (0,5) && inclusiveEndpoints == (1,4))
    putStrLn ("generator requested endpoints, strict/inclusive: " ++ show (strictEndpoints,inclusiveEndpoints))
    let forbiddenArea = areaWholeOnlyAddForbiddenValues [0::Int] (areaWholeOnlyWhole [0])
        samples = [sampleAt (arbitraryWhole forbiddenArea) s | s <- [0..4095]]
        forbiddenCount = length (filter (==0) samples)
    ensure "forbidden special-value escape observed" (forbiddenCount > 0)
    putStrLn ("forbidden zero samples / 4096 fixed seeds: " ++ show forbiddenCount)
    let Just gen = linearArbitraryTupleRelatedBy (elements [0::Int,1]) [1::Int,2]
            [((1,2),[LT])]
        tuples = [(s,sampleAt gen s) | s <- [0..4095]]
        incomplete = [(s,xs) | (s,xs) <- tuples, length xs /= 2]
    ensure "incomplete feasible strict tuple observed" (not (null incomplete))
    ensure "well-shaped tuples satisfy ordering" $
        and [a < b | (_, [a,b]) <- tuples]
    putStrLn ("feasible two-value LT tuples truncated / 4096 seeds: " ++ show (length incomplete))
    print (take 4 incomplete)
