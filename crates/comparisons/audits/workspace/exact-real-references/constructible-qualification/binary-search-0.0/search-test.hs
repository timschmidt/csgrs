module Main (main) where

import Test.QuickCheck
import Numeric.Search.Bounded as B
import Numeric.Search.Integer as I
import Numeric.Search.Range

main :: IO ()
main = flip mapM_ tests $ \ (Test n t) -> do
	putStrLn $ "Testing: " ++ n
	t

data Test = Test String (IO ())

mkTest :: Testable a => String -> a -> Test
mkTest n t = Test n (test t)

tests :: [Test]
tests = [
	mkTest "searchIntegers" prop_searchIntegers,
	mkTest "searchIntegersFrom" prop_searchIntegersFrom,
	mkTest "searchIntegersTo" prop_searchIntegersTo,
	mkTest "searchIntegersTo (const False)" prop_searchIntegersToF,
	mkTest "searchFromTo" prop_searchFromTo,
	mkTest "searchFromTo (const False)" prop_searchFromToF,
	mkTest "searchBounded" prop_searchBounded,
	mkTest "searchBounded (const False)" prop_searchBoundedF,
	mkTest "searchBoundedFrom" prop_searchBoundedFrom,
	mkTest "searchBoundedFrom (const False)" prop_searchBoundedFromF,
	mkTest "searchBoundedTo" prop_searchBoundedTo,
	mkTest "searchBoundedTo (const False)" prop_searchBoundedToF]

-- Every upward closed predicate is equivalent to either (const False),
-- or (>= n) for some n.

prop_searchIntegers :: Integer -> Bool
prop_searchIntegers n =
	I.search (>= n)  ==  n

-- I.search (const False) does not terminate

--	I.searchFrom p l  ==  I.search (\ i -> i >= l && p i)

prop_searchIntegersFrom :: Integer -> Integer -> Bool
prop_searchIntegersFrom n l =
	I.searchFrom (>= n) l  ==  max l n

-- I.searchFrom (const False) l does not terminate

--	I.searchTo p h  ==  if n > h then Nothing else Just n
--		let k = I.search (\ i -> i > h || p i)
--		in if k <= h then Just k else Nothing

prop_searchIntegersTo :: Integer -> Integer -> Bool
prop_searchIntegersTo n h =
	I.searchTo (>= n) h  ==  if n <= h then Just n else Nothing

prop_searchIntegersToF :: Integer -> Bool
prop_searchIntegersToF h =
	I.searchTo (const False) h  ==  Nothing

--	searchFromTo p l h  ==  I.search (\ i -> i < l || i <= h && p i)

prop_searchFromTo :: Int -> Int -> Int -> Bool
prop_searchFromTo n l h =
	searchFromTo (>= n) l h  ==  if k <= h then Just k else Nothing
  where k = max n l

prop_searchFromToF :: Int -> Int -> Bool
prop_searchFromToF l h =
	searchFromTo (const False) l h  ==  Nothing

prop_searchBounded :: Int -> Bool
prop_searchBounded n =
	B.search (>= n)  ==  Just n

prop_searchBoundedF :: Bool
prop_searchBoundedF =
	B.search (const False :: Int -> Bool)  ==  Nothing

prop_searchBoundedFrom :: Int -> Int -> Bool
prop_searchBoundedFrom n l =
	B.searchFrom (>= n) l  ==  Just (max l n) 

prop_searchBoundedFromF :: Int -> Bool
prop_searchBoundedFromF l =
	B.searchFrom (const False) l  ==  Nothing

prop_searchBoundedTo :: Int -> Int -> Bool
prop_searchBoundedTo n h =
	B.searchTo (>= n) h  ==  if n <= h then Just n else Nothing

prop_searchBoundedToF :: Int -> Bool
prop_searchBoundedToF h =
	B.searchTo (const False) h  ==  Nothing
