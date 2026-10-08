module Main where

import Prelude hiding (EQ, LT, GT)
import Control.Monad (unless)
import Data.Bits ((.&.))
import Data.List (sort)
import qualified Data.Map.Strict as Map
import qualified Data.Set as Set
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Misc.Maybe ((&&?))

ensure :: String -> Bool -> IO ()
ensure label ok = unless ok (error label)

facts :: PartialOrdering -> [Bool]
facts rel = [rel == LT, rel == LT || rel == EQ, rel == EQ,
             rel == NC, rel == GT || rel == EQ, rel == GT]

info :: [Maybe Bool] -> PartialOrderingPartialInfo
info [a,b,c,d,e,f] = PartialOrderingPartialInfo a b c d e f
info _ = error "six fields required"

possible :: [Maybe Bool] -> [PartialOrdering]
possible fields = [r | r <- [EQ,LT,GT,NC],
    and (zipWith (\known actual -> maybe True (== actual) known) fields (facts r))]

subsetOrder :: Int -> Int -> PartialOrdering
subsetOrder a b
    | a == b = EQ
    | a .&. b == a = LT
    | a .&. b == b = GT
    | otherwise = NC

pairs :: [(Int,Int)]
pairs = [(a,b) | a <- [0..3], b <- [a+1..3]]

-- Independent preorder oracle: construct the complete <= relation matrix and
-- check reflexivity/transitivity, without the donor's consequence table.
isPreorder :: [((Int,Int),PartialOrdering)] -> Bool
isPreorder edges = and [not (le a b && le b c) || le a c |
    a <- [0..3], b <- [0..3], c <- [0..3]]
  where
    table = Map.fromList edges
    le a b | a == b = True
           | a < b = table Map.! (a,b) `elem` [LT,EQ]
           | otherwise = table Map.! (b,a) `elem` [GT,EQ]

main :: IO ()
main = do
    let records = sequence (replicate 6 [Nothing, Just False, Just True])
        consistent = [(f, possible f, sort (partialInfo2PartialOrdering (info f))) |
            f <- records, not (null (possible f))]
        lost = [(f,want,got) | (f,want,got) <- consistent, any (`notElem` got) want]
        extra = [(f,want,got) | (f,want,got) <- consistent, got /= want]
    ensure "partial-info dropped a possible relation" (null lost)
    putStrLn ("partial-info records total/consistent/over-approximated: " ++
        show (length records, length consistent, length extra))
    mapM_ print (take 4 extra)
    mapM_ (\r -> ensure "full-information roundtrip" $
        partialInfo2PartialOrdering (partialOrdering2PartialInfo (Just r)) == [r]) [EQ,LT,GT,NC]
    let triples = Set.fromList [(subsetOrder a b, subsetOrder b c, subsetOrder a c) |
            a <- [0..7], b <- [0..7], c <- [0..7]]
    ensure "triple table differs from powerset oracle" $
        triples == Set.fromList partialOrderingVariantsTriples
    mapM_ (\(a,b) -> ensure "consequence table differs from powerset oracle" $
        transitivityConsequences a b ==
        Set.fromList [c | (x,y,c) <- Set.toList triples, x == a, y == b])
        [(a,b) | a <- [EQ,LT,GT,NC], b <- [EQ,LT,GT,NC]]
    putStrLn ("powerset oracle triples / relation pairs: " ++ show (Set.size triples,16::Int))
    let oracle = Set.fromList [zip pairs rs | rs <- sequence (replicate 6 [EQ,LT,GT,NC]),
            isPreorder (zip pairs rs)]
        actual = pickConsistentOrderings (const True) [0..3::Int] []
    ensure "four-element preorder oracle" (Set.fromList actual == oracle)
    ensure "duplicate enumerated preorder" (length actual == Set.size oracle)
    -- Also test constraints and their reverse, each admitted subset of outcomes.
    let allowedSets = sequence (replicate 4 [False,True])
        constraints = [[r | (r,True) <- zip [EQ,LT,GT,NC] flags] | flags <- allowedSets]
    mapM_ (\(pair@(a,b),rels) -> do
        let expected = Set.filter (\edges -> lookup pair edges `elem` map Just rels) oracle
            forward = pickConsistentOrderings (const True) [0..3::Int] [(pair,rels)]
            backward = pickConsistentOrderings (const True) [0..3::Int]
                [((b,a),map partialOrderingTranspose rels)]
        ensure "forward constrained oracle" (Set.fromList forward == expected)
        ensure "reverse constrained oracle" (Set.fromList backward == expected))
        [(p,rs) | p <- pairs, rs <- constraints]
    putStrLn ("four-element preorders / directed constraint tests: " ++ show (Set.size oracle,192::Int))
    let weak = pOrdInfLT (partialOrderingPartialInfoAnd
            (partialOrderingPartialInfoAllNothing {pOrdInfLT = Just False})
            partialOrderingPartialInfoAllNothing)
    ensure "expected information-loss observation" (weak == Nothing)
    ensure "strong Kleene false and unknown" (Just False &&? Nothing == Just False)
    putStrLn "partial-info conjunction drops False AND Unknown; Misc.Maybe preserves it"
