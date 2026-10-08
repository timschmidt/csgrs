module Combinatorics where

factorials :: [Integer]
factorials = scanl (*) 1 [1..]

choose :: Int -> Int -> Integer
k `choose` i = 
  (product [k'-i'+1..k']) `div` (factorials!!i)
 where
  i' = fromIntegral i
  k' = fromIntegral k