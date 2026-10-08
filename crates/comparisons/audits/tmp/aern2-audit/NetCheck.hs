{-# LANGUAGE NoRebindableSyntax #-}
{-# LANGUAGE Arrows, TypeFamilies, FlexibleContexts #-}
module Main where
import Prelude
import Control.Arrow
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (unless)
import Data.Ratio ((%))
import AERN2.QA.Protocol
import AERN2.QA.Strategy.CachedUnsafe ()
import AERN2.QA.Strategy.Cached.Arrow
import AERN2.QA.Strategy.Parallel
import System.Exit (exitFailure)

-- A bounded dyadic protocol for one rational, independent of the old real
-- layers. Larger questions dominate smaller ones; cached numerators are
-- explicitly rescaled, so every returned answer must retain its own scale.
data DyadicProtocol = DyadicProtocol deriving Show
instance QAProtocol DyadicProtocol where
  type Q DyadicProtocol = Int
  type A DyadicProtocol = (Int,Integer)
instance QAProtocolCacheable DyadicProtocol where
  type QACache DyadicProtocol = Maybe (Int,Integer)
  newQACache _ = Nothing
  lookupQACache _ (Just (p,n)) q | p >= q =
    (Just (q,n `div` 2^(p-q)), Just "hit")
  lookupQACache _ _ _ = (Nothing,Just "miss")
  updateQACache _ q a old = case old of
    Just (p,_) | p >= q -> old
    _ -> Just a

queries :: [Int]
queries = [12,6,24,0,24,50,30,53,200,100]
node :: QAArrow a => QA a DyadicProtocol
node = newQA "22/7" [] DyadicProtocol (Just 12)
             (const (arr (\q -> (q,(22*2^q) `div` 7))))

sequential :: QAArrow a => [QARegOption] -> a () [(Int,Integer)]
sequential options = proc () -> do
  qa <- qaRegister options -< node
  go -< (qa,queries)
  where
  go = proc (qa,qs) -> case qs of
    [] -> returnA -< []
    q:rest -> do
      answer <- qaMakeQueryA Nothing -< (qa,q)
      later <- go -< (qa,rest)
      returnA -< answer:later

batch :: QAArrow a => [QARegOption] -> a () [(Int,Integer)]
batch options = proc () -> do
  qa <- qaRegister options -< node
  qaMakeQueriesA Nothing -< [(qa,q) | q <- queries]

check :: String -> [Int] -> [(Int,Integer)] -> IO ()
check label qs answers = do
  let correct = length answers == length qs && and
         [p == q && n % 2^p <= 22%7 && 22%7 < (n+1)%2^p
          | (q,(p,n)) <- zip qs answers]
  putStrLn (label ++ ": " ++ show correct ++ ", answers=" ++ show (length answers))
  unless correct exitFailure

logCounts :: String -> QANetLog -> IO ()
logCounts label lg = putStrLn (label ++ " log " ++ show
  [("creates", length [() | QANetLogCreate{} <- lg]),
   ("queries", length [() | QANetLogQuery{} <- lg]),
   ("answers", length [() | QANetLogAnswer{} <- lg]),
   ("hit descriptions", length [() | QANetLogAnswer _ _ desc _ <- lg,
                                     desc == "hit" || desc == "used cache (hit)"])])

main :: IO ()
main = do
  let pureAnswers = sequential [] ()
  evaluate (force pureAnswers) >>= check "pure cache sequential" queries
  let (cachedLog,cached) = executeQACachedA (sequential [])
      (uncachedLog,uncached) = executeQAUncachedA (sequential [])
      (_,cachedBatch) = executeQACachedA (batch [])
  check "stateful cache sequential" queries cached
  check "uncached sequential" queries uncached
  check "stateful cache batch" queries cachedBatch
  logCounts "cached" cachedLog
  logCounts "uncached" uncachedLog
  executeQAParA (sequential [QARegPreferSerial]) >>= check "STM serial sequential" queries
  executeQAParA (sequential [QARegPreferParallel]) >>= check "STM parallel sequential" queries
  executeQAParA (batch [QARegPreferParallel]) >>= check "STM parallel batch" queries
  (lg,answers) <- executeQAParAwithLog (sequential [QARegPreferParallel])
  check "STM logged sequential" queries answers
  logCounts "parallel" lg
