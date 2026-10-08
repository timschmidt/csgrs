{-# LANGUAGE BangPatterns #-}
module FieldBench where
import qualified Main as Probe
import qualified Data.ByteString.Char8 as B
import Control.Exception (evaluate)
import Control.Monad (foldM)
import GHC.Clock (getMonotonicTimeNSec)
import System.CPUTime (getCPUTime)
import System.Environment (getArgs)

-- Read fresh bytes on every round: no Construct object, parsed expression,
-- or proved comparison is shared between repetitions. This deliberately
-- measures cached-file ingestion + construction + exact decision together.
roundOnce :: FilePath -> Int -> Int -> IO Int
roundOnce path !total _ = do
  bytes <- B.readFile path
  results <- mapM check (drop 1 (B.lines bytes))
  evaluate (total + sum results)
  where
    check line = case Probe.tabs (B.unpack line) of
      [_,_,want,lhs,rhs] -> do
        got <- evaluate (compare (Probe.parse lhs) (Probe.parse rhs))
        if show got == want then pure 1 else error "incorrect comparison"
      _ -> error "malformed row"
main :: IO ()
main = do
  [path,roundsText] <- getArgs
  let rounds = read roundsText :: Int
  if rounds <= 0 then error "rounds" else pure ()
  cpu0 <- getCPUTime
  wall0 <- getMonotonicTimeNSec
  total <- foldM (roundOnce path) 0 [1..rounds]
  wall1 <- getMonotonicTimeNSec
  cpu1 <- getCPUTime
  if total <= 0 then error "no checks" else pure ()
  putStrLn ("BENCH\t"++show rounds++"\t"++show total++"\t"++show ((cpu1-cpu0) `div` 1000)++"\t"++show (wall1-wall0))
