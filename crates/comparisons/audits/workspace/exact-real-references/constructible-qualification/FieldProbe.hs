module Main where
import Control.Exception (SomeException,evaluate,try)
import Data.Real.Constructible (Construct)
import Data.Ratio ((%))
import System.Environment (getArgs)
import System.Exit (exitWith,ExitCode(..))
import System.IO (hSetBuffering,BufferMode(..),stdout)
import System.CPUTime (getCPUTime)

expr :: [String] -> (Construct,[String])
expr ("q":n:d:rest) = (fromRational (read n % read d),rest)
expr ("n":rest) = let (x,r)=expr rest in (negate x,r)
expr ("s":rest) = let (x,r)=expr rest in (sqrt x,r)
expr (op:rest) | op `elem` ["+","*","/"] =
  let (x,r)=expr rest; (y,r')=expr r
  in ((case op of "+" -> (+); "*" -> (*); "/" -> (/); _ -> error "op") x y,r')
expr _ = error "malformed expression"
parse :: String -> Construct
parse s = case expr (words s) of (x,[]) -> x; _ -> error "trailing tokens"
tabs :: String -> [String]
tabs s = let (a,b)=break (=='\t') s in a:case b of [] -> []; (_:r) -> tabs r

run :: String -> IO Bool
run row = case tabs row of
  [group,label,want,lhs,rhs] -> do
    start <- getCPUTime
    answer <- try (evaluate (compare (parse lhs) (parse rhs))) :: IO (Either SomeException Ordering)
    end <- getCPUTime
    case answer of
      Left e -> putStrLn ("ERROR\t"++group++"\t"++label++"\t"++show e) >> pure False
      Right got -> do
        let ok=show got==want
        putStrLn ((if ok then "PASS" else "WRONG")++"\t"++group++"\t"++label++"\t"++show got++"\t"++show (end-start))
        pure ok
  _ -> error "malformed row"
main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  [path,group] <- getArgs
  corpus <- readFile path
  let rows=filter (\r -> group=="all" || head (tabs r)==group) (drop 1 (lines corpus))
  if null rows then error "empty group" else pure ()
  good <- mapM run rows
  putStrLn ("SUMMARY\t"++show (length (filter id good))++"\t"++show (length good))
  if and good then pure () else exitWith (ExitFailure 2)
