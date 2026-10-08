module Main where
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM, unless)
import Data.List (nub)
import System.Environment (getArgs)
import Numeric.IEEE.RoundMode
import Numeric.AERN.RealArithmetic.Basis.Double ()
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as R

inputs :: [Double]
inputs = nub $ filter (not . isInfinite) $
  [encodeFloat n e | n <- [-3,-1,0,1,3], e <- [-1074,-1073,-1022,-53,-52,-1,0,1,52,53,1022,1023]]
  ++ [0.1,0.2,0.3,1/3,encodeFloat (2^53-1) 971]

valid up exact value = not (isNaN value) &&
  if isInfinite value then if up then value > 0 else value < 0
  else if up then toRational value >= exact else toRational value <= exact

main = do
  args <- getArgs
  let mode = if args == ["up"] then Upward else ToNearest
      operations = [("add",(+),R.addDnEff (),R.addUpEff ()),
                    ("multiply",(*),R.multDnEff (),R.multUpEff ()),
                    ("divide",(/),R.divDnEff (),R.divUpEff ())]
  -- Fix the exact binary inputs before any donor operation changes rounding.
  values <- evaluate (force inputs)
  forM operations $ \(name,exactOp,dn,up) -> do
    let cases = [(a,b,upper) | a <- values,b <- values, name /= "divide" || b /= 0, upper <- [False,True]]
    results <- forM cases $ \(a,b,upper) -> do
      ok <- setRound mode
      unless ok (error "rounding mode unavailable")
      result <- evaluate ((if upper then up else dn) a b)
      finalMode <- getRound
      let exact = exactOp (toRational a) (toRational b)
      good <- evaluate (valid upper exact result)
      return (a,b,upper,result,finalMode,good)
    let bad = [r | r@(_,_,_,_,_,False) <- results]
    putStrLn (name ++ " initial=" ++ show mode ++ ": " ++ show (length results) ++
      " directed checks; invalid=" ++ show (length bad))
    mapM_ print (take 6 bad)
