module Main where
import qualified Data.Number.Real as R
import qualified Data.Number.MPFR as D
import Control.Monad
main :: IO ()
main = forM_ [("Int",R.fromInt (2^40+1)),("Word",R.fromWord (2^40+1)),
              ("decimal",R.fromString "0.1")] $ \(name,x) ->
    forM_ [3,8,16] $ \digits -> case R.approx x digits of
      Right d -> putStrLn $ name++"\t"++show digits++"\tRight\t"++show (D.decompose d)
      Left (d,achieved) -> putStrLn $ name++"\t"++show digits++"\tLeft\t"++show (achieved,D.decompose d)
