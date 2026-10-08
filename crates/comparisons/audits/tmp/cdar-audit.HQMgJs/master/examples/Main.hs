module Main where

import Data.CDAR
import ExamplesRump
--import Criterion.Main
--import Criterion.Config

main :: IO ()
main = do
  putStrLn . showA . require 50 . sin $ pi / 2
  examplesRump
