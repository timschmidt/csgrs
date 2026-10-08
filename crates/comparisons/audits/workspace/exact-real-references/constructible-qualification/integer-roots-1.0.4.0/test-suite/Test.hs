import Test.Tasty
import Test.Tasty.QuickCheck
import Test.Tasty.SmallCheck

import qualified Math.NumberTheory.Roots.CubesTests as Cubes
import qualified Math.NumberTheory.Roots.FourthTests as Fourth
import qualified Math.NumberTheory.Roots.GeneralTests as General
import qualified Math.NumberTheory.Roots.SquaresTests as Squares

main :: IO ()
main
  = defaultMain
  $ adjustOption
    (\(QuickCheckTests n) -> QuickCheckTests (max n 10000))
  $ adjustOption
    (\(SmallCheckDepth n) -> SmallCheckDepth (max n 100))
  $ tests

tests :: TestTree
tests = testGroup "All"
  [ Squares.testSuite
  , Cubes.testSuite
  , Fourth.testSuite
  , General.testSuite
  ]
