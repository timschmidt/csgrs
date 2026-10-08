module Main where

import Prelude
import Control.Monad (forM_, unless)
import Data.Ratio ((%), numerator, denominator)
import qualified Data.Map.Strict as Map
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified AERN2.MP.Float as F
import qualified AERN2.MP.Affine as A
import System.Environment (getArgs)
import System.Exit (exitFailure)

-- Inputs and point substitutions are dyadic exact rationals. Shared symbol
-- 101 gets the same substitution in both operands. No hash collisions are
-- sought or forced; this tests ordinary numerical enclosure propagation.
affine :: Integer -> Int -> Rational -> [(Int,Rational)] -> A.MPAffine
affine p cap c es = A.MPAffine (A.MPAffineConfig cap p) (flt c)
  (Map.fromList [(A.ErrorTermId k,flt e) | (k,e) <- es])
  where flt q = F.ceduCentre (F.fromRationalCEDU (B.prec p) q)

bounds :: A.MPAffine -> (Rational,Rational)
bounds a = (c-e,c+e)
  where c = M.rational (A.centre a)
        e = sum [abs (M.rational v) | v <- Map.elems (A.errTerms a)]

value :: A.MPAffine -> Map.Map A.ErrorTermId Rational -> Rational
value a env = M.rational (A.centre a) +
  sum [M.rational c * (env Map.! k) | (k,c) <- Map.toList (A.errTerms a)]

encloses :: (Rational,Rational) -> Rational -> Bool
encloses (l,u) x = l <= x && x <= u

assignments :: [Map.Map A.ErrorTermId Rational]
assignments = [Map.fromList (zip (map A.ErrorTermId [101,102,103]) [x,y,z])
              | x <- vals, y <- vals, z <- vals]
  where vals = [-1,-1%2,0,1%2,1]

inputs :: [(Integer,Int,A.MPAffine,A.MPAffine)]
inputs = [(p,cap,affine p cap x [(101,1%4),(102,-1%8)],
                 affine p cap y [(101,-1%8),(103,1%16)])
          | p <- [10,53,100], cap <- [1,2,4],
            x <- [-5%2,-1%2,0,1%2,5%2], y <- [-5%2,-1%2,1%2,5%2]]

check :: String -> [(String,Bool)] -> IO ()
check name xs = do
  let bad = [s | (s,False) <- xs]
  putStrLn (name ++ ": " ++ show (length xs-length bad) ++ "/" ++ show (length xs))
  forM_ (take 4 bad) putStrLn
  unless (null bad) exitFailure

numerical :: IO ()
numerical = do
  forM_ [("add",M.add,(+)),("sub",M.sub,(-)),("mul",M.mul,(*)),("div",M.divide,(/))] $
    \(name,op,exact) -> check ("correlated affine " ++ name)
      [(show (p,cap,i,j), encloses result (exact (value a env) (value b env)))
       | (i,(p,cap,a,b)) <- zip [0::Int ..] inputs,
         let result = bounds (op a b), (j,env) <- zip [0::Int ..] assignments]
  check "affine normalization"
    [(show (p,cap,i,j), encloses result (value a env))
     | (i,(p,cap,a,_)) <- zip [0::Int ..] inputs,
       let result = bounds (A.mpAffNormalise a), (j,env) <- zip [0::Int ..] assignments]
  check "affine exact shared subtraction"
    [(show (p,cap,i), bounds (M.sub a a) == (0,0))
     | (i,(p,cap,a,_)) <- zip [0::Int ..] inputs]
  check "affine lower precision"
    [(show (cap,i,j), encloses result (value a env))
     | (i,(_,cap,a,_)) <- zip [0::Int ..] inputs,
       let result = bounds (B.setPrecision (B.prec (5::Integer)) a),
       (j,env) <- zip [0::Int ..] assignments]
  forM_ [("exp",M.exp),("sqrt",M.sqrt),("sin",M.sin),("cos",M.cos)] $ \(name,op) -> do
    let a = affine 53 2 (1%2) [(101,1%16),(102,1%32)]
        result = op a
    putStrLn ("term cap diagnostic " ++ name ++ ": " ++ show (A.maxTerms (A.config result), Map.size (A.errTerms result)))

exportElementary :: IO ()
exportElementary = forM_ [10,53,100] $ \p ->
  forM_ [1,2,4] $ \cap ->
    forM_ [("sqrt",M.sqrt),("exp",M.exp),("sin",M.sin),("cos",M.cos)] $ \(name,op) ->
      forM_ [-5%2,-1%2,0,1%2,5%2] $ \c ->
        unless (name == "sqrt" && c <= 0) $ do
          let a = affine p cap c [(101,1%4),(102,-1%8)]
              (l,u) = bounds (op a)
          forM_ [env | env <- assignments, env Map.! A.ErrorTermId 103 == 0] $ \env -> do
            let x = value a env
            putStrLn (unwords [name,show p,show (numerator x),show (denominator x),
                              show (numerator l),show (denominator l),show (numerator u),show (denominator u)])

main :: IO ()
main = do
  args <- getArgs
  case args of
    ["export"] -> exportElementary
    _ -> numerical
