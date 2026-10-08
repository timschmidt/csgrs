{-# LANGUAGE DataKinds #-}
module Main where
import Prelude
import Control.Monad (forM_, unless)
import Data.Ratio ((%),numerator,denominator)
import Data.Proxy (Proxy(..))
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified ERC.Examples as E
import System.Environment (getArgs)
import System.Exit (exitFailure)

bounds :: B.MPBall -> (Rational,Rational)
bounds b = (c-e,c+e)
  where c = M.rational (B.ball_value b)
        e = M.rational (B.ball_error b)
encloses :: B.MPBall -> Rational -> Bool
encloses b x = l<=x && x<=u where (l,u)=bounds b

check :: String -> [Bool] -> IO ()
check name xs = do
  putStrLn (name ++ ": " ++ show (length (filter id xs)) ++ "/" ++ show (length xs))
  unless (and xs) exitFailure

numerical :: IO ()
numerical = do
  forM_ [("linear",E.run_erc_Round1),("logarithmic",E.run_erc_Round2)] $ \(name,op) ->
    check ("ERC nearby integer "++name)
      [let k=op x in k==floor x || k==ceiling x | n<-[-100,-11,-1,0,1,11,100],d<-[1,2,3,7],let x=n%d]
  check "ERC Heron sqrt enclosure"
    [let (l,u)=bounds (E.run_erc_HeronSqrt x p)
     in u>=0 && u*u>=x && (l<=0 || l*l<=x)
     | x<-[1%19,1%3,1,2,7,100],p<-[10,30,80]]
  check "ERC trisection sqrt enclosure"
    [let (l,u)=bounds (E.run_erc_Root_sqrt x p)
     in u>=0 && u*u>=x && (l<=0 || l*l<=x)
     | x<-[1%7,1%2,6%7],p<-[10,20]]
  let recurrence n = fst (iterate step (11%2,61%11) !! n)
      step (a,b)=(b,111-(1130-3000/a)/b)
  check "ERC Muller recurrence rational oracle"
    [encloses (E.run_erc_JMMuller (toInteger n) p) (recurrence n)
     | n<-[0,1,2,5,10,25],p<-[10,40]]
  check "ERC nonsingular determinant rational oracle"
    [encloses (E.run_erc_Det (Proxy::Proxy 2) [[1,2],[3,5]] 20) (-1),
     encloses (E.run_erc_Det (Proxy::Proxy 3) [[1,2,1],[1,2,0],[2,2,1]] 20) (-2)]

exportElementary :: IO ()
exportElementary = do
  forM_ [("sqrt",E.run_erc_HeronSqrt),("exp",E.run_erc_Exp)] $ \(name,op) ->
    forM_ [1%19,1%3,1,2,7] $ \x -> forM_ [10,30,80] $ \p -> do
      let (l,u)=bounds (op x p)
      putStrLn (unwords [name,show p,show (numerator x),show (denominator x),
                        show (numerator l),show (denominator l),show (numerator u),show (denominator u)])

main :: IO ()
main = do
  args<-getArgs
  case args of
    ["export"] -> exportElementary
    _ -> numerical
