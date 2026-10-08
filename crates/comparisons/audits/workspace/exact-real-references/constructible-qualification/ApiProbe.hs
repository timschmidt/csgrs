module ApiProbe where
import qualified Main as Probe
import Data.Real.Constructible (Construct,ConstructException(..),deconstruct)
import Data.Ratio ((%))
import Text.Read (readMaybe)
import Control.Exception (SomeException,evaluate,try,fromException)
import System.Environment (getArgs)
import System.Exit (exitWith,ExitCode(..))
import System.IO (hSetBuffering,BufferMode(..),stdout)

decomposition :: Int -> Construct -> Bool
decomposition 0 _ = error "deconstruction depth cap"
decomposition n x = case deconstruct x of
  Left r -> fromRational r == x
  Right (a,b,r) -> b/=0 && r>0 && x==a+b*sqrt r && all (decomposition(n-1)) [a,b,r]
checkValue :: String -> Construct -> [(String,Bool)]
checkValue label x = [(label++"-show",readMaybe(show x)==Just x),
  (label++"-negative-show",readMaybe(show(negate x))==Just(negate x)),
  (label++"-deconstruct",decomposition 512 x)]
enumRows :: [(String,Bool)]
enumRows = concat [
  [(show i++"-succ",succ x==x+1),(show i++"-pred",pred x==x-1),
   (show i++"-from",take 6 [x..]==[x+fromInteger n|n<-[0..5]]),
   (show i++"-then",take 6 [x,x+sqrt 3..]==[x+fromInteger n*sqrt 3|n<-[0..5]]),
   (show i++"-to",[x..x+5]==[x+fromInteger n|n<-[0..5]]),
   (show i++"-descending",[x,x-1..x-5]==[x-fromInteger n|n<-[0..5]]),
   (show i++"-zero-step",take 6 [x,x..x]==replicate 6 x)]
  | (i,x)<-zip [(0::Int)..] ([fromRational(n%2)|n<-[-8..8]]++[fromInteger s*sqrt(fromInteger n)|n<-[2,3,5],s<-[-1,1]] :: [Construct])]
run :: (String,Bool) -> IO Bool
run(label,value)=do
  answer<-try(evaluate value)::IO(Either SomeException Bool)
  let ok=either (const False) id answer
  putStrLn ((if ok then "PASS" else "FAIL")++"\t"++label++"\t"++either show show answer)
  pure ok
main :: IO ()
main=do
  hSetBuffering stdout LineBuffering
  [path]<-getArgs
  corpus<-readFile path
  let rows=concat [checkValue(group++"-"++label++"-lhs")(Probe.parse lhs)++checkValue(group++"-"++label++"-rhs")(Probe.parse rhs)
                 | line<-drop 1(lines corpus),let [group,label,_,lhs,rhs]=Probe.tabs line]
      syntax=[("precedence-add",readMaybe "1+2*3"==Just(7::Construct)),
              ("precedence-sub",readMaybe "10-3-2"==Just(5::Construct)),
              ("precedence-div",readMaybe "12/3/2"==Just(2::Construct)),
              ("precedence-neg",readMaybe "-2*3+4"==Just(-2::Construct)),
              ("precedence-root",readMaybe "sqrt (3+2*sqrt 2)"==Just(1+sqrt 2::Construct)),
              ("trailing-garbage",(readMaybe "1 nonsense"::Maybe Construct)==Nothing),
              ("list-read",readMaybe "[1, sqrt 2, -3/2]"==Just([1,sqrt 2,-3/2]::[Construct]))]
  good<-mapM run(rows++enumRows++syntax)
  exception<-try(evaluate(logBase (2::Construct) 8))::IO(Either SomeException Construct)
  let logBaseOk=case exception of Left e->fromException e==Just(Unconstructible "logBase");Right _->False
  putStrLn((if logBaseOk then "PASS" else "FAIL")++"\tunsupported-logBase")
  let passed=length(filter id good)+if logBaseOk then 1 else 0
  putStrLn("SUMMARY\t"++show passed++"\t"++show(length good+1))
  if and good&&logBaseOk then pure() else exitWith(ExitFailure 2)
