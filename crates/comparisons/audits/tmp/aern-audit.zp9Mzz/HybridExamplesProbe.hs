module Main where

import Control.Exception (evaluate)
import Control.Monad (forM_, unless)
import qualified Data.Map as Map
import qualified Data.Set as Set
import Data.Ratio ((%))
import Numeric.AERN.Basics.Interval (Interval(..))
import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Interval.Double ()
import qualified Numeric.AERN.RefinementOrder as Q
import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Examples.Hybrid.Simple
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))

type D = Interval Double
type P = IntPoly String D

cf :: Rational -> D
cf q = Interval (fromRational q) (fromRational q)

sample :: P
sample = newConstFn limits [("t",Interval 0 1)] (cf 0)
  where
  limits = (defaultIntPolySizeLimits (cf 0) () 1)
    {ipolylimits_maxdeg=8,ipolylimits_maxsize=128}

check :: String -> Bool -> IO ()
check label pass = unless pass (error label)

contains :: D -> Rational -> Bool
contains (Interval l r) q = toRational l <= q && q <= toRational r

vectorContains :: [D] -> [Rational] -> Bool
vectorContains xs qs = length xs == length qs && and (zipWith contains xs qs)

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let catalog = ivpByNameMap sample
  forM_ (Map.toList catalog) $ \(name,problem) -> do
    let system=hybivp_system problem
        n=length (hybsys_componentNames system)
        fields=hybsys_modeFields system
        invariants=hybsys_modeInvariants system
    check (name++": mode keys") (Map.keysSet fields == Map.keysSet invariants)
    forM_ (Map.toList (hybivp_initialStateEnclosure problem)) $ \(mode,state) -> do
      check (name++": initial arity") (length state==n)
      case (invariants Map.! mode) state of
        Nothing -> error (name++": initial invariant empty")
        Just narrowed -> check (name++": invariant arity") (length narrowed==n)
      let fns=map (newConstFnFromSample sample) state
      check (name++": field arity") (length ((fields Map.! mode) fns)==n)
    forM_ (Map.keys fields) $ \mode ->
      forM_ (Map.elems (hybsys_eventSpecification system mode)) $ \(next,_,mask,_) -> do
        check (name++": target mode") (Map.member next fields)
        check (name++": mask arity") (length mask==n)
    putStrLn ("catalog shape/initial invariant: "++name++" PASS")

  let cases=[(x,fromIntegral a,fromIntegral b) | x<-[0,1%2,2],a<-[-4..4::Int],b<-[-4..a]]
      beadModels=[("energy",ivp2BeadColumnEnergy sample,False),
                  ("energy+velocity-difference",ivp2BeadColumnEnergyVDiff sample,True)]
  forM_ beadModels $ \(name,problem,withDiff) -> do
    let system=hybivp_system problem
        mode=HybSysMode "move"
        (_,reset,_,prune) = hybsys_eventSpecification system mode Map.! HybSysEventKind "bc2"
        inv=hybsys_modeInvariants system Map.! mode
    forM_ cases $ \(x,v1,v2) -> do
      let energy v=v*v+20*x
          before=[x,v1,energy v1,x,v2,energy v2]++[v1-v2 | withDiff]
          u1=(v1+3*v2)/4
          u2=(3*v1+v2)/4
          expected=[x,u1,energy u1,x,u2,energy u2]++[u1-u2 | withDiff]
          label=name++show (x,v1,v2)
      case prune (cf 0) (map cf before) of
        Nothing -> error (label++": excluded exact contact")
        Just contact -> do
          let after=reset contact
          check (label++": reset") (vectorContains after expected)
          case inv after of
            Nothing -> error (label++": post invariant empty")
            Just narrowed -> check (label++": post invariant") (vectorContains narrowed expected)
    putStrLn (name++": "++show (length cases)++" independent rational collision checks PASS")

  -- Execute the CLI checker formula using the actual donor state-union operation;
  -- this is not a build or execution of either GTK-linked CLI.
  let modeA=HybSysMode "A"
      modeB=HybSysMode "B"
      exact=Map.fromList [(modeA,[cf 0]),(modeB,[cf 1])]
      swapped=Map.fromList [(modeA,[cf 1]),(modeB,[cf 0])]
      eff=Q.joinmeetDefaultEffort (cf 0)
      (es,ev)=getHybridStateUnion eff exact
      (ss,sv)=getHybridStateUnion eff swapped
      refines a b=Q.pLeq b a == Just True
      refinesVec a b=and (zipWith refines a b)
      cliPass=es `Set.isSubsetOf` ss && refinesVec ev sv
      perModePass=all (\(mode,values)->refinesVec values (swapped Map.! mode)) (Map.toList exact)
  evaluate cliPass
  putStrLn ("CLI formula swapped modes: union check="++show cliPass++", mode-wise check="++show perModePass)
  putStrLn ("CLI formula missing vector: "++show (refinesVec [cf 1] []))
  check "mode union witness" (cliPass && not perModePass)
  check "zip truncation witness" (refinesVec [cf 1] [])
  putStrLn ("TOTAL: "++show (Map.size catalog)++" catalog checks, "++show (2*length cases)++" rational collision cases, 2 checker witnesses")
