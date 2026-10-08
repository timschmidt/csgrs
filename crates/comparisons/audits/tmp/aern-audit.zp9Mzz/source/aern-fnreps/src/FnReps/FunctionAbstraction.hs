module FnReps.FunctionAbstraction where

--import Numeric.AERN.MPFRBasis.Interval
import Numeric.AERN.DoubleBasis.Interval

type RA = DI

class (Floating fn) => RF fn where
    evalMI :: fn -> RA -> RA
    constFn :: RA -> fn
    idFn :: fn
    weierstrassFn :: fn
    primitiveFn :: fn -> fn
    


