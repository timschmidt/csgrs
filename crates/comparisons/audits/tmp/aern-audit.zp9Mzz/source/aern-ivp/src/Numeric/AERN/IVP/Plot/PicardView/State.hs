{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.IVP.Plot.PicardView.State
    Description :  internal state of the PicardView application
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for PicardView.
    Internal state of the PicardView application.
    
    NOT COMPLETED, CURRENTLY STALLED
-}
module Numeric.AERN.IVP.Plot.PicardView.State
--(
--)
where

import Numeric.AERN.IVP.Specification.ODE

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Misc.Debug

data PicardViewState =
    PicardViewState
    {
        pcrdvstComputed :: [Int],  
        pcrdvstRequested :: [Int]  
    }
    
type PicardViewFns f =
    [ -- list of segment information
     ([Domain f] -- initial values on this segment
     , 
     [ -- list of Picard iterations for this segment
      ([f], -- the solution enclosure produced by this iteration
       Bool -- whether this enclosure is within the previous enclosure (if any)
      )
     ]
     )
    ]
    
initState :: 
    (ArithInOut.RoundedReal (Domain f), 
     HasConstFns f)
    =>
    f ->
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    ODEIVP f -> 
    (PicardViewFns f, PicardViewState)
initState sampleF effReal ivp =
    ([(initialVals, [(mkInitialEnclosures sampleF effReal initialVals, False)])],
        PicardViewState
        {
            pcrdvstRequested = [1],
            pcrdvstComputed = [1]
        }
    )
    where
    initialVals = odeivp_makeInitialValueFnVec ivp

mkInitialEnclosures ::
    (ArithInOut.RoundedReal (Domain f), 
     HasConstFns f)
    =>
    f ->
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    [Domain f] ->
    [f]
mkInitialEnclosures sampleF effReal initialVals =
    map mkInitEncl initialVals
    where
    mkInitEncl initialVal =
        newConstFnFromSample sampleF $ initialVal <+> cm11
        where
        c1 = one sampleDom
        cm11 = (neg c1) </\> c1

    (</\>) = RefOrd.meetOutEff effJoinMeet
    (<+>) = ArithInOut.addOutEff effAdd

    effJoinMeet = ArithInOut.rrEffortJoinMeet sampleDom effReal
    effAdd =
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    sampleDom = getSampleDomValue sampleF
        