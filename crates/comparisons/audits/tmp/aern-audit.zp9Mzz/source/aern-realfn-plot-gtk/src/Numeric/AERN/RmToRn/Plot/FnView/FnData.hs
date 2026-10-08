{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.FnView.FnData
    Description :  data for controlling the functions shown
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for FnView.
    Data for controlling the functions shown.
-}
module Numeric.AERN.RmToRn.Plot.FnView.FnData
--(
--)
where

import Numeric.AERN.RmToRn.Plot.Params

import Numeric.AERN.RmToRn.Domain

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RefinementOrder as RefOrd


{-|
    Two transactional variables with values of the following two types
    will be used by the client(s) to communicate to the viewer what it
    should be showing.
-}
data FnData f =
    FnData
    {
        dataFns :: [[(GraphOrParamPlotFn f, Var f)]] -- ^ groups of (piece-wise) functions to plot
    }
    
data GraphOrParamPlotFn f
    = GraphPlotFn [f]
    | ParamPlotFns [(f,f)] [Var f]
    
instance (Show f) => Show (GraphOrParamPlotFn f)
    where
    show (GraphPlotFn v) = show v
    show (ParamPlotFns v _scanVars) = show v

data FnMetaData f =
    FnMetaData
    {
        dataDestroyed :: Bool, -- ^ command to destroy OR signal that user destroyed
        dataFnsUpdated :: Bool, -- ^ avoid checking functions for equality
        dataDomName :: String, -- ^ name of the domain variable (eg "t")
        dataDomL :: Domain f, -- ^ left endpoint of the global domain
        dataDomR :: Domain f, -- ^ right endpoint of the global domain
        dataValLO :: Domain f, -- ^ lower bounds for values of all functions
        dataValHI :: Domain f, -- ^ upper bounds for values of all functions
        dataFnNames :: [[String]], -- ^ a name for each function
        dataFnGroupNames :: [String],
        dataFnStyles :: [[FnPlotStyle]],
        dataDefaultActiveFns :: [[Bool]],
        dataDefaultEvalPoint :: Domain f, -- ^ show the values of the functions at this point
        dataDefaultEvalPointName :: String, -- ^ label to put on the button
        dataDefaultCanvasParams :: CanvasParams (Domain f)
    }
    
instance 
    (RefOrd.PartialComparison (Domain f)) 
    => 
    (Eq (FnMetaData f))
    where
    fnmeta1 == (fnmeta2 :: FnMetaData f) =
        equalComp dataDestroyed
        &&
        equalComp dataFnsUpdated
        &&
        equalComp dataDomName
        &&
        equalComp dataFnNames
        &&
        equalComp dataFnStyles
        &&
        equalComp dataDefaultActiveFns
        &&
        equalComp dataDefaultEvalPointName
        &&
        equalComp dataDefaultCanvasParams
        &&
        (and $ map equalCompDF [dataDomL, dataDomR, dataValLO, dataValHI, dataDefaultEvalPoint])
        where
        equalComp :: (Eq a) => (FnMetaData f -> a) -> Bool
        equalComp comp = (comp fnmeta1) == (comp fnmeta2)
        equalCompDF :: (FnMetaData f -> (Domain f)) -> Bool
        equalCompDF comp = (comp fnmeta1) `eq` (comp fnmeta2)
        a `eq` b = Just True == RefOrd.pEqualEff (RefOrd.pCompareDefaultEffort a) a b  

deriving instance
    (Show (Domain f)) 
    => 
    (Show (FnMetaData f))

defaultFnData :: FnData f
defaultFnData =
    FnData
    {
        dataFns = []
    }

defaultFnMetaData ::
    (HasDomainBox f, 
     HasZero (Domain f), HasOne (Domain f)
    )
    =>
    f -> FnMetaData f
defaultFnMetaData sampleF =
    FnMetaData
    {
        dataDestroyed = False,
        dataFnsUpdated = False,
        dataDomName = "x",
        dataDomL = zero sampleDom,
        dataDomR = one sampleDom,
        dataValLO = zero sampleDom,
        dataValHI = one sampleDom,
        dataFnNames = [],
        dataFnGroupNames = [],
        dataFnStyles = [[defaultFnPlotStyle]], -- black
        dataDefaultActiveFns = [],
        dataDefaultEvalPoint = zero sampleDom,
        dataDefaultEvalPointName = "default",
        dataDefaultCanvasParams = defaultCanvasParams sampleDom
    }
    where
    sampleDom = getSampleDomValue sampleF


getDefaultCentre ::
    (ArithInOut.RoundedReal (Domain f))
    =>
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    (FnMetaData f) ->
    (Domain f, Domain f)
getDefaultCentre effReal fnmeta =
    (cX,cY)
    where
    cX =
        (fnL <+> fnR) </>| (2 :: Int) 
    cY = 
        (fnLO <+> fnHI) </>| (2 :: Int)
    (fnLO, fnHI, fnL, fnR) = getFnExtents fnmeta

    (<+>) = ArithInOut.addOutEff effAdd
    (</>|) = ArithInOut.mixedDivOutEff effDivInt

    effDivInt =
        ArithInOut.mxfldEffortDiv sampleDom (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDom effReal
    effAdd =
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    sampleDom = fnLO
    
getFnExtents :: 
    FnMetaData f -> (Domain f, Domain f, Domain f, Domain f)
getFnExtents fnmeta =
    (dataValHI fnmeta, dataValLO fnmeta, 
     dataDomL fnmeta, dataDomR fnmeta)
    
