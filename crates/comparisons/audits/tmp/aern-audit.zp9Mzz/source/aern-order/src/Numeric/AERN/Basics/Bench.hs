{-|
    Module      :  Numeric.AERN.Basics.Bench
    Description :  miscellaneous utilities for benchmarking  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Miscellaneous utilities for benchmarking using Criterion.
-}
module Numeric.AERN.Basics.Bench where

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Effort 

import Control.DeepSeq
import Criterion
import Test.QuickCheck
import Test.QuickCheck.Gen
import qualified Test.QuickCheck.Random as QCR
import qualified System.Random as R

-- The following 
--criterionConfig name samples =
--    defaultConfig 
--    { 
--        cfgSummaryFile = ljust $ name ++ ".csv", 
--        cfgSamples = ljust samples, 
--        cfgResamples = ljust 200, 
----        cfgPlotSameAxis = ljust True,
--        cfgPerformGC = ljust True
----        ,
----        cfgPlot = M.singleton KernelDensity (PDF 1024 780) 
--    }

mkBenchAreasSequences1 ::
    (RefOrd.ArbitraryOrderedTuple t, EffortIndicator ei, NFData t) =>
    (ei -> t -> String) {-^ function constructing benchmark names -} ->
    (ei -> t -> t) {-^ function to benchmark -} ->
    [(String, Area t)] {-^ areas in the input space and their descriptions; empty means whole space only -} ->
    Int {-^ how many benchmarks to generate -} ->
    ei -> t -> [Benchmark]
mkBenchAreasSequences1 mkComment fnEff [] n initEffort sample =
    mkBenchSequence1 mkComment fnEff Nothing n initEffort sample
mkBenchAreasSequences1 mkComment fnEff areas n initEffort sample =
    map areaSequence areas
    where
    areaSequence (descr, area) = 
       bgroup descr $ mkBenchSequence1 mkComment fnEff (Just area) n initEffort sample

mkBenchSequence1 ::
    (RefOrd.ArbitraryOrderedTuple t, EffortIndicator ei, NFData t) =>
    (ei -> t -> String) {-^ function constructing benchmark names -} ->
    (ei -> t -> t) {-^ function to benchmark -} ->
    (Maybe (Area t))  {-^ area in the input space; Nothing means whole space only -} ->
    Int {-^ how many benchmarks to generate -} ->
    ei -> t -> [Benchmark] 
mkBenchSequence1 mkComment fnEff maybeArea n initEffort sample =
    map mkBench $ zip3 [1..n] efforts inputs
    where
    mkBench (n, effort, [input]) =
        bench name (nf (\(e,i) -> fnEff e i) (effort, input))
        where
        name =
--           "" ++ showPad 2 n ++ ":" ++ 
           mkComment effort input ++ ""
    showPad l n = 
        replicate (max 0 (l - (length sn))) '0' ++ sn 
        where sn = show n
    _ = [sample] : inputs
    inputs = 
        map (\(g, size) -> unGen arbitraryInArea g size) $ zip gs sizes
        -- get a sample sequence (always the same!)
    gs = 
        iterate (snd . R.next) $ QCR.mkQCGen 111111321
        -- sequence of random generators
    arbitraryInArea =
       case case maybeArea of
            Nothing -> RefOrd.arbitraryTupleRelatedBy [1] []
            (Just area) -> RefOrd.arbitraryTupleInAreaRelatedBy area [1] []
       of Just gen -> gen
    sizes = 
        concat $ map (replicate 1) $ scanl1 (*) [2,2..] 
        -- ie [2,4,8,16..]
--        -- ie [2,2,2,4,4,4,8,8,8..]
    efforts = 
        concat $ map (replicate 1) $ effortIncrementSequence initEffort

mkBenchSequence2 mkComment fnEff initEffort sample1 sample2 =
    map mkBench $ zip3 [1..20] efforts inputs
    where
    mkBench (n, effort, (input1, input2)) =
        bench name (nf (\(e,i1,i2) -> fnEff e i1 i2) (effort, input1, input2))
        where
        name =
           "(" ++ showPad 2 n ++ ": " ++ mkComment effort input1 input2 ++ ")"
    showPad l n = 
        replicate (max 0 (l - (length sn))) '0' ++ sn 
        where sn = show n
    inputs = zip inputs1 inputs2
    _ = sample1 : inputs1
    inputs1 = 
        map (\(g, size) -> unGen arbitrary g size) $ zip gs1 sizes
    gs1 = 
        iterate (snd . R.next) $ QCR.mkQCGen 1587346765
    _ = sample2 : inputs2
    inputs2 = 
        map (\(g, size) -> unGen arbitrary g size) $ zip gs2 sizes
    gs2 = 
        iterate (snd . R.next) $ QCR.mkQCGen 658246234
    sizes = 
        concat $ map (replicate 3) $ scanl1 (*) [2,2..] 
    efforts = 
        concat $ map (replicate 10) $ effortIncrementSequence initEffort 

    