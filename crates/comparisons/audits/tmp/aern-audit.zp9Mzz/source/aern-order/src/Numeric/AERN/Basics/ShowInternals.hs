{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.Basics.Exception
    Description :  formatting with optional display of internal components   
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Formatting with optional display of internal components.
-}

module Numeric.AERN.Basics.ShowInternals where

class ShowInternals t where
    type ShowInternalsIndicator t
    defaultShowIndicator :: t -> ShowInternalsIndicator t
    showInternals :: (ShowInternalsIndicator t) -> t -> String

showUsingShowInternals :: (ShowInternals t) => t -> String
showUsingShowInternals a =
    showInternals (defaultShowIndicator a) a
    

