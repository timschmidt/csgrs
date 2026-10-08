{-# LANGUAGE NoRebindableSyntax #-}
{-# OPTIONS_GHC -fno-warn-missing-import-lists #-}
{-# OPTIONS_GHC -w #-}
module PackageInfo_FewDigits (
    name,
    version,
    synopsis,
    copyright,
    homepage,
  ) where

import Data.Version (Version(..))
import Prelude

name :: String
name = "FewDigits"
version :: Version
version = Version [0,5,0] []

synopsis :: String
synopsis = "Exact real arithmetic library"
copyright :: String
copyright = ""
homepage :: String
homepage = "http://r6.ca/FewDigits/"
