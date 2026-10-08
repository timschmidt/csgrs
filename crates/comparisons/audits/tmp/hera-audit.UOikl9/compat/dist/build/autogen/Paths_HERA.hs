{-# LANGUAGE CPP #-}
{-# LANGUAGE NoRebindableSyntax #-}
#if __GLASGOW_HASKELL__ >= 810
{-# OPTIONS_GHC -Wno-prepositive-qualified-module #-}
#endif
{-# OPTIONS_GHC -fno-warn-missing-import-lists #-}
{-# OPTIONS_GHC -w #-}
module Paths_HERA (
    version,
    getBinDir, getLibDir, getDynLibDir, getDataDir, getLibexecDir,
    getDataFileName, getSysconfDir
  ) where


import qualified Control.Exception as Exception
import qualified Data.List as List
import Data.Version (Version(..))
import System.Environment (getEnv)
import Prelude


#if defined(VERSION_base)

#if MIN_VERSION_base(4,0,0)
catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
#else
catchIO :: IO a -> (Exception.Exception -> IO a) -> IO a
#endif

#else
catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
#endif
catchIO = Exception.catch

version :: Version
version = Version [0,2] []

getDataFileName :: FilePath -> IO FilePath
getDataFileName name = do
  dir <- getDataDir
  return (dir `joinFileName` name)

getBinDir, getLibDir, getDynLibDir, getDataDir, getLibexecDir, getSysconfDir :: IO FilePath




bindir, libdir, dynlibdir, datadir, libexecdir, sysconfdir :: FilePath
bindir     = "/tmp/hera-audit.UOikl9/compat-install/bin"
libdir     = "/tmp/hera-audit.UOikl9/compat-install/lib/x86_64-linux-ghc-9.6.7/HERA-0.2-DJZl1M3lt5T5Ba1eLHDkbL"
dynlibdir  = "/tmp/hera-audit.UOikl9/compat-install/lib/x86_64-linux-ghc-9.6.7"
datadir    = "/tmp/hera-audit.UOikl9/compat-install/share/x86_64-linux-ghc-9.6.7/HERA-0.2"
libexecdir = "/tmp/hera-audit.UOikl9/compat-install/libexec/x86_64-linux-ghc-9.6.7/HERA-0.2"
sysconfdir = "/tmp/hera-audit.UOikl9/compat-install/etc"

getBinDir     = catchIO (getEnv "HERA_bindir")     (\_ -> return bindir)
getLibDir     = catchIO (getEnv "HERA_libdir")     (\_ -> return libdir)
getDynLibDir  = catchIO (getEnv "HERA_dynlibdir")  (\_ -> return dynlibdir)
getDataDir    = catchIO (getEnv "HERA_datadir")    (\_ -> return datadir)
getLibexecDir = catchIO (getEnv "HERA_libexecdir") (\_ -> return libexecdir)
getSysconfDir = catchIO (getEnv "HERA_sysconfdir") (\_ -> return sysconfdir)



joinFileName :: String -> String -> FilePath
joinFileName ""  fname = fname
joinFileName "." fname = fname
joinFileName dir ""    = dir
joinFileName dir fname
  | isPathSeparator (List.last dir) = dir ++ fname
  | otherwise                       = dir ++ pathSeparator : fname

pathSeparator :: Char
pathSeparator = '/'

isPathSeparator :: Char -> Bool
isPathSeparator c = c == '/'
