module Plot (Plottable(..), FI(..), plotCmd, plotData, clear) where

-- This whole module is a hack introduced in order to visualize solutions to initial value problems in 
-- ODE.hs. Warning!
-- import Control.Concurrent
import System.Process
import System.IO
import System.IO.Unsafe

-- Handle for writing to gnuplot process ---------------------------------------
-- Unsafe!! Starts that process as side effect on first use -------------------
handle :: Handle
handle = unsafePerformIO $ 
           do (inp,_,_,_) <-
                 runInteractiveProcess "/usr/local/bin/gnuplot" [] Nothing Nothing
              hSetBinaryMode inp False
              hSetBuffering inp LineBuffering  
              hPutStrLn inp "set multiplot\nplot 1\nclear"
              return inp

-- Send gnuplot command to gnuplot process
plotCmd :: String -> IO ()
plotCmd str = do hPutStrLn handle str
                 return ()

-- Clear plot window
clear :: IO ()
clear = plotCmd "clear"


plotData :: [[String]] -> String -> String -> IO ()
plotData   [] _ _ = clear
plotData   fss a b = plotCmd ("clear\nplot [" ++ a ++ ":" ++ b 
                     ++ "] '-' w l t \"0\"" 
                     ++ concatMap h [1..length fss-1] ++ "\n" ++ concatMap p fss)
   where h n = "  ,'' w l t " ++ show (show n)
         p fs = unlines fs ++ "e\n"


class Plottable a where
   plot :: [a] -> IO ()

data FI a = FI (a -> a) a a

instance  (Fractional a, Show a) => Plottable (FI a) where 
  plot fs@(f : _) = plotData (map p fs) (show a) (show b)
    where FI _ a b = f
          p (FI f a b) = map mkLine [0..500]
                  where mkLine n = show x ++ " " ++ show (f x)
                          where x = fromIntegral n*((b-a)/500) + a

