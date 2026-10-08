{-# LANGUAGE Arrows #-}

{- | An experiment in using Haskell Arrows, see <http://www.haskell.org/arrows/>,
   to automagically compute Lipschitz constants of maps. Something like this can
   also be used to compute derivatives of functions.

   Here we use floats, but later we intend to replace them with exact reals.
-}

module Data.Reals.Lipschitz where

import Control.Category
import Control.Arrow

-- | Locally Lipshitz maps from @a@ to @b@
data Lipschitz a b = Lipschitz { scaling :: a -> Float, apply :: a -> b }

instance Category Lipschitz where

    id = Lipschitz { scaling = const 1.0, apply = (\x -> x) }

    g . f = Lipschitz { scaling = (\x -> scaling f x * scaling g (apply f x)),
                        apply = (\x -> apply g (apply f x)) }


instance Arrow Lipschitz where

    arr f = Lipschitz { scaling = const infinity, apply = f } 
        where infinity = 1.0 / 0.0

    first f = Lipschitz { scaling = (\(x, _) -> scaling f x),
                          apply = (\(x,y) -> (apply f x, y)) }

    second f = Lipschitz { scaling = (\(_, y) -> scaling f y),
                           apply = (\(x,y) -> (x, apply f y)) }
               
    f *** g = Lipschitz { scaling = (\(x,y) -> max (scaling f x) (scaling g y)),
                          apply = (\(x,y) -> (apply f x, apply g y)) }

    f &&& g = Lipschitz { scaling = (\x -> max (scaling f x) (scaling g x)),
                          apply = (\x -> (apply f x, apply g x)) }



