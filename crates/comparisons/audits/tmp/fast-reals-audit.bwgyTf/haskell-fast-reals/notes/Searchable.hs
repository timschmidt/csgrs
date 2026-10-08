-- searchable spaces, based on code by Martin Escardo

module Searchable where

data Searchable a = Finder ((a -> Bool) -> a)
find :: Searchable a -> (a -> Bool) -> a
find (Finder epsilon) = epsilon

-- auxiliary function search
search :: Searchable a -> (a -> Bool) -> Maybe a
search s p =
    let x = find s p
    in if p x then Just x else Nothing

-- quantifiers

exists s p = p (find s p)

forall s p = not (exists s (not . p))

-- some searchable spaces

-- singleton
singleton x = Finder (const x)

-- doubleton
doubleton x y = Finder (\p -> if p x then x else y)

-- finite non-empty sets
finiteSet :: [a] -> Searchable a

finiteSet lst = Finder (\p ->
    let loop []     = undefined
        loop [x]    = x
        loop (x:xs) = if p x then x else loop xs
    in loop lst)

-- the sum of two searchable sets a and b is searchable
sum s t = Finder (\p -> let x = Left (find s (p . Left))
                            y = Right (find t (p . Right))
                        in if p x then x else y)

-- a union of a searchable family of searchable spaces ss
bigUnion :: Searchable (Searchable a) -> Searchable a
bigUnion ss = Finder (\p -> find (find ss (\s -> exists s p)) p)

-- a union of two sets is a special case
union s t = bigUnion (doubleton s t)

-- the image of a searchable set s under a map f
image f s = Finder (\p -> f (find s (p . f)))
                              
-- monad structure for searchable spaces
instance Monad Searchable where
    return   = singleton
    s >>= f = bigUnion (image f s)

-- product of a and b
a `times` b = do x <- a
                 y <- b
                 return (x, y)

-- a product of a list of spaces, where the list may be infinite
prod [] = return []
prod (a:as) = do x <- a
                 xs <- prod as
                 return (x:xs)

-- the Cantor space
two = doubleton False True
cantor = prod (repeat two)

-- we may test equality of functions
equal a f g = forall a (\x -> f x == g x)
