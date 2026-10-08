module LookInt (Lookint(..), lig0, lig1, lig2, ligu,
	       liglist, lignewl,
	       ligilist', ligilist,
	       ligGlist', ligGlist,
	       extract_Lookahead,
	       toIntList, e, e2, g) where

data Lookint = Lookint (Int, Int) deriving Show

li_add :: Lookint -> Lookint -> Lookint
li_add (Lookint (x,a)) (Lookint (y,b)) = Lookint (x+y, max a b)

li_sub :: Lookint -> Lookint -> Lookint
li_sub (Lookint (x,a)) (Lookint (y,b)) = Lookint (x-y, max a b)

li_mul :: Lookint -> Lookint -> Lookint
li_mul (Lookint (x,a)) (Lookint (y,b)) = Lookint (x*y, max a b)

li_abs :: Lookint -> Lookint
li_abs (Lookint (x,a)) = Lookint (abs x, a)

li_negate :: Lookint -> Lookint
li_negate (Lookint (x,a)) = Lookint (negate x, a)

li_signum :: Lookint -> Lookint
li_signum (Lookint (x,a)) = Lookint (signum x, a)

li_fromInteger :: Integer -> Lookint
li_fromInteger x = Lookint (fromInteger x, 0)



li_quotRem :: Lookint -> Lookint -> (Lookint, Lookint)
li_quotRem (Lookint (x,a)) (Lookint (y,b)) = (Lookint (quot x y, max a b),
					      Lookint (rem x y, max a b))


li_divMod :: Lookint -> Lookint -> (Lookint, Lookint)
li_divMod (Lookint (x,a)) (Lookint (y,b)) = (Lookint (div x y, max a b),
					     Lookint (mod x y, max a b))


li_toInteger :: Lookint -> Integer
li_toInteger (Lookint (x,_)) = toInteger x



li_eq :: Lookint -> Lookint -> Bool
li_eq (Lookint (x,a)) (Lookint (y,b)) = x==y

li_gte :: Lookint -> Lookint -> Bool
li_gte (Lookint (x,a)) (Lookint (y,b)) = x>=y

li_gt :: Lookint -> Lookint -> Bool
li_gt (Lookint (x,a)) (Lookint (y,b)) = x>y


li_lte :: Lookint -> Lookint -> Bool
li_lte (Lookint (x,a)) (Lookint (y,b)) = x<=y

li_lt :: Lookint -> Lookint -> Bool
li_lt (Lookint (x,a)) (Lookint (y,b)) = x<y


li_compare ::  Lookint -> Lookint -> Ordering
li_compare (Lookint (x,a)) (Lookint (y,b)) = 
	   case (signum (x-y)) of
		(-1)   -> LT
		(0)   -> EQ
		(1)   -> GT

li_max :: Lookint -> Lookint -> Lookint
li_max (Lookint (x,a)) (Lookint (y,b)) = Lookint (max x y, max a b)

li_min :: Lookint -> Lookint -> Lookint
li_min (Lookint (x,a)) (Lookint (y,b)) = Lookint (min x y, max a b)


li_toEnum :: Int -> Lookint
li_toEnum x = Lookint (x,0)

li_fromEnum :: Lookint -> Int
li_fromEnum  (Lookint (x,_)) = x

li_enumFrom :: Lookint -> [Lookint]
li_enumFrom (Lookint (x,a)) = (Lookint (x,a):li_enumFrom (Lookint (x+1,a+1)))

li_enumFromThen :: Lookint -> Lookint -> [Lookint]
li_enumFromThen (Lookint (x,a)) (Lookint (y,_)) = (Lookint (x,a):
	   li_enumFromThen (Lookint (y,a+1)) (Lookint (2*y-x, 0)))


li_minBound :: Lookint 
li_minBound = (Lookint (minBound :: Int, 0))

li_maxBound :: Lookint 
li_maxBound = (Lookint (maxBound :: Int, 0))

li_toRational :: Lookint -> Rational
li_toRational (Lookint (x,_)) = toRational x



instance Eq Lookint where
   (==)   = li_eq

instance Ord Lookint where
   (>=)	   = li_gte 
   (>)	   = li_gt 
   (<=)	   = li_lte 
   (<)	   = li_lt 
   compare = li_compare

instance Enum Lookint where
   toEnum       = li_toEnum
   fromEnum     = li_fromEnum
   enumFrom     = li_enumFrom
   enumFromThen = li_enumFromThen


instance Num Lookint where
   (+)	  = li_add 
   (-)	  = li_sub
   (*)	  = li_mul
   abs	  = li_abs
   signum = li_signum
   negate = li_negate
   fromInteger = li_fromInteger


instance Real Lookint where
   toRational = li_toRational

instance Integral Lookint where
   quotRem	 = li_quotRem
   divMod	 = li_divMod
   toInteger	 = li_toInteger



-- Lookint Generators

lig0 :: Int -> Lookint
lig0 = fromIntegral

lig1 :: Int -> Lookint -> Lookint
lig1 i (Lookint (_,a)) = Lookint (i,a)

lig2 :: Int -> Lookint -> Lookint -> Lookint
lig2 i (Lookint (_,a)) (Lookint (_,b)) = Lookint (i,max a b)


-- Return 1st digit with lookahead of max 1st, 2nd
ligu :: Lookint -> Lookint -> Lookint
ligu (Lookint (x,a)) (Lookint (_,b)) = Lookint (x,max a b)


-- Generate a list of x with lookahead of specified digit.
liglist :: Int -> Lookint -> [Lookint]
liglist x l@(Lookint (_,a)) = (Lookint (x,a): liglist x l)


-- Generate a list with lookahead growing from specified digit.
ligGlist' :: Lookint -> [Lookint] -> [Lookint]
ligGlist' (Lookint (_,a)) ((Lookint (x,_)):r) =
	  ((Lookint (x,a)): ligGlist' (Lookint (0,a+1)) r)

-- Generate a list with lookahead growing from one.
ligGlist :: [Lookint] -> [Lookint]
ligGlist x = ligGlist' (g 0 1) x


-- Take digit and stream, and return stream with lookahead max of dig/stream
lignewl :: Lookint -> [Lookint] -> [Lookint]
lignewl (Lookint (_,a)) (Lookint (y,b):rest) 
	| b >= a    = (Lookint (y,b):rest)
	| otherwise = (Lookint (y,a):lignewl (Lookint (0,a)) rest)



-- ligilist : Convert int list -> lookint list
ligilist' :: [Int] -> Lookint -> [Lookint]
ligilist' (x:y) (Lookint (_,a)) = (Lookint (x,a):ligilist' y (Lookint (0,a+1)))

ligilist :: [Int] -> [Lookint]
ligilist x = ligilist' x (g 0 1)



extract_Lookahead :: [Lookint] -> String
extract_Lookahead (Lookint (_,a):x) = show a ++", "++(extract_Lookahead x)


-- Extract digit lookahead
e :: Lookint -> Int
e (Lookint (_,a)) = a


-- Extract max lookahead of 2 digits
e2 :: Lookint -> Lookint -> Int
e2 (Lookint (_,a)) (Lookint (_,b))= max a b

-- Generate digit with lookahead
g :: Int -> Int -> Lookint 
g x a = (Lookint (x,a))


-- toIntList : Convert [Lookint] -> [Int]
toIntList :: [Lookint] -> [Int]
toIntList [] = []
toIntList (Lookint (a,_):x) = (a:toIntList x)


