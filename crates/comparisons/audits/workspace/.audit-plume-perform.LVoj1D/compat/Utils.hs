module Utils (ctoi, itoc, 
Perhaps, pos, unsure, neg, opp,
listhead, pow,
take_integer, drop_integer, toInt, fromInt,
isSpace', isUpper', isLower', isAlpha', isDigit', isAlphanum'
)
	 where


-- Haskell
ctoi :: Char -> Int
ctoi c = fromEnum(c) - fromEnum('0')

itoc :: Int -> Char
itoc i = toEnum (i + fromEnum('0'))


-- Required for Haskell
isSpace', isUpper', isLower', isAlpha', isDigit', isAlphanum' :: Char -> Bool
isSpace' c              =  c == ' ' || c == '\t' || c == '\n' ||
                          c == '\r' || c == '\f' || c == '\v'
isUpper' c              =  c >= 'A'   &&  c <= 'Z'
isLower' c              =  c >= 'a'   &&  c <= 'z'
isAlpha' c              =  isUpper' c  ||  isLower' c
isDigit' c              =  c >= '0'   &&  c <= '9'
isAlphanum' c           =  isAlpha' c  ||  isDigit' c


data Perhaps = Yes  | No | NotSure

pos Yes      = True
pos _        = False

unsure NotSure = True
unsure _     = False

neg No       = True
neg _        = False

opp No       = Yes
opp Yes      = No
opp NotSure    = NotSure



--debug only
listhead 0 _     = []
listhead n (a:x) = (a:listhead (n-1) x)


-- Type : Integer or Int?
pow :: (Integral a) => a -> a -> Double
pow x 0          = 1.0
pow x 1          = fromIntegral x
pow x e | e > 1  = fromIntegral (x^e)
pow x e          = 1.0 / (pow x (-e))



-- take and drop functions with Integer number as no. of elements
-- This is hacked to just use up to the Int maximum.
take_integer :: (Integral a) => a -> [b] -> [b]
take_integer n x = take (toInt n) x

drop_integer :: (Integral a) => a -> [b] -> [b]
drop_integer n x = drop (toInt n) x

-- Audit-only obsolete Prelude conversions; numerical formulas unchanged.
toInt :: Integral a => a -> Int
toInt = fromIntegral

fromInt :: Num a => Int -> a
fromInt = fromIntegral
