module Parsing where

{-
One of the many incarnations of the standard
combinator parser library (last modified: Friday, March 07, 1997)

Erik Meijer (erik@cs.ruu.nl)
-}

import Char
infixr 5 +++

--- The parser monad
---------------------------------------------------------

newtype Parser a   = Parser (String -> [(a,String)])

instance Functor Parser where
   -- map         :: (a -> b) -> (Parser a -> Parser b)
   map f (Parser p)     = Parser (\inp -> [(f v, out) | (v,out) <- p inp])

instance Monad Parser where
   -- return      :: a -> Parser a
   return v        = Parser (\inp -> [(v,inp)])

   -- >>=         :: Parser a -> (a -> Parser b) -> Parser b
   (Parser p) >>= f     = Parser (\inp -> concat [papply (f v) out
                                       | (v,out) <- p inp])

instance MonadZero Parser where
   -- zero        :: Parser a
   zero            = Parser (\inp -> [])

instance MonadPlus Parser where
   -- (++)        :: Parser a -> Parser a -> Parser a
   (Parser p) ++ (Parser q)  = Parser (\inp -> (p inp ++ q inp))

--- Other primitive parser combinators
---------------------------------------

item              :: Parser Char
item               = Parser (\inp -> case inp of
                                   []     -> []
                                   (x:xs) -> [(x,xs)])

force             :: Parser a -> Parser a
force (Parser p)        = Parser (\inp -> let x = p inp in
                                (fst (head x), snd (head x)) : tail
                                x)

first             :: Parser a -> Parser a
first (Parser p)        = Parser (\inp -> case p inp of
                                   []     -> []
                                   (x:xs) -> [x])

papply            :: Parser a -> String -> [(a,String)]
papply (Parser p) inp   = p inp

--- Derived combinators
------------------------------------------------------

(+++)             :: Parser a -> Parser a -> Parser a
p +++ q            = first (p ++ q)


sat               :: (Char -> Bool) -> Parser Char
sat p              = do {x <- item; if p x then return x else zero}


many              :: Parser a -> Parser [a]
many p             = force (many1 p +++ return [])

many1             :: Parser a -> Parser [a]
many1 p            = do {x <- p; xs <- many p; return (x:xs)}

sepby             :: Parser a -> Parser b -> Parser [a]
p `sepby` sep      = (p `sepby1` sep) +++ return []

sepby1            :: Parser a -> Parser b -> Parser [a]
p `sepby1` sep     = do {x <- p
                        ; xs <- many (do {sep; p})
                        ; return(x:xs)
                        }

chainl            :: Parser a -> Parser (a -> a -> a)
                     -> a -> Parser a
chainl p op v      = (p `chainl1` op) +++ return v

chainl1           :: Parser a -> Parser (a -> a -> a) -> Parser a
p `chainl1` op     = do {x <- p; rest x}
                     where
                        rest x = do {f <- op; y <- p; rest (f x y)}
                                 +++ return x

chainr :: Parser a -> Parser (a -> a -> a) -> a -> Parser a
chainr p op v      = (p `chainr1` op) +++ return v

chainr1           :: Parser a -> Parser (a -> a -> a) -> Parser a
p `chainr1` op     = do {x <- p; rest x}
                     where
                        rest x = do {f <- op; y <- p `chainr1` op;
                        return (f x y)}
                                 +++ return x

ops               :: [(Parser a, b)] -> Parser b
ops xs             = foldr1 (+++) [do {p; return op} | (p,op) <- xs]

bracket           :: Parser a -> Parser b -> Parser c -> Parser b
bracket open p close = do {open; x <- p; close; return x}

--- Useful parsers
-----------------------------------------------------------

char              :: Char -> Parser Char
char x             = sat (\y -> x == y)

digit             :: Parser Char
digit              = sat isDigit

lower             :: Parser Char
lower              = sat isLower

upper             :: Parser Char
upper              = sat isUpper

letter            :: Parser Char
letter             = sat isAlpha

alphanum          :: Parser Char
alphanum           = sat isAlphanum

string            :: String -> Parser String
string ""          = return ""
string (x:xs)      = do {char x; string xs; return (x:xs)}

ident             :: Parser String
ident              = do {x <- lower
                        ; xs <- many alphanum
                        ; return(x:xs)
                        }

nat               :: Parser Int
nat                = do {x <- digit
                        ; return (ord x - ord '0')
                        } `chainl1` return op
                     where
                        m `op` n = 10*m + n

int               :: Parser Int
int                = do {char '-'; n <- nat; return (-n)} +++ nat

--- Lexical combinators
------------------------------------------------------

spaces            :: Parser ()
spaces             = do {many1 (sat isSpace); return ()}

junk              :: Parser ()
junk               = do {many (spaces); return ()}

parse             :: Parser a -> Parser a
parse p            = do {junk; p}

token             :: Parser a -> Parser a
token p            = do {v <- p; junk; return v}

--- Token parsers
------------------------------------------------------------

natural           :: Parser Int
natural            = token nat

integer           :: Parser Int
integer            = token int

symbol            :: String -> Parser String
symbol xs          = token (string xs)

identifier        :: [String] -> Parser String
identifier ks      = token (do {x <- ident
                               ; if not (elem x ks) then return x else zero})

---------------------------------------------------------------------
---------

whitespace = sat isSpace
alpha      = sat isAlpha
punctuation = sat (`elem` "?!.,;:-_()`\'\"")
hexdigit   = sat isHex
printable  = sat isPrint
isHex c = isDigit c || (toLower c >= 'a'   &&  toLower c <= 'f')
