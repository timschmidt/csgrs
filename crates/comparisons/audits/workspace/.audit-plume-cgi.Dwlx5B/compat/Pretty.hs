module Pretty
( (<>)
, (<+>)
, ($$)
, sep
, nest
, text
, pretty
, pack
, row
, column
) where

{-
A variant of John Hughes's Pretty Printer Combinators.
Look in"The Design of a Pretty-printing Library"
in Advanced Functional Programming
Johan Jeuring and Erik Meijer (eds)
LNCS 925 for explanation.

(last modified: Friday, March 07, 1997)

Erik Meijer (erik@cs.ruu.nl)
-}


infixl 6 <>
infixl 6 <+>
infixl 5 $$

text   :: String -> Doc
(<>)   :: Doc -> Doc -> Doc
(<+>)  :: Doc -> Doc -> Doc
($$)   :: Doc -> Doc -> Doc
sep    :: [Doc] -> Doc
nest   :: Int -> Doc -> Doc
pretty :: Int -> Int -> Doc -> ShowS
pack   :: [Doc] -> Doc

instance Show Doc where
   showsPrec w = pretty 80 w
   -- standard textwidth is 80; ribbon supplied by user

{-

Quick reference for the Pretty-print Combinators

  |---|    |----|   |-------|
  |koe| <> |beer| = |koebeer|
  |---|    |----|   |-------|

  |---|     |----|   |--------|
  |koe| <+> |beer| = |koe beer|
  |---|     |----|   |--------|

  |---|    |----|   |----|
  |koe| $$ |beer| = |koe |
  |---|    |----|   |beer|
                    |----|

  |---|           |----|   |-------|
  |koe| <> nest 2 |beer| = |koebeer|
  |---|           |----|   |-------|

  |---|           |----|   |------|
  |koe| $$ nest 2 |beer| = |koe   |
  |---|           |----|   |  beer|
                           |------|

  sep [d1,...,dn]
  =  d1 <+> ... <+> dn
  |  d1 $$  ... $$  dn

  pack [d1,...,dn]
  = d1 <+> ... <+> di
    $$ di+1
    ...
    $$ dn

  pretty width ribbon :: ShowS

  row [d1,...,dn] = d1 <+> ... <+> dn
  column [d1,...,dn] = d1 $$ ... $$ dn

-}

pack []
 = text ""
pack ds
 = foldl1 (\d1 d2 -> sep [d1,d2]) ds

row []
 = text ""
row as
 = foldr1 (<+>) as

column []
 = text ""
column as
 = foldr1 ($$) as

------------------------------------------------------------------
--     YOU PROBABLY DON'T WANT TO LOOK AT THE CODE BELOW        --
------------------------------------------------------------------

data Doc
 = Nil
 | NilAbove Doc
 | TextBeside Str Doc
 | Nest Int Doc
 | Union Doc Doc
 | Empty

type Str = (Int,ShowS) {-in
   len, cat, toString, fromString, toShowS-}

len :: Str -> Int
len (i,_) = i

cat :: Str -> Str -> Str
(i,s) `cat` (j,t) = (i+j,s . t)

fromString :: String -> Str
fromString s = (length s,showString s)

toString :: Str -> String
toString (_,s) = s ""

toShowS :: Str -> ShowS
toShowS (_,s) = s

text s = fromString s `TextBeside` Nil

nest = Nest

d1 $$ d2 = aboveNest d1 0 d2

aboveNest :: Doc -> Int -> Doc -> Doc
aboveNest d
 = strict (\k ->
    case d of
      Nil              -> NilAbove . nest k
      NilAbove d       -> NilAbove . aboveNest d k
      s `TextBeside` d -> (s `TextBeside`) . aboveNest (Nil <> d)
                                    (k - len s)
      Nest k' d        -> Nest k' . aboveNest d (k-k')
      d1 `Union` d2    -> \d -> (aboveNest d1 k d)
                                 `Union` (aboveNest d2 k d)
      Empty            -> \d -> Empty
      )

d1 <+> d2 = d1 <> text " " <> d2

d1 <> d2
 = case d1 of
      Nil                 -> case d2 of
                              (Nest k d) -> Nil <> d
                              _ -> d2
      NilAbove d1         -> NilAbove (d1 <> d2)
      (s `TextBeside` d1) -> s `TextBeside` (d1 <> d2)
      Nest k d1           -> Nest k (d1 <> d2)
      Empty               -> Empty
      (d0 `Union` d1)     -> (d0 <> d2) `Union` (d1 <> d2)


sep ds = sepBy ds " "

sepBy ds s
 = case ds of
     [d]  -> d
     d:ds -> sep' d 0 ds
   where
      sep' d
       = strict (\k ->
          case d of
             Nil              -> \ds -> (fit (foldl (<+>) Nil ds))
                                         `Union` (vertical Nil k ds)
             NilAbove d       -> vertical (NilAbove d) k
             s `TextBeside` d -> (s `TextBeside`) . sep' (Nil <> d)
                                                      (k - len s)
             Nest n d         -> Nest n.sep' d (k-n)
             d1 `Union` d2    -> \ds -> (sep' d1 k ds) `Union`
                                           (vertical d2 k ds)
             Empty            -> \ds -> Empty
             ) where
                  d1 <+> d2       = d1 <> text s <> d2
                  vertical d k ds = d $$ nest k (foldr1 ($$) ds)
                  fit d
                   = case d of
                        Nil              -> Nil
                        NilAbove d       -> Empty
                        s `TextBeside` d -> s `TextBeside` (fit d)
                        Nest n d         -> Nest n (fit d)
                        d1 `Union` d2    -> fit d1
                        Empty            -> Empty

best w r
 = \d -> case d of
            Nil              -> Nil
            NilAbove d       -> NilAbove (best w r d)
            s `TextBeside` d -> s `TextBeside` (best' w r s d)
            Nest k d         -> Nest k (best (w-k) r d)
            d1 `Union` d2    -> nicest w r (best w r d1)
                                           (best w r d2)
            Empty            -> Empty

best' w r s
 = \d -> case d of
            Nil              -> Nil
            NilAbove d       -> NilAbove (best (w - len s) r d)
            t `TextBeside` d -> t `TextBeside`
                                  (best' w r (s `cat` t) d)
            Nest k d         -> best' w r s d
            d1 `Union` d2    -> nicest' w r s (best' w r s d1)
                                    (best' w r s d2)
            Empty            -> Empty

nicest w r          = nicest' w r (fromString  "")
nicest' w r s d1 d2 = if fits (w `min` r) (len s) d1 then d1 else d2

fits n k d
 = if n < k
   then False
   else case d of
           Nil              -> True
           NilAbove _       -> True
           s `TextBeside` d -> fits n (k + len s) d
           Empty            -> False

layout_ :: Int -> Doc -> ShowS
layout_ k (Nest k' d) = layout_ (k+k') d
layout_ k d           = showString (space k) . layout' k d

space n = take n (repeat ' ')

layout' :: Int -> Doc -> ShowS
layout' k Nil                = showString "\n"
layout' k (NilAbove d)       = showString "\n" . layout_ k d
layout' k (s `TextBeside` d) = toShowS s . layout' (k + len s) d

--pretty :: Int -> Int -> Doc -> ShowS
pretty w r = layout_ 0 . best w r
