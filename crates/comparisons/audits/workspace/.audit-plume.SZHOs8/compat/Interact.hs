-----------------------------------------------------------------------------
-- Library for simple interactive programs:
--
-- Suitable for use with Hugs 1.4.
-----------------------------------------------------------------------------

module Interact(
	Interact(..),
	end,
	readChar, peekChar, unreadChar, pressAnyKey,
	writeChar, writeStr,
	readLine,
	ringBell
	) where

--- Interactive program combining forms:

type Interact = String -> String

end                      :: Interact
readChar, peekChar       :: Interact -> (Char -> Interact) -> Interact
pressAnyKey              :: Interact -> Interact
unreadChar               :: Char -> Interact -> Interact
writeChar                :: Char -> Interact -> Interact
writeStr                 :: String -> Interact -> Interact
ringBell                 :: Interact -> Interact
readLine                 :: String -> (String -> Interact) -> Interact

end cs                    = ""

readChar eof use []       = eof []
readChar eof use (c:cs)   = use c cs

peekChar eof use []       = eof []     -- like readChar, but character is
peekChar eof use cs@(c:_) = use c cs   -- not removed from input stream

pressAnyKey prog          = readChar prog (\c -> prog)

unreadChar c prog cs      = prog (c:cs)

writeChar c prog cs       = c : prog cs

writeStr s prog cs        = s ++ prog cs

ringBell                  = writeChar '\BEL'

readLine prompt g is  = prompt ++ lineOut 0 line ++ "\n"
                               ++ g (noBackSpaces line) input'
 where line     = before '\n' is
       input'   = after  '\n' is
       after x  = tail . dropWhile (x/=)
       before x = takeWhile (x/=)

       rubout  :: Char -> Bool
       rubout c = (c=='\DEL' || c=='\BS')

       lineOut                      :: Int -> String -> String
       lineOut n ""                  = ""
       lineOut n (c:cs)
                 | n>0  && rubout c  = "\BS \BS" ++ lineOut (n-1) cs
                 | n==0 && rubout c  = lineOut 0 cs
                 | otherwise         = c:lineOut (n+1) cs

       noBackSpaces :: String -> String
       noBackSpaces  = reverse . delete 0 . reverse
                       where delete n ""          = ""
                             delete n (c:cs)
                                      | rubout c  = delete (n+1) cs
                                      | n>0       = delete (n-1) cs
                                      | otherwise = c:delete 0 cs

-----------------------------------------------------------------------------
