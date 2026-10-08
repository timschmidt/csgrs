module Main where

import qualified Lexer as L
import qualified ILexer as IL
import qualified Parser as P
import qualified IParser as IP
import qualified ParseTree as T
import qualified IParseTree as IT
import Alex (Posn(..))
import Control.Monad (forM_, unless)
import Data.List (isPrefixOf)
import System.Environment (getArgs)

-- Independent small lexer specification. No donor transition tables are used.
data Lexeme = K String | N String | I String | V String | E Int Int Int
  deriving (Eq, Show)

keywords = ["sin","cos","arctan","exp","ln","sqrt","max","min","pi","Digits","exit"]
extraKeywords = ["int","fmax","fmin"]
white c = c `elem` " \t\n\r\f\v"
digit c = c >= '0' && c <= '9'
alpha c = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
ident c = alpha c || digit c || c `elem` "_'"

reference integrated = go (0,1,1)
  where
    go _ [] = []
    go pos s@(c:rest)
      | white c = go (advance pos c) rest
      | digit c = let (whole,tail1) = span digit s
                        -- Decimal requires a digit on both sides of the dot.
                        in case tail1 of
                          '.':d:ds | digit d -> let (fraction,tail2) = span digit (d:ds)
                                                    token = whole ++ "." ++ fraction
                                                in N token : go (walk pos token) tail2
                          _ -> I whole : go (walk pos whole) tail1
      | alpha c = let (name,tail1) = span ident s
                      kind = if name `elem` (keywords ++ if integrated then extraKeywords else []) then K else V
                  in kind name : go (walk pos name) tail1
      | ":=" `isPrefixOf` s = K ":=" : go (walk pos ":=") (drop 2 s)
      | c `elem` ("()+-*/," ++ if integrated then "" else "^") = K [c] : go (advance pos c) rest
      | otherwise = let (a,l,col) = pos in [E a l col]
    walk = foldl advance
    advance (a,l,col) '\n' = (a+1,l+1,1)
    advance (a,l,col) '\t' = (a+1,l,1+8*((col+7) `div` 8))
    advance (a,l,col) _ = (a+1,l,col+1)

ordinaryToken token = case token of
  T.TokenNum s -> N s
  T.TokenIntnum s -> I s
  T.TokenVar s -> V s
  T.Err (Pn a l c) -> E a l c
  _ -> K (case token of
    T.TokenDigits -> "Digits"; T.TokenExit -> "exit"; T.TokenSin -> "sin"
    T.TokenCos -> "cos"; T.TokenArctan -> "arctan"; T.TokenExp -> "exp"
    T.TokenLn -> "ln"; T.TokenSqrt -> "sqrt"; T.TokenMax -> "max"
    T.TokenMin -> "min"; T.TokenPi -> "pi"; T.TokenAssign -> ":="
    T.TokenPlus -> "+"; T.TokenMinus -> "-"; T.TokenTimes -> "*"
    T.TokenDiv -> "/"; T.TokenOB -> "("; T.TokenCB -> ")"
    T.TokenPow -> "^"; T.TokenComma -> ",")

integratedToken token = case token of
  IT.TokenNum s -> N s
  IT.TokenIntnum s -> I s
  IT.TokenVar s -> V s
  IT.Err (Pn a l c) -> E a l c
  _ -> K (case token of
    IT.TokenDigits -> "Digits"; IT.TokenExit -> "exit"; IT.TokenSin -> "sin"
    IT.TokenCos -> "cos"; IT.TokenArctan -> "arctan"; IT.TokenExp -> "exp"
    IT.TokenLn -> "ln"; IT.TokenSqrt -> "sqrt"; IT.TokenMax -> "max"
    IT.TokenMin -> "min"; IT.TokenPi -> "pi"; IT.TokenAssign -> ":="
    IT.TokenPlus -> "+"; IT.TokenMinus -> "-"; IT.TokenTimes -> "*"
    IT.TokenDiv -> "/"; IT.TokenOB -> "("; IT.TokenCB -> ")"
    IT.TokenComma -> ","; IT.TokenIntegr -> "int"; IT.TokenFMax -> "fmax"
    IT.TokenFMin -> "fmin")

check label expected actual = unless (expected == actual) $
  error (label ++ " expected=" ++ show expected ++ " actual=" ++ show actual)

lexicalChecks = do
  let chars = ['\0'..'\127']
      tiny = "" : map (:[]) chars ++ [[a,b] | a <- chars, b <- chars]
      words = [prefix ++ word ++ suffix | word <- keywords ++ extraKeywords,
               prefix <- [""," \t","x","("], suffix <- ["","1","_","'","x"," ","("]]
      nums = [a ++ b ++ c | a <- ["0","1","1234567890"], b <- ["",".",".0",".001"],
                          c <- [""," ","x",".2","\n"]]
      positions = ["a\tb\n\r\f\v@", "\n\t\t0.4@", "pi\r\n@", "0.12+4\n  ?"]
      inputs = tiny ++ words ++ nums ++ positions
  forM_ inputs $ \s -> do
    check ("ordinary lexer " ++ show s) (reference False s) (map ordinaryToken (L.lexer s))
    check ("integrated lexer " ++ show s) (reference True s) (map integratedToken (IL.lexer s))
  putStrLn ("lexer\t" ++ show (2*length inputs) ++ "\tpass")

-- AST expectations do not use the generated grammar actions.
n s = T.Factor (T.Intnum (T.An_Intnum s))
e s = T.Term (T.NFactor (n s))
asExp x = show (T.Exp1 x)
factorExp f = T.Term (T.NFactor (T.Factor f))

parserChecks = do
  let common =
        [("",show T.NoExp),(" \t",show T.NoExp),("exit",show T.Exit),
         ("Digits := 12",show (T.Digits "12")),("x := 3",show (T.Assign "x" (e "3"))),
         ("1",asExp (e "1")),("1+2*3",asExp (T.Plus (e "1") (T.Times (T.NFactor (n "2")) (n "3")))),
         ("1-2-3",asExp (T.Minus (T.Minus (e "1") (T.NFactor (n "2"))) (T.NFactor (n "3")))),
         ("8/2/2",asExp (T.Term (T.Div (T.Div (T.NFactor (n "8")) (n "2")) (n "2")))),
         ("-1",asExp (T.Term (T.NFactor (T.Neg (T.Intnum (T.An_Intnum "1")))))),
         ("1.25",asExp (factorExp (T.Num "1.25"))),
         ("x_1'",asExp (factorExp (T.Var (T.A_Var "x_1'"))))]
        ++ [(name ++ "(1)",asExp (T.Term (T.NFactor (T.Factor (ctor (e "1")))))) |
            (name,ctor) <- [("sin",T.Sin),("cos",T.Cos),("arctan",T.Arctan),("exp",T.Exponential),("ln",T.Ln),("sqrt",T.Sqrt)]]
        ++ [(name ++ "(1,2)",asExp (T.Term (T.NFactor (T.Factor (ctor (e "1",e "2")))))) |
            (name,ctor) <- [("min",T.Min),("max",T.Max)]]
      bad = ["1.",".1","+1","--1","1 2","1+","1@","sin","sin()","sin(1,2)","min(1)",
             "max(1,2,3)","(1","1)","exit 1","Digits:=1.0","x:=","1:=2","0x10","1e3"]
  forM_ common $ \(s,wanted) -> do
    check ("ordinary parse " ++ s) wanted (show (P.parse s))
    check ("integrated parse " ++ s) wanted (show (IP.parse s))
  forM_ bad $ \s -> do
    check ("ordinary reject " ++ s) "Error" (show (P.parse s))
    check ("integrated reject " ++ s) "Error" (show (IP.parse s))
  check "ordinary power" (asExp (T.Term (T.Pow (n "2") (n "3")))) (show (P.parse "2^3"))
  forM_ ["2^3^4","2*3^4","2/3^4"] $ \s -> check ("restricted power " ++ s) "Error" (show (P.parse s))
  check "integrated no power" "Error" (show (IP.parse "2^3"))
  let atom f = IT.Term (IT.NFactor (IT.Factor f))
      x = atom (IT.Var (IT.A_Var "x"))
      lo = atom (IT.Intnum (IT.An_Intnum "0"))
      hi = atom (IT.Intnum (IT.An_Intnum "1"))
  forM_ [("int",IT.Integrate),("fmax",IT.FMax),("fmin",IT.FMin)] $ \(name,ctor) -> do
    let expected = IT.Exp1 (IT.Term (IT.NFactor (IT.Factor (ctor (x,IT.A_Var "x",lo,hi)))))
    check ("functional AST " ++ name) expected (IP.parse (name ++ "(x,x,0,1)"))
  putStrLn ("parser\t" ++ show (2*length common+2*length bad+8) ++ "\tpass")

main = do
  args <- getArgs
  case args of
    [] -> lexicalChecks >> parserChecks
    ["observe",expression] -> do
      print (P.parse expression)
      print (IP.parse expression)
    _ -> error "usage: parser-probe [observe EXPRESSION]"
