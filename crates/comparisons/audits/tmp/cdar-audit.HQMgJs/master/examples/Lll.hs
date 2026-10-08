--module Lll where
{-# LANGUAGE FlexibleContexts #-}

--import Control.Monad
import Data.CDAR
import Data.Foldable (toList)
--import Data.Int
import Data.Sequence (Seq (..))
import Data.Sequence as S
--import Data.Word
import Text.Parsec as P
import Text.Parsec.String (Parser)

type U = Int -- Word64
type I = Int -- Int64

data Cell
  = CU U
  | CI I
  | CAddr U
  | CZ Integer
  | CReal CR
  | CSeq (Seq Cell) deriving (Eq,Ord,Read)

instance Show Cell where
  show (CU u) = "CU " ++ show u
  show (CI u) = "CI " ++ show u
  show (CAddr u) = "CAddr " ++ show u
  show (CZ u) = "CZ " ++ show u
  show (CReal u) = "CR " ++ showCR 1 u
  show (CSeq u) = "CSeq " ++ show u

data PgmState = PgmState
  { pgm :: Seq Op
  , pc :: U
  , stack :: Seq Cell
  , rstack :: Seq U
  } deriving (Eq,Ord,Read)

instance Show PgmState where
  show (PgmState p cpc s r) = "PgmState " ++ show cpc ++ ' ' : show (S.index p cpc) ++ ' ' : show s ++ ' ' : show r

data Op
  = Dup U U | Pop U | Rot U U
  | Apush U | Scall U | Dcall | Ret | Jmp U | Jnz U
  | Ipush I | Ineg | Iadd | Imul | Idiv | Isgn
  | Zconv | Zneg | Zadd | Zmul | Zdiv | Zsgn | Zsh
  | Rconv | Rneg | Radd | Rmul | Rdiv | Rsh | Rch | Rlim U
  | Entc | Lvc U | Noop | Unresolved Op String
  deriving (Eq,Ord,Read,Show)

data Result = Success PgmState | Error String | Result PgmState

-- Parser

pident :: Parser String
pident = (:) <$> letter <*> many alphaNum

plabel :: Parser String
plabel = pident <* char ':'

peol :: Parser ()
peol = optional (char '#' *> many (noneOf "\r\n")) <* lexeme endOfLine

whitespace :: Parser String
whitespace = many (oneOf " \t")

lexeme :: Parser a -> Parser a
lexeme p = p <* whitespace

pline :: Parser (Maybe String, Op)
pline = (,) <$> optionMaybe (try (lexeme plabel)) <*> option Noop (lexeme pop)

pint :: Parser I
pint = read <$> many1 (oneOf "-0123456789") <* whitespace

puns :: Parser U
puns = read <$> many1 (oneOf "0123456789") <* whitespace

pop :: Parser Op
pop = choice $ map try
      [ Dup <$> (keyword "dup" *> puns) <*> puns
      , Pop <$> (keyword "pop" *> puns)
      , Rot <$> (keyword "rot" *> puns) <*> puns
      , Unresolved (Apush 0) <$> (keyword "apush" *> pident)
      , Unresolved (Scall 0) <$> (keyword "scall" *> pident)
      , Dcall <$ keyword "dcall"
      , Ret <$ keyword "ret"
      , Unresolved (Jmp 0) <$> (keyword "jmp" *> pident)
      , Unresolved (Jnz 0) <$> (keyword "jnz" *> pident)
      , Ipush <$> (keyword "ipush" *> pint)
      , Ineg <$ keyword "ineg"
      , Iadd <$ keyword "iadd"
      , Imul <$ keyword "imul"
      , Idiv <$ keyword "idiv"
      , Isgn <$ keyword "isgn"
      , Zconv <$ keyword "zconv"
      , Zneg <$ keyword "zneg"
      , Zadd <$ keyword "zadd"
      , Zmul <$ keyword "zmul"
      , Zdiv <$ keyword "zdiv"
      , Zsgn <$ keyword "zsgn"
      , Zsh <$ keyword "zsh"
      , Rconv <$ keyword "rconv"
      , Rneg <$ keyword "rneg"
      , Radd <$ keyword "radd"
      , Rmul <$ keyword "rmul"
      , Rdiv <$ keyword "rdiv"
      , Rsh <$ keyword "rsh"
      , Rch <$ keyword "rch"
      , Unresolved (Rlim 0) <$> (keyword "rlim" *> pident)
      , Entc <$ keyword "entc"
      , Lvc <$> (keyword "lvc" *> puns)
      ]

keyword :: String -> Parser String
keyword s = lexeme (string s)

removeNoop :: [String] -> [(Maybe String, Op)] -> [([String], Op)]
removeNoop _ [] = []
removeNoop ls ((l, Noop) : rows) = removeNoop (toList l ++ ls) rows
removeNoop ls ((l, x) : rows) = (toList l ++ ls, x) : removeNoop [] rows

extractLabels :: [[String]] -> [(String,U)]
extractLabels ls = let as = Prelude.zip ls [0..]
                       f (xs, n) = map (flip (,) n) xs
                   in concatMap f as

resolveLabels :: [([String],Op)] -> [Op]
resolveLabels xs =
  let (ls, os) = Prelude.unzip xs
  in map (res (extractLabels ls)) os
  where res els (Unresolved (Apush _) l) = maybe (error "Label not found") Apush (Prelude.lookup l els)
        res els (Unresolved (Scall _) l) = maybe (error "Label not found") Scall (Prelude.lookup l els)
        res els (Unresolved (Jmp _) l) = maybe (error "Label not found") Jmp (Prelude.lookup l els)
        res els (Unresolved (Jnz _) l) = maybe (error "Label not found") Jnz (Prelude.lookup l els)
        res els (Unresolved (Rlim _) l) = maybe (error "Label not found") Rlim (Prelude.lookup l els)
        res _ x = x

pprogram :: Parser [(Maybe String, Op)]
pprogram = whitespace *> sepBy pline peol

readProgram :: FilePath -> IO [Op]
readProgram name = do
  p <- readFile name
  let rows = parse pprogram name p
  return . resolveLabels . removeNoop [] . either undefined id $ rows

-- Interpreter

step :: Op -> PgmState -> Result
step op s@(PgmState _p cpc st rst) =
  case op of
    Dup n k | S.length st >= n -> Result $ s { stack = (S.take k . S.drop (n-k) $ st) >< st }
    Pop n | S.length st >= n -> Result $ s { stack = S.drop n $ st }
    Rot n k | S.length st >= n -> Result $ s { stack = let (as, ss) = S.splitAt n $ st
                                                           (bs, cs) = S.splitAt (n-k) as
                                                       in cs >< bs >< ss }
    Apush a -> Result $ s { stack = CAddr a <| st }
    Scall a -> Result $ s { pc = a, rstack = cpc <| rst }
    Dcall | S.length st >= 1 -> case st of
                                  CAddr a :<| ss -> Result $ s { pc = a, stack = ss, rstack = cpc <| rstack s }
                                  _ -> Main.Error "Dcall: Not an address"
    Ret -> case rstack s of
             (a :<| rs) -> Result $ s { pc = a, rstack = rs }
             S.Empty -> Success $ s
    Jmp a -> Result $ s { pc = a }
    Jnz a | S.length st >= 1 -> case st of
                                  (CI i :<| ss) -> Result $ if i /= 0
                                                            then s { pc = a, stack = ss }
                                                            else s { stack = ss }
                                  _ -> Main.Error "Jnz: Stack underflow"
    Ipush i -> Result $ s { stack = CI i <| st }
    Ineg -> iunary negate
    Iadd -> ibinary (+)
    Imul -> ibinary (*)
    Idiv -> ibinary div
    Isgn -> iunary signum
    Zconv -> case st of
               (CI a :<| ss) -> Result $ s { stack = CZ (fromIntegral a) <| ss }
               _ -> Main.Error "Zconv: Stack underflow or not an int."
    Zneg -> zunary negate
    Zadd -> zbinary (+)
    Zmul -> zbinary (*)
    Zdiv -> zbinary div
    Zsgn -> zunary signum
    Zsh -> case st of
             (CI a :<| CZ b :<| ss) -> Result $ s { stack = CZ (scale b a) <| ss }
             _ -> Main.Error "Zsh: Stack underflow or not correct types."
    Rconv -> case st of
               (CZ a :<| ss) -> Result $ s { stack = CReal (fromIntegral a) <| ss }
               _ -> Main.Error "Rconv: Stack underflow or not an integer."
    Rneg -> runary negate
    Radd -> rbinary (+)
    Rmul -> rbinary (*)
    Rdiv -> rbinary (/)
    Rsh -> case st of
             (CI a :<| CReal b :<| ss) -> Result $ s { stack = CReal (scale b a) <| ss }
             _ -> Main.Error "Rsh: Stack underflow or not correct types."
    Rch -> case st of
             (CI n :<| ss) -> let (cs, ss') = S.splitAt n ss
                                  (Select i) = selectCR (map (\(CReal x) -> x) (toList cs))
                              in Result $ s { stack = CI i <| ss' }
             _ -> Main.Error "Rsh:"
    Rlim a -> let f s' n = last . interpret $ s' { pc = a, stack = CI n <| st, rstack = S.Empty }
                  g n = case stack $ f s n of
                          CReal x :<| _ -> x
                          _s -> error $ "Rlim: Not correctly called.\n" ++ show _s
                  y = limCR g
              in Result $ s { stack = CReal y <| st }
    
    Entc -> Result s
    Lvc _ -> Result s
    _ -> Main.Error "Unknown op code"
  where
    iunary f = case st of
                 (CI a :<| ss) -> Result $ s { stack = CI (f a) <| ss }
                 _ -> Main.Error "Iunary: Stack underflow or not an int."
    ibinary f = case st of
                  (CI a :<| CI b :<| ss) -> Result $ s { stack = CI (f b a) <| ss }
                  _ -> Main.Error "Ibinary: Stack underflow or not an int."
    zunary f = case st of
                 (CZ a :<| ss) -> Result $ s { stack = CZ (f a) <| ss }
                 _ -> Main.Error "Zunary: Stack underflow or not an integer."
    zbinary f = case st of
                  (CZ a :<| CZ b :<| ss) -> Result $ s { stack = CZ (f b a) <| ss }
                  _ -> Main.Error "Zbinary: Stack underflow or not an integer."
    runary f = case st of
                 (CReal a :<| ss) -> Result $ s { stack = CReal (f a) <| ss }
                 _ -> Main.Error "Runary: Stack underflow or not a real."
    rbinary f = case st of
                  (CReal a :<| CReal b :<| ss) -> Result $ s { stack = CReal (f b a) <| ss }
                  _ -> Main.Error ("Rbinary: Stack underflow or not a real.\n" ++ show s)

interpret :: PgmState -> [PgmState]
interpret t =
  let cpc = pc t
      op = index (pgm t) cpc
      s = t { pc = cpc+1 }
  in case step op s of
    (Result s') -> t : interpret s'
    (Success s') -> [s']
    (Main.Error e) -> error e
    -- _ -> error ""

interpretFile :: FilePath -> Seq Cell -> IO (Seq Cell)
interpretFile name s = do
  p <- readProgram name
  return . stack . last . interpret $ PgmState (fromList p) 0 s S.Empty

main :: IO ()
main = do
  r <- interpretFile "examples/bounded.lll" (CReal (0.001) <| S.Empty)
  putStrLn (show r)

test :: Seq Op -> Int -> Seq Cell -> IO ()
test p initpc s = do
  let r = interpret (PgmState p initpc s S.Empty)
  mapM_ (putStrLn . show) r
  putStrLn . show . stack . last $ r
