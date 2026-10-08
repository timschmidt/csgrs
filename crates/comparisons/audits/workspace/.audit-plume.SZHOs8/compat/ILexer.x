%{
module ILexer (lexer) where

import Alex
import IParseTree
%}


{ ^d = 0-9      }			-- digits
{ ^l = [a-zA-Z] }			-- alphabetic characters

"tokens_lx"/"tokens_acts":-

  <>        ::=  ^w+			-- white space
  <intnum>  ::=  ^d+			%{ intnum  p s = TokenIntnum s %}
  <num>     ::=  ^d+^.^d+		%{ num     p s = TokenNum s %}	
  <assign>  ::=  ^:^=			%{ assign  p s = TokenAssign %}
  <oplus>   ::=  ^+			%{ oplus   p s = TokenPlus %}
  <ominus>  ::=  ^-			%{ ominus  p s = TokenMinus %}
  <otimes>  ::=  ^*			%{ otimes  p s = TokenTimes %}
  <odiv>    ::=  ^/			%{ odiv    p s = TokenDiv %}
  <ob>      ::=  ^(			%{ ob      p s = TokenOB %}
  <cb>      ::=  ^)			%{ cb      p s = TokenCB %}
  <comma>   ::= ^,			%{ comma   p s = TokenComma %}
  <fsin>    ::=  sin			%{ fsin    p s = TokenSin %}
  <fcos>    ::=  cos			%{ fcos    p s = TokenCos %}
  <farctan> ::=  arctan			%{ farctan p s = TokenArctan %}
  <fexp>    ::=  exp			%{ fexp    p s = TokenExp %}
  <fln>     ::=  ln			%{ fln     p s = TokenLn %}
  <fsqrt>   ::=  sqrt			%{ fsqrt   p s = TokenSqrt %}
  <fmax>    ::=  max			%{ fmax    p s = TokenMax %}
  <fmin>    ::=  min			%{ fmin    p s = TokenMin %}
  <fint>    ::=  int			%{ fint    p s = TokenIntegr %}
  <ffmax>   ::=  fmax			%{ ffmax   p s = TokenFMax %}
  <ffmin>   ::=  fmin			%{ ffmin   p s = TokenFMin %}
  <cpi>     ::=  pi			%{ cpi     p s = TokenPi %}
  <digits>  ::=  Digits			%{ digits  p s = TokenDigits %}
  <bye>     ::=  exit			%{ bye     p s = TokenExit %}
  <var>     ::=  ^l[^l^d^_^']*		%{ var     p s = TokenVar s  %}
 

%{

lexer:: String -> [Token]
lexer inp = scan tokens_scan inp

tokens_scan:: Scan Token
tokens_scan = load_scan (tokens_acts,stop_act) tokens_lx
	where
	stop_act p ""  = []
	stop_act p inp = [Err p]
%}
