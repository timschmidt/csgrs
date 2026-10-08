NICE = nice

#HC = ghc-2.10
HC = ghc
HC_PROFA = #-prof -auto 
HC_PROFB = #-prof -auto 
HC_PROF_LINK = #-prof -auto
HC_NO_O =  -H35m -K12m
#HC_NO_O = -H20m 
HC_OPTS = -O $(HC_NO_O) 

HAPPY = happy
ALEX = alex

SRCS = Alex.hs DyStream.hs IParseTree.hs ParseTree.hs Utils.hs Calc.hs \
       Functions.hs IParser.hs Parser.hs X_Repn.hs Convert.hs ICalc.hs \
       IReal.hs SBinDec.hs Database.hs IDatabase.hs Integr.hs SBinFloat.hs \
       DyDigit.hs ILexer.hs Lexer.hs SBinStream.hs DyFloat.hs IMain.hs \
       Main.hs

TESTOBJS = Utils.o \
       SBinStream.o SBinFloat.o \
       DyDigit.o DyStream.o DyFloat.o \
       SBinDec.o \
       Convert.o X_Repn.o \
       IReal.o Functions.o \
       proftest.o

TESTOBJS2 = Utils.o \
       SBinStream.o SBinFloat.o \
       DyDigit.o DyStream.o DyFloat.o \
       SBinDec.o \
       Convert.o X_Repn.o \
       Tests.o Test.o \
       Functions.o  IReal.o Integr.hs

DEMO1OBJS = Utils.o \
       SBinStream.o SBinFloat.o \
       SBinDec.o \
       Interact.o \
       Demos.o demo1.o 

OBJS = Utils.o \
       SBinStream.o SBinFloat.o \
       DyDigit.o DyStream.o DyFloat.o \
       SBinDec.o \
       Convert.o X_Repn.o \
       IReal.o Functions.o Integr.o \
       Interact.o \
       Alex.o Lexer.o Parser.o \
       ParseTree.o Database.o Calc.o \
       Main.o

IOBJS = Utils.o \
       SBinStream.o SBinFloat.o \
       DyDigit.o DyStream.o DyFloat.o \
       SBinDec.o \
       Convert.o X_Repn.o \
       IReal.o Functions.o Integr.o \
       Interact.o \
       Alex.o ILexer.o IParser.o \
       IParseTree.o IDatabase.o ICalc.o \
       IMain.o


CGIOBJS = Utils.o \
       SBinStream.o SBinFloat.o \
       DyDigit.o DyStream.o DyFloat.o \
       SBinDec.o \
       Convert.o X_Repn.o \
       IReal.o Functions.o Integr.o \
       Interact.o \
       Alex.o Lexer.o Parser.o \
       ParseTree.o Database.o Calc.o \
       WebSite.o CGI.o CGISystem.o HTML.o HTMLWizard.o \
       Pretty.o PrettyHTML.o Mime.o Request.o Response.o \
       Parsing.o URL.o UrlEncoded.o \
       demo.o Test.o

.SUFFIXES : .o .hi .hs .lhs .hc .s .x .y

all:	calc


icalc: 	$(IOBJS)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(IOBJS)

calc:	$(OBJS)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(OBJS)

proftest: $(TESTOBJS)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(TESTOBJS)

test:	$(TESTOBJS2)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(TESTOBJS2)

demo1:  $(DEMO1OBJS)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(DEMO1OBJS)

cgi:    $(CGIOBJS)
	$(RM) $@
	$(NICE) $(HC) -o $@ $(HC_OPTS) $(HC_PROF_LINK) $(CGIOBJS)

clean:
	$(RM) icalc calc *.o *.hi



.x.o:	
	$(RM) $@
	$(NICE) $(ALEX) $<
	$(NICE) $(HC) $(HC_OPTS) $*.hs 

.y.o:	
	$(RM) $@
	$(NICE) $(HAPPY) $<
	$(NICE) $(HC) -c $(HC_NO_O) $*.hs 

.y:
	$(RM) $@
	$(NICE) $(HAPPY) $<





.o.hi:
	@:




Parser.hs: ./Parser.y
Lexer.hs: ./Lexer.x

proftest.o: proftest.hs SBinDec.hi Functions.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Alex.o : Alex.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Calc.o : Calc.hs Parser.hi ParseTree.hi Database.hi Parser.hi ParseTree.hi \
	 Database.hi SBinFloat.hi X_Repn.hi Functions.hi Integr.hi SBinDec.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Convert.o : Convert.hs SBinStream.hi SBinFloat.hi DyDigit.hi DyStream.hi \
	    DyFloat.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Database.o : Database.hs ParseTree.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

DyDigit.o : DyDigit.hs 
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs


DyStream.o : DyStream.hs DyDigit.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

DyFloat.o : DyFloat.hs DyStream.hi DyDigit.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Functions.o : Functions.hs Utils.hi SBinStream.hi SBinFloat.hi SBinDec.hi \
	      IReal.hi X_Repn.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

ICalc.o : ICalc.hs IParser.hi IParseTree.hi IDatabase.hi SBinFloat.hi \
	  X_Repn.hi Functions.hi Integr.hi SBinDec.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

IDatabase.o : IDatabase.hs IParseTree.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

ILexer.o : ILexer.hs Alex.hi IParseTree.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB)  $*.hs


IMain.o : IMain.hs ICalc.hi SBinDec.hi Interact.hi IDatabase.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

IParseTree.o : IParseTree.hs Alex.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


IParser.o : IParser.hs IParseTree.hi ILexer.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_NO_O) $(HC_PROFB) $*.hs

IReal.o : IReal.hs Utils.hi SBinStream.hi SBinFloat.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Integr.o : Integr.hs SBinStream.hi SBinFloat.hi SBinDec.hi X_Repn.hi \
	   DyDigit.hi DyStream.hi DyFloat.hi Convert.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Interact.o : Interact.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


Lexer.o : Lexer.hs Alex.hi ParseTree.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Main.o : Main.hs Calc.hi SBinDec.hi Interact.hi Database.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

ParseTree.o : ParseTree.hs Alex.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


Parser.o : Parser.hs ParseTree.hi Lexer.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_NO_O) $(HC_PROFB) $*.hs

SBinDec.o : SBinDec.hs Utils.hi SBinStream.hi SBinFloat.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

SBinFloat.o : SBinFloat.hs Utils.hi SBinStream.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

SBinStream.o : SBinStream.hs Utils.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Utils.o : Utils.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

X_Repn.o : X_Repn.hs SBinStream.hi SBinFloat.hi DyDigit.hi DyStream.hi \
	   DyFloat.hi Convert.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFA) $*.hs

Tests.o : Tests.hs SBinStream.hi SBinFloat.hi SBinDec.hi DyStream.hi \
          DyFloat.hi Convert.hi X_Repn.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Test.o : Test.hs Tests.hi SBinDec.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


Demos.o : Demos.hs Utils.hi SBinDec.hi SBinFloat.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


demo1.o : demo1.hs Interact.hi SBinDec.hi Demos.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs


CGI.o 	: CGI.hs Request.hi Response.hi Mime.hi CGISystem.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Request.o	: Request.hs Mime.hi CGISystem.hi UrlEncoded.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Response.o	: Response.hs Mime.hi URL.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Mime.o	: Mime.hs HTMLWizard.hi PrettyHTML.hi UrlEncoded.hi CGISystem.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

CGISystem.o : CGISystem.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

UrlEncoded.o : UrlEncoded.hs Parsing.hi Pretty.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

URL.o	: URL.hs Parsing.hi Pretty.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

HTMLWizard.o	: HTMLWizard.hs HTML.hi URL.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

PrettyHTML.o	: PrettyHTML.hs HTML.hi Pretty.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Parsing.o : Parsing.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Pretty.o : Pretty.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

HTML.o  : HTML.hs
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

WebSite.o : WebSite.hs CGI.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

demo.o : demo.hs CGI.hi Test.hi WebSite.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs

Test.o : Test.hs CGI.hi Calc.hi Database.hi
	$(RM) $@ 
	$(NICE) $(HC) -c $(HC_OPTS) $(HC_PROFB) $*.hs



