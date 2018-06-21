%{
#include <stdio.h>
extern int yydebug;
%}
%token LCB RCB SEMIC INT PRELUDE MULT COLON EXCLAMATION LBR RBR
%token CALLOC GREATERT LESST HASH GOTO ASSIGNMENT PUTCHAR SIZEOF GETCHAR
%token UAND PLUS MINUS DIVIDE FSTOP LSB RSB COMMA AND END OR FALSE
%token TRUE NOT WRITE READ DO IF THEN ELSIF WHILE ELSE RETURN INTEGER
%token DIGIT BOOLEAN ARRAY OF CBEGIN VAR 
%token PROCEDURE NUMBER LETTER EQUALS 
%%
program  : module2 mprocdecl beginn END FSTOP
         ;
pardecl  : varn ident COLON type
         ;
type     : basetype 
         | ARRAY number OF basetype
         ;
module   : module2 beginn END SEMIC
         ;
module2  : module2 VAR ident COLON type SEMIC {/* $$=declare_var($3,$5)*/}
         |
         ;
varn     : VAR
         |
         ;
mprocdecl: mprocdecl procdecl 
         |
         ;
procdecl : PROCEDURE ident paramsn SEMIC module
         ;
paramsn  : params
         |
         ;
params   : LBR pardecl pardecl2 RBR
         ;
pardecl2 : pardecl2 SEMIC pardecl
         | 
         ;
beginn   : CBEGIN statseq
         |
         ; 
basetype : INTEGER
         | BOOLEAN
         ;
statseq  : statement statseq2
         ;
statseq2 : statseq2 SEMIC statement
         | 
         ;
statement : assignment
         | procall
         | ifstat
         | whilestat
         | iostat
         | RETURN
         ;
assignment : qualident ASSIGNMENT expression
         ;
procall  : ident LBR procall2 RBR
         ;
procall2 : expression expressionm
         |
         ;
expressionm : expressionm COMMA expression
         |
         ;
ifstat   : IF expression THEN statseq elsem elsen END
         ;
elsem    : elsem ELSIF expression THEN statseq
         |
         ;
elsen    : ELSE statseq
         |
         ;
whilestat : WHILE expression DO statseq END
         ;
iostat   : READ ident
         | WRITE expression
         ;
qualident: ident
         | ident LSB expression RSB
         ;
expression : simpexpr simpexprn
         ;
simpexprn : relation simpexpr
         |
         ;
relation : GREATERT
         | LESST
         | EQUALS 
         | HASH
         ;
simpexpr : plusorminus term term2
         ;
plusorminus : PLUS
            | MINUS
            |
         ;
term2    :term2 addop term  
         | 
         ;
term     :factor factor2
         ;
factor2  : factor2 mulop factor
         |
         ;
factor   : NOT factor
         | LBR expression RBR
         | qualident 
         | number   
         | TRUE     
         | FALSE    
         ;
addop    : PLUS
         | MINUS
         | OR
         ;
mulop    : MULT
         | DIVIDE
         | AND
         ;
ident    : LETTER letterm
         ;
letterm  : letterm LETTER | DIGIT 
         |
         ;
number   : DIGIT digitm 
         ;
digitm   : digitm DIGIT 
         |
         ;  
%%
main()
{
 yydebug=1;
 yyparse();
}
