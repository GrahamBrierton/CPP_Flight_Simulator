	ifndef	??version
?debug	macro
	endm
publicdll macro	name
	public	name
	endm
	endif
	?debug	V 300h
	?debug	S "HSUB4.CPP"
	?debug	C E95670B41A0948535542342E435050
_TEXT	segment byte public 'CODE'
_TEXT	ends
DGROUP	group	_DATA,_BSS
	assume	cs:_TEXT,ds:DGROUP
_DATA	segment word public 'DATA'
d@	label	byte
d@w	label	word
_DATA	ends
_BSS	segment word public 'BSS'
b@	label	byte
b@w	label	word
_BSS	ends
_TEXT	segment byte public 'CODE'
	?debug	C E8010948535542342E4350505670B41A
   ;	
   ;	void hsub1(int addr,char mask)
   ;	
	?debug	L 1
	assume	cs:_TEXT
@hsub1$qizc	proc	near
	?debug	B
	push	bp
	mov	bp,sp
	?debug	C E6046D61736B020A0600000461646472040A0400+
	?debug	C 00
	?debug	B
   ;	
   ;	{
   ;	}
   ;	
	?debug	L 3
	pop	bp
	ret	
	?debug	C E60461646472040A040000046D61736B020A0600+
	?debug	C 00
	?debug	E
	?debug	E
@hsub1$qizc	endp
	?debug	C E9
	?debug	C FA00000000
_TEXT	ends
_DATA	segment word public 'DATA'
s@	label	byte
_DATA	ends
_TEXT	segment byte public 'CODE'
_TEXT	ends
	public	@hsub1$qizc
_s@	equ	s@
	?debug	C EA0509
	?debug	C E31800000023010000
	?debug	C EC0B4068737562312471697A63181800
	end
