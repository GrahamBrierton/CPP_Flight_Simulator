	ifndef	??version
?debug	macro
	endm
publicdll macro	name
	public	name
	endm
	endif
	?debug	V 300h
	?debug	S "HSUB1.CPP"
	?debug	C E9265AB41A0948535542312E435050
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
	?debug	C E8010948535542312E435050265AB41A
   ;
   ;	hsub1(int addr,char mask)
   ;
	?debug	L 1
	assume	cs:_TEXT
	public  @hsub1$qizc
@hsub1$qizc	proc	near
	?debug	B
	push	bp
	mov	bp,sp
	?debug	C E6046D61736B020A0600000461646472040A0400+
	?debug	C 00
	?debug	B
   ;
   ;	{
	 mov ax,0A000h
	 mov ds,ax
	 mov si,[bp+6]
	 mov ax,3ceh
	 mov dx,ax
	 mov al,8
	 out dx,al
	 inc dx
	 mov al,[bp+8]
	 out dx,al
	 mov al,ds:[si]
	 mov ds:[si],al
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
	?debug	C E31800000023040000
	?debug	C EC0B4068737562312471697A63181800
	end
