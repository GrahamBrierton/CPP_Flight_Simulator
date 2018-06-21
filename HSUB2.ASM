	ifndef	??version
?debug	macro
	endm
publicdll macro	name
	public	name
	endm
	endif
	?debug	V 300h
	?debug	S "HSUB2.CPP"
	?debug	C E98B5AB41A0948535542322E435050
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
	?debug	C E8010948535542322E4350508B5AB41A
   ;
   ;	hsub2(int addr,char mask1,char mask2)
   ;
	?debug	L 1
	assume	cs:_TEXT
	PUBLIC @hsub2$qizczc
@hsub2$qizczc	proc	near
	?debug	B
	push	bp
	mov	bp,sp
	?debug	C E6056D61736B32020A080000056D61736B31020A+
	?debug	C 0600000461646472040A040000
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
	mov al,[bp+10]
	out dx,al
	inc si
	mov al,ds:[si]
	mov ds:[si],al
   ;	}
   ;	
	?debug	L 3
	pop	bp
	ret	
	?debug	C E60461646472040A040000056D61736B31020A06+
	?debug	C 0000056D61736B32020A080000
	?debug	E
	?debug	E
@hsub2$qizczc	endp
	?debug	C E9
	?debug	C FA00000000
_TEXT	ends
_DATA	segment word public 'DATA'
s@	label	byte
_DATA	ends
_TEXT	segment byte public 'CODE'
_TEXT	ends
	public	@hsub2$qizczc
_s@	equ	s@
	?debug	C EA0509
	?debug	C E31800000023040000
	?debug	C EC0D4068737562322471697A637A63181800
	end
