	ifndef	??version
?debug	macro
	endm
publicdll macro	name
	public	name
	endm
	endif
	?debug	V 300h
	?debug	S "HSUB3.CPP"
	?debug	C E9045BB41A0948535542332E435050
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
	?debug	C E8010948535542332E435050045BB41A
   ;	
   ;	hsub3(int address,char mask1,char mask2,int nbyte)
   ;	
	?debug	L 1
	assume	cs:_TEXT
	PUBLIC  @hsub3$qizczci
@hsub3$qizczci	proc	near
	?debug	B
	push	bp
	mov	bp,sp
	?debug	C E6056E62797465040A0A0000056D61736B32020A+
	?debug	C 080000056D61736B31020A060000076164647265+
	?debug	C 7373040A040000
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
	mov al,0ffh
	out dx,al
	mov cx,[bp+12]
 a1 :   inc si
	mov al,ds:[si]
	mov ds:[si],al
	loop a1
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
	?debug	C E60761646472657373040A040000056D61736B31+
	?debug	C 020A060000056D61736B32020A080000056E6279+
	?debug	C 7465040A0A0000
	?debug	E
	?debug	E
@hsub3$qizczci	endp
	?debug	C E9
	?debug	C FA00000000
_TEXT	ends
_DATA	segment word public 'DATA'
s@	label	byte
_DATA	ends
_TEXT	segment byte public 'CODE'
_TEXT	ends
	public	@hsub3$qizczci
_s@	equ	s@
	?debug	C EA0509
	?debug	C E31800000023040000
	?debug	C EC0E4068737562332471697A637A6369181800
	end
