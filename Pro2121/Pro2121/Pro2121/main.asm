;
; COMP2121 elevator project
; Pro2121.asm
;
; Created: 5/9/2018 11:37:18 PM
; Author : King

.include "m2560def.inc"


.def row   = r16           ; current row number
.def col   = r17           ; current column number
.def rmask = r18           ; mask for current row during scan
.def cmask = r19           ; mask for current column during scan
.def Streg = r20		   ; statu register
.def strobepattern = r21
.def temp1 = r22
.def temp2 = r23

.equ PORTLDIR    = 0xF0    ; PD7-4: output, PD3-0:input
.equ INITCOLMASK = 0xEF    ; scan from the rightmost column
.equ INITROWMASK = 0x01    ; scan from the top row
.equ ROWMASK     = 0x0F    ; for obtaining input form Port D


;---------------
;|MARCO set		|
;----------------
;|LCD marco set

.equ LCD_RS = 7
.equ LCD_E  = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

.macro clear
    ldi YL, low(@0)        ; load the memory address to Y
    ldi YH, high(@0)
    clr temp1
    st Y+, temp1           ; clear the two bytes at @0 in SRAM
    st Y, temp1
.endmacro

.macro do_lcd_command
  ldi temp1, @0
  rcall lcd_command
  rcall lcd_wait
.endmacro

.macro do_lcd_data
	ldi temp1, @0
    rcall lcd_data
    rcall lcd_wait
.endmacro

.macro lcd_set
  sbi PORTA, @0
.endmacro

.macro lcd_clr
  cbi PORTA, @0
.endmacro

.macro do_lcd_data_print
    mov temp1, @0
    rcall lcd_data
    rcall lcd_wait
.endmacro

.macro print_floor
    push temp2
	push temp1
	lds temp1,FloorNumber
    mov temp2, temp1
    subi temp2, -'0'
    do_lcd_data_print temp2
	pop temp1
    pop temp2
.endmacro

.macro print_current_floor
    lds r2, FloorNumber
    print_digit r2
.endmacro

.macro lcd_clrscreen
    do_lcd_command 0b00111000 ; 2x5x7
    rcall sleep_5ms
    do_lcd_command 0b00111000 ; 2x5x7
    do_lcd_command 0b00111000 ; 2x5x7
    do_lcd_command 0b00001000 ; display off?
    do_lcd_command 0b00000001 ; clear display
    do_lcd_command 0b00000110 ; increment, no display shift
    do_lcd_command 0b00001110 ; cursor on, bar, no blink
.endmacro

.macro lcd_floor
    do_lcd_data 'F'
    do_lcd_data 'L'
    do_lcd_data 'O'
    do_lcd_data 'O'
    do_lcd_data 'R'
    do_lcd_data ':'
    do_lcd_data ' '
.endmacro


.macro lcd_emergency
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data 'E'
    do_lcd_data 'm'
    do_lcd_data 'e'
    do_lcd_data 'r'
    do_lcd_data 'g'
    do_lcd_data 'e'
    do_lcd_data 'n'
    do_lcd_data 'c'
    do_lcd_data 'y'
    do_lcd_command 0b11000000
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data 'C'
    do_lcd_data 'a'
    do_lcd_data 'l'
    do_lcd_data 'l'
    do_lcd_data ' '
    do_lcd_data '0'
    do_lcd_data '0'
    do_lcd_data '0'
.endmacro


.dseg
TempCounter: .byte 2
SecondCounter: .byte 2
CurrentPattern: .byte 2      ; Pattern for the LEDs to represent moving/doors/lift
FloorNumber: .byte 2         ; Floor Number
FloorBits: .byte 2			 ;Using shifting lsl or rsl
FloorQueue: .byte 2			 ;Using to queue

.cseg
.org 0x0000
    jmp RESET
.org INT0addr
    jmp EXT_INT0
.org OVF0addr
    jmp Timer0OVF
	jmp DEFAULT

DEFAULT: reti

RESET:
    ldi temp1, low(RAMEND)   ;initialize the stack
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1

    ; Keyboard set up
    ldi temp1, PORTLDIR      ;PL7:4/Pl3:0, out/in
    sts DDRL, temp1

    ;input/output set up
    ser temp1               
    out DDRC, temp1          ;PORTC is output
    out PORTC, temp1		 ;out PORTC all leds light
	out DDRG, temp1          ;PORTG is output
	out PORTG, temp1         ;PORTG all leds light 
	out DDRF, temp1			;PORTF is output
    out DDRA, temp1			;PORTA is output
                   
    out DDRB, temp1			; PORTB is output
    clr temp1
    out PORTB, temp1         ; using in emrgency

    clr temp1
    out PORTF, temp1		;LCD is clr
    out PORTA, temp1		

    ; Initial LCD Message
    ldi temp1, 0xFF
    out PORTC, temp1
    out PORTG, temp1

    ; Motor set up
    ser temp1  
    out DDRE, temp1     ;PORTE is output

    ldi temp1, 0b00000000
    sts OCR3BL, temp1
    clr temp1
    sts OCR3BH, temp1

    ldi temp1, (1<<CS30)
    sts TCCR3B, temp1
    ldi temp1, (1<<WGM30)|(1<<COM3B1)
    sts TCCR3A, temp1

	; Strobe Light Setup
    ser temp1                ; PORTB is output
    out DDRB, temp1
    clr temp1
    out PORTB, temp1         ; only use in emrgency

	; Timer Set up
    ldi temp1, 0b00000000
    out TCCR0A, temp1
    ldi temp1, 0b00000010
    out TCCR0B, temp1                 ; Prescaling value = 8
    ldi temp1, 1<<TOIE0               ; = 128 microseconds
    sts TIMSK0, temp1                 ; T/C0 interrupt enable

	;statu set up
	clr Streg 
	clear TempCounter
	clear SecondCounter
	clear FloorNumber
	clear FloorBits
	clear	FloorQueue 
	clear CurrentPattern
	
    ; close bottom Setup
    ldi temp1, (2 << ISC00)              ; set INT0 as falling-
    sts EICRA, temp1                     ; edge triggered interrupt
    in temp1, EIMSK                      ; enable INT0
    ori temp1, (1<<INT0)
    out EIMSK, temp1

  ; set floor to 0 in lcd
    lcd_clrscreen
    lcd_floor
    do_lcd_data '0'

    sei                               ; Enable global interrupt
	rjmp main



Timer0OVF:
    in temp1, SREG
    push temp1                      ; Prologue starts
    push temp2
    push r29
    push r28
    push r27
    push r26
    push r25
    push r24                        ; Prologue ends
                                    ; Load the vlaue of the temporary counter
	
    lds r24, TempCounter
    lds r25, TempCounter+1
    adiw r25:r24, 1
	ldi temp1, high(7812)
    cpi r24, low(7812)
    cpc r25, temp1
    brne NotSecond
    clear TempCounter
	lds r24, SecondCounter
    lds r25, SecondCounter+1
    adiw r25:r24, 1
	sts SecondCounter, r24
	sts SecondCounter+1, r25
	rjmp EndIF
	NotSecond:
		
		sts TempCounter, r24
		sts TempCounter+1, r25

    CheckDoor:
        ; check if door moving
        mov temp2, streg
		andi temp2, 0b00010000
	    cpi temp2, 0b00010000
        brne CheckOpenDoor
        
        mov temp2, streg
		andi temp2, 0b00100000
	    cpi temp2, 0b00100000
        breq DoorClosing
        andi streg, 0b11101111
        ori streg, 0b00100000
        ;pattern to LEDS
        ldi temp1, 0xC6     ; door moving pattern
        out PORTC, temp1
        ldi temp1, 0x02
        out PORTG, temp1

        brne Jmp_Endif2
		Jmp_Endif2:
            rjmp Jmp_EndIF
        clear SecondCounter ; This takes 1 second only
        ; or its open and it is closing
		DoorClosing:
            andi streg, 0b1110111
            ldi temp1, 0xB5     ; door moving pattern
            out PORTC, temp1
            ldi temp1, 0x02
            out PORTG, temp1
            clear SecondCounter
            rjmp Jmp_EndIF

    CheckOpenDoor:
        ; check not moving and door open
        mov temp2, streg
		andi temp2, 0b00100000
		cpi temp2, 0b00100000
        brne CheckMoving
        ldi temp1, 0b00000011  
        out PORTC, temp1
        out PORTG, temp1
       
        sts SecondCounter, r24
        sts SecondCounter+1, r25
        cpi r24, 3;3s
        brne Jmp_EndIF
       
        ori streg, 0b00010000
        clear SecondCounter
        rjmp Jmp_EndIF

    CheckMoving:
        ; out the 0xFF pattern
        ser temp1
        out PORTC, temp1
        ldi temp1, 0x03
        out PORTG, temp1
        clr r24
        clr r25
        lds r24, FloorQueue
        lds r25, FloorQueue+1
        cpi r24, 0x00
        ldi temp1, 0x00
        cpc r25, temp1
        breq jmp_EndIF

        mov temp2, streg
		andi temp2, 0b00000100
		cpi temp2, 0b00000100
        brne jmp_Lift

        cpi r24, 2 
        sts SecondCounter, r24
        sts SecondCounter+1, r25
        brge Arrive
        lds r26, CurrentPattern   
        lds r27, CurrentPattern+1
        out PORTC, r26
        out PORTG, r27

	Jmp_EndIF:
        rjmp EndIF

jmp_Lift:
    jmp Lift


Arrive:
    push temp2
    push temp1
	push r28
    push r27
	push r26
    push r25
    push r24

    
    mov temp2, streg
    andi temp2, 0b00001000;direction up
    cpi temp2, 0b00001000
    brne DirDown
    lds r27, FloorNumber
    ldi r28, 1
    add r27, r28
    sts FloorNumber, r27
    rjmp AfterUpdateFloorNumber

    DirDown:
        lds r2, FloorNumber
		ldi r28, -1
	    add r27, r28
		sts FloorNumber, r27

    AfterUpdateFloorNumber:
		clr r24
		clr r25
		ldi r24, 0b00000001
		lds temp1, FloorNumber
		NotFloor:
			cpi temp1, 1
			brlt FloorFounded1
			dec temp1
			lsl r24
			rol r25
			rjmp NotFloor

		FloorFounded1: 
		sts FloorBits, r24
		sts FloorBits+1, r25

    mov temp2, streg
    andi temp2, 0b00000001
    cpi temp2, 0b00000001
    brne PrintFloorNumber
    rjmp PrintDown

    PrintFloorNumber:
        lcd_clrscreen      ; print FloorNumber to LCD
        lcd_floor
        print_floor

    PrintDown:
    clear SecondCounter
    andi streg, 0b11111011

    ; Turn off motor
    ldi temp1, 0b00000000
    sts OCR3BL, temp1
    clr temp1
    sts OCR3BH, temp1


	;compare FloorBits with FloorQueue
	lds r5, FloorQueue
    and r24, r5
    cpi r24, 0x00
    brne StartOpen
	rjmp FloorNoQueue

    StartOpen:
        ori streg, 0b00010000
        ldi temp1, 0xB5
        out PORTC, temp1
        ldi temp1, 0x02
        out PORTG, temp1
        clear SecondCounter

    FloorNoQueue:
	com r24
    com r25
    lds r26, FloorQueue
    lds r27, FloorQueue+1
    and r26, r24
    and r27, r25
    sts FloorQueue, r26
    sts FloorQueue+1, r27

    

    pop r24
    pop r25
	pop r26
    pop r27
    pop r28
    pop temp1
    pop temp2
    rjmp EndIF

Lift: 
    
    push temp2
	push r29
	push r28
    push r27
    push r26
    push r25
    push r24

    clr r29 
    clr r28 
    clear SecondCounter 
    clear TempCounter
    ori streg, 0b00000100 

    
    lds r26, FloorBits
    lds r27, FloorBits+1
    mov r6, r26 ; 
    mov r7, r27 ;

    out PORTC, r26 
    out PORTG, r27

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    mov r4, r24 
    mov r5, r25
	cp r26, r24  
    cpc r27, r25
    brge DirctionDown

    sbiw r27:r26, 1
    and r26, r4
    cpi r26, 0x00
    breq DirctionUp
    rjmp PreContinue

    DirctionDown:
        andi streg, 0b11110111
        lsr r6
        ror r7
        rjmp CurrentDirection

    DirctionUp:
        ori streg, 0b00001000
        lsl r6
        rol r7
        rjmp CurrentDirection

    PreContinue:
        mov temp2, streg
		andi temp2, 0b00001000
		cpi temp2, 0b00001000
        breq DirctionUp
        rjmp DirctionDown

    CurrentDirection:
        ldi temp2, 0b11111111
        sts OCR3BL, temp2

    sts CurrentPattern, r6
    sts CurrentPattern+1, r7

   
    pop r24
    pop r25
    pop r26
    pop r27
	pop r28
	pop r29
    pop temp2
    
    rjmp EndIF

EndIF:
    pop r24                         ; Epilogue starts
    pop r25                         ; Restore all conflict registers from the stack
    pop r26
    pop r27
    pop r28
    pop r29
    pop temp2
    pop temp1
    out SREG, temp1
    reti                            ; Return from the interrupt

EXT_INT0: ; pb0 set 
    push temp1
    in temp1, SREG
    push temp1
	mov temp2, streg
    andi temp2, 0b00100010
    cpi temp2, 0b00100010
	breq END_INT0
	ori streg, 0b00010010
    clear SecondCounter

END_INT0:
    pop temp1
    out SREG, temp1
    pop temp1
    reti




main:
    ldi cmask, INITCOLMASK   ; initial column mask
    clr col                  ; initial column

colloop:
    cpi col, 4
    breq main                ; If all keys are scanned, repeat.
    sts PORTL, cmask         ; otherwise scan a column

    ldi temp1, 0xFF          ; Slow down the scan operation
delay:
    dec temp1
    brne delay

    lds temp1, PINL          ; Read PORTL
    andi temp1, ROWMASK      ; Get the keypad output value
    cpi temp1, 0xF           ; Check if any row is low
    breq nextcol
                           ; If yes, find which row is low
    ldi rmask, INITROWMASK   ; Initialize for row check
    clr row

rowloop:
    cpi row, 4
    breq nextcol             ; the row scan is over
    mov temp2, temp1
    and temp2, rmask         ; check un-masked bit
    breq convert             ; if bit is clear, the key is pressed
    inc row
    lsl rmask
    jmp rowloop

nextcol:                   ; if row scan is over
    lsl cmask
    inc col                  ; increase the column value
    jmp colloop              ; go to the next column

convert:
    cpi col, 3               ; if the pressed key is in col 3
    breq main                ; we have  a letter - but we ignore non-digits for this part
                           ; if key is not in col 3 and
    cpi row, 3               ; if the key is in row3,
    breq symbols             ; we have a symbol or 0

    mov temp1, row           ; otherwise we have a number 1-9
    lsl temp1
    add temp1, row
    add temp1, col           ; temp1 = row*3 + col
    subi temp1, -1           ; Add 1 to the value since temp1 should start at 1 not 0
    jmp convert_end

symbols:
    cpi col, 0
    breq star
    cpi col, 1               ; Check if we have 0
    breq zero
    rjmp main                ; else ignore

zero:
    ldi temp1, 0             ; Set to zero
    jmp convert_end


star:
	mov temp2, streg
	andi temp2, 0b00000010
	cpi temp2, 0b00000010
    breq jmp_main
    ori streg, 0b00000010

    mov temp2, streg
	andi temp2, 0b00000001
	cpi temp2, 0b00000001 ; Lift should resume only when * is pressed again.
    breq emergency_end
    rjmp emergency_happen

jmp_main:
    jmp main

emergency_end:
    lcd_clrscreen; change LCD to the default message of "FLOOR: N"
    lcd_floor   ; where 'N' is the current floor
	push temp2
	push r5
    lds r5, FloorNumber
	mov temp2, r5
    subi temp2, -'0'
    
	pop r5
    pop temp2

    andi streg, 0b11111110
    clr strobepattern 
    out PORTB, strobepattern
    rjmp main


emergency_happen:
    lcd_clrscreen
    lcd_emergency
    ori streg, 0b00000001
	lds r24 ,FloorNumber
    cpi r24, 0
    brne LiftTo0
    ori streg, 0b00100000
    rjmp main

    LiftTo0:;make lift to floor0
    mov temp2, streg
    andi temp2, 0b00100000; check if lift door is open
    cpi temp2, 0b00100000
    brne StopOpening
    ori streg, 0b00010000

    ; close the door
    StopOpening:
        mov temp2, streg
		andi temp2, 0b00010000
		cpi temp2, 0b00010000
        brne Closeforce
        ori streg, 0b00100000

    Closeforce:
		clear SecondCounter
		ldi r24, 0b00000000
		sts FloorQueue, r24
		clr r24
		sts FloorQueue+1, r24
		andi streg, 0b11110111
		rjmp main

convert_end:
	rcall addtoQueue
    rjmp main

jmp_main1:
    jmp main

addtoQueue: ; Adds  floor to the FloorQueue
    push temp2
    push temp1
    push r25
    push r24

    clr r24
    clr r25
    ldi r24, 0b00000001

    FindFloor:
		cpi temp1, 1
		brlt FloorFounded
        dec temp1
        lsl r24
        rol r25
        rjmp FindFloor
	FloorFounded:
    mov temp1, r24
    mov temp2, r25
    lds r24, FloorQueue
    lds r25, FloorQueue+1
    or r24, temp1
    or r25, temp2
	sts FloorQueue, r24
    sts FloorQueue+1, r25

	FloorFoundded:
    pop r24
    pop r25
    pop temp1
    pop temp2
    ret

end: rjmp end

lcd_command:
  push temp1
  out PORTF, temp1
  rcall sleep_1ms
  lcd_set LCD_E
  rcall sleep_1ms
  lcd_clr LCD_E
  rcall sleep_1ms
  pop temp1
  ret

lcd_data:
    push temp1
    out PORTF, temp1
    lcd_set LCD_RS
    rcall sleep_1ms
    lcd_set LCD_E
    rcall sleep_1ms
    lcd_clr LCD_E
    rcall sleep_1ms
    lcd_clr LCD_RS
    pop temp1
    ret

lcd_wait:
    push temp1
    clr temp1
    out DDRF, temp1
    out PORTF, temp1
    lcd_set LCD_RW
lcd_wait_loop:
    rcall sleep_1ms
    lcd_set LCD_E
    rcall sleep_1ms
    in temp1, PINF
    lcd_clr LCD_E
    sbrc temp1, 7
    rjmp lcd_wait_loop
    lcd_clr LCD_RW
    ser temp1
    out DDRF, temp1
    pop temp1
    ret

sleep_1ms:
    push r24
    push r25
    ldi r25, high(DELAY_1MS)
    ldi r24, low(DELAY_1MS)
delayloop_1ms:
    sbiw r25:r24, 1
    brne delayloop_1ms
    pop r25
    pop r24
    ret

sleep_5ms:
    rcall sleep_1ms
    rcall sleep_1ms
    rcall sleep_1ms
    rcall sleep_1ms
    rcall sleep_1ms
    ret




