;COMP2121 project

.include "m2560def.inc"
.include "macros.asm"			

.equ Lift_movingup   = 0xAAAA
.equ Lift_movingdown = 0x5555
.equ Motor_off   = 0x00
.equ Motor_on    = 0xFF

.def strobelight = r5	   ;using for strobe light flash 
.def row   = r16           ; current row number
.def col   = r17           ; current column number
.def rmask = r18           ; mask for current row during scan
.def cmask = r19           ; mask for current column during scan
.def temp1 = r20
.def temp2 = r21
.def Secondreg = r22		;this is register to save second counter number
.def Streg = r23			;this is register to present the lift statu
.undef ZL
.undef ZH

.equ PORTLDIR    = 0xF0    ; PL7-4: output, PL3-0:input
.equ INITCOLMASK = 0xEF    ; scan from the rightmost column
.equ INITROWMASK = 0x01    ; scan from the top row
.equ ROWMASK     = 0x0F    ; for obtaining input form Port D

;//Status situation
.equ emergencystatu_on = 0b00000001			;there are some status for elevator can be include in these part: for bit0 is emergency on or not, 
.equ liftmoving_on = 0b00000010				;bit1 is if lift moving or not, bit2 is if door open or closed. bit3 is moving direction.bit4 is
.equ door_open = 0b00000100					;check the statu if debounce. bit5 is check if door is moving or not
.equ moving_up = 0b00001000
.equ debounce_counter_on = 0b00010000
.equ doormoving_on = 0b00100000

.equ emergencystatu_off = 0b11111110		;ori can set the statu on,  using top statu value and andi can set the statu off, using bot value
.equ liftmoving_off = 0b11111101
.equ door_close = 0b11111011
.equ moving_down = 0b11110111
.equ debounce_counter_off = 0b11101111		//When we want to reset the state, ANDI the r23 with its counterparts defined here
.equ doormoving_off = 0b11011111


.dseg
StrobelightCounter: .byte 2
DebounceCounter: .byte 2
TempCounter: .byte 2
SecondCounter: .byte 1		 ;using r23 to store the second counter data
CurrentPattern: .byte 2      ;Pattern for the LEDs to represent lift going up or down
FloorNumber: .byte 1         ;Floor Number
FloorBits: .byte 2			 ;transfer floor number to bit like 00000110 for floor number is 5(6-1) but fot bit is 00100000
FloorQueue: .byte 2			 ;Using to save floor in queue


.cseg
.org 0x0000
    jmp RESET
.org INT0addr
    jmp EXT_INT0
.org INT1addr
    jmp EXT_INT1

	jmp DEFAULT
	jmp DEFAULT                      
    
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

    ;LED and strobelight input/output set up
    ser temp1               
    out DDRC, temp1         ;PORTC is output(LED)
    out PORTC, temp1		 
	out DDRG, temp1         ;PORTG is output(LED)
	out PORTG, temp1          
	out DDRF, temp1			;PORTF is output(LCD)
    out DDRA, temp1			;PORTA is output(LCD)
    out DDRB, temp1			;PORTB is output(Srobelight) //wire to port B for 
    
	clr temp1
    out PORTB, temp1        ;connect srobelight to portB as output for emergency
	out PORTF, temp1		;LCD is clr
    out PORTA, temp1		

    ; door is closed, 
    ldi temp1, 0xFF
    out PORTC, temp1
    out PORTG, temp1

	;clear counter
	clear DebounceCounter
	clear TempCounter
	clear SecondCounter
	clear FloorNumber
	clear FloorBits
	clear FloorQueue 
	clr Streg

	;set floorbit for ground floor firstly
    ldi r24, 0b00000001
    sts FloorBits, r24

    ;after reset the current pattern should be moving up statu at first
    lds r24, CurrentPattern
    lds r25, CurrentPattern+1
    ldi  r24, low(Lift_movingup)		
    sts CurrentPattern, r24
    ldi  r25, high(Lift_movingup)
    sts CurrentPattern+1, r25

  ; Motor set up
    ser temp1
    out DDRE, temp1     ;PORTE(pe2) is output(Motor)

    clr temp1
    sts OCR3BL, temp1
    sts OCR3BH, temp1

	ldi temp1, (1<<CS30)
    sts TCCR3B, temp1
    ldi temp1, (1<<WGM30)|(1<<COM3B1)
    sts TCCR3A, temp1

	; Timer0 Set up
    ldi temp1, 0b00000000
    out TCCR0A, temp1
    ldi temp1, 0b00000010
    out TCCR0B, temp1                 ; Prescaling value = 8
    ldi temp1, 1<<TOIE0               ; = 128 microseconds
    sts TIMSK0, temp1                 ; T/C0 interrupt enable
	
    ; close botton Setup
    ldi temp1, (2 << ISC00)              ; set INT0 as falling-
    sts EICRA, temp1                     ; edge triggered interrupt
    in temp1, EIMSK                      ; enable INT0
    ori temp1, (1<<INT0)
    out EIMSK, temp1

    ; open botton Setup
    ldi temp1, (0 << ISC10)              ; set INT1 as low-level interrupt
    sts EICRA, temp1                     ; triggered interrupt
    in temp1, EIMSK                      ; enable INT1
    ori temp1, (1<<INT1)
    out EIMSK, temp1

	;lcd screen setup							
    lcd_setup							; set floor to 0 in lcd
    lcd_floor							//this marco prints FLOOR on the LCD
    do_lcd_data '0'

	sei                               ; Enable global interrupt
	rjmp main


Timer0OVF:
    in temp1, SREG					; For r24, r25 use fot temporary situation
    push temp1                      ; For r26, 27, use to store or load Floorqueue data,
    push temp2						; For r28,r29, basicly use to store or load FloorBits data, For r30, r31 use CurrentPatten
	push r31
	push r30
    push r29
    push r28
    push r27
    push r26
    push r25
    push r24                        
    
	;We need build 3 counter: Debouce counter for deboucing, strobelight counter to make strobelight flash, and second counter to do control lift part
	
	;1. Debouce counter                         
    DebouceCountermake:
        CompareStatus debounce_counter_on
        breq StartDebouncing
        rjmp StrobelightCountermake

    StartDebouncing:
		lds r24, DebounceCounter
        lds r25, DebounceCounter+1
        adiw r25:r24, 1

        ldi temp1, high(2500)				;2500/7812 = 0.32s  
        cpi r24, low(2500)
        cpc r25, temp1
        breq StopDebounce
        jmp KeepDebouncing

	KeepDebouncing:
		sts DebounceCounter, r24
		sts DebounceCounter+1, r25
		rjmp EndIF
		
    StopDebounce:
		clear DebounceCounter
        StatusOff debounce_counter_off

	;2. StrobelightCounter 
	StrobelightCountermake:
		CompareStatus emergencystatu_on
        brne SecondCountermake
		clr r24
		clr r25
        lds r24, StrobelightCounter
        lds r25, StrobelightCounter+1
        adiw r24:r25, 1
       
        ldi temp1, high(1000)				;1000/7812 = 0.128s flash once
        cpi r24, low(1000)
        cpc r25, temp1
        brne Keepcounting
        clear StrobelightCounter
        com strobelight
        out PORTB, strobelight
        rjmp SecondCountermake

    Keepcounting:
        sts StrobelightCounter, r24
        sts StrobelightCounter+1, r25


	;3. SecondCounter
    SecondCountermake:
        lds r24, TempCounter
        lds r25, TempCounter+1
        adiw r25:r24, 1
        
        ldi temp1, high(7812)
        cpi r24, low(7812)
        cpc r25, temp1
        breq MakeaSecond
        rjmp NotSecond

    MakeaSecond:
        clear TempCounter
        lds Secondreg, SecondCounter
		subi Secondreg, -1

;The next steps is to about 2 part:
;1.Check the statu of the door What is the statu of the door:
;	a.if the door is moving or not
;	b.if the door is open or not	
;2.lift moving part
;	a.set the direction of lift if its moving up or down
;	b.check floor queue:
;		1)if in floor queue do open/close door processing 
;		2)if not keepchecking
;		3)if finish this floor, out the floor to floor queue


	;1. Door part	
	CheckingDoor:
		CompareStatus doormoving_on					;Check if door is moving or not
        brne OpenOrClose							;If it is not moving, go to check if door state is open or close
        CompareStatus door_open						;Now door is moving, check if moving open or moving close
        breq DoorisClosing							;Go to moving close


		;a, door opengin or closing process
		DoorisOpening:											;door is opening  part.
			ldi temp1, 0x31										;door opening or closing prosses can be (1000110001)
			out PORTC, temp1
			ldi temp1, 0x02
			out PORTG, temp1
												
			ldi temp2, Motor_on									;turn on the motor
			sts OCR3BL, temp2

			cpi Secondreg, 1									;compare with secondcounter make opening door 1s.
			sts SecondCounter, Secondreg
			brne To_ENDIF0
			clear SecondCounter 
			StatusOff doormoving_off 						;aftering opened, set door to not mov and opened state
			StatusOn door_open
			To_ENDIF0:
				rjmp TO_ENDIF1

        DoorisClosing:											;almost same with opening part diffirent is set door to close state
            ldi temp1, 0x31     
            out PORTC, temp1
            ldi temp1, 0x02
			out PORTG, temp1

			ldi temp2, Motor_on
			sts OCR3BL, temp2

			clear SecondCounter
			StatusOff doormoving_off
            StatusOff door_close
            rjmp To_ENDIF1


		;b, the statu of door open or close
		OpenOrClose:		
		DoorisOpen:
			CompareStatus door_open								;check if door is open
			brne DoorisClose									;if not open go to close
			ldi temp1, 0x01										;state of door open (1000000001)
			out PORTC, temp1
			ldi temp1, 0x02
			out PORTG, temp1
			ldi temp2, Motor_off								;turn off the motor
			sts OCR3BL, temp2
			sts SecondCounter, Secondreg						;open 3 second
			
			cpi Secondreg, 3
			brne To_ENDIF1
			StatusOn doormoving_on								;set door to moving state
			clear SecondCounter
			rjmp To_ENDIF1

		DoorisClose	:
			ser temp1											;state of door close is(1111111111)
			out PORTC, temp1
			ldi temp1, 0x03
			out PORTG, temp1
			ldi temp2, Motor_off								;turn off the motor
			sts OCR3BL, temp2


	;2 Lift moving part
	Startliftmoving:

		lds r26, FloorQueue										;firstly check if need to move, if no number in floor queue dont need move
		lds r27, FloorQueue+1
		ldi temp1, 0b00000000								 ;compare both high and low with zero
		cp r26, temp1
		cpc r27, temp1
		breq TO_ENDIF1
		CompareStatus liftmoving_on	                          ;check if lift moving
		brne Directionset									  ;if not moving now, set direction
		cpi Secondreg, 2									  ;if lift is moving now we need make it travel 2s between each floor compare second counter with 2	
		sts SecondCounter, Secondreg
		breq FindFloor										  
		lds r30 , CurrentPattern							  ;If it is less than 2 second it means in moving process between two floor, we ne
		lds r31 , CurrentPattern+1
		out PORTC, r30
		out PORTG, r31
	
		To_ENDIF1:
			rjmp ENDIF

		Directionset:
		StatusOn liftmoving_on				;set lift to moving statu

		lds r28, FloorBits
		lds r29, FloorBits+1
		cp r26, r28							;if Floorqueue < floor bits should going down
		cpc r27, r29
		brlt DirectionDown
		sbiw r29:r28, 1
		and r28, r26
		cpi r28, 0x00
		breq DirectionUp
		rjmp KeepChecking

		DirectionDown:
			StatusOff moving_down
			New_FloorNumber -1 
			ldi  r30, low(lift_movingdown)
			ldi  r31, high(lift_movingdown)
			rjmp ShowinPattern

		DirectionUp:
			StatusOn moving_up
			New_FloorNumber 1
			ldi  r30, low(lift_movingup)
			ldi  r31, high(lift_movingup)
			rjmp ShowinPattern

		KeepChecking:
			CompareStatus moving_up
			breq DirectionUp
			rjmp DirectionDown

		ShowinPattern:
			sts CurrentPattern, r30
			sts CurrentPattern+1, r31
			rjmp EndIF

	FindFloor:
		clr r26
		clr r27
		ldi r26, 0b00000001
		lds temp1, FloorNumber
		TransferNumberToBits r26, r27
		sts FloorBits, r26
		sts FloorBits+1, r27
		CompareStatus emergencystatu_on			;we need find if emergency happen when lift moving
		brne PrintFloorNumberFunc
		rjmp Keepfindfloor

		PrintFloorNumberFunc:
			lcd_setup; print FloorNumber to LCD
			lcd_floor
			PrintFloorNumber

		Keepfindfloor:
			clear SecondCounter
			StatusOff liftmoving_off
			lds r4, FloorQueue
			lds r5, FloorQueue+1
			and r26, r4
			cpi r26, 0x00
			brne Openthedoor
			and r27, r5
			cpi r27, 0x00
			brne Openthedoor
			rjmp NotInQueue

		Openthedoor:
			StatusOn doormoving_on
		NotInQueue:					;When we reach the floor, delete the floor from floor queue
			com r26
			com r27
			lds r24, FloorQueue
			lds r25, FloorQueue+1
			and r24, r26
			and r25, r27
			sts FloorQueue, r24
			sts FloorQueue+1, r25
			rjmp EndIF

	NotSecond:
		sts TempCounter, r24
		sts TempCounter+1, r25
		rjmp EndIF

EndIF:
    pop r24                         ; Epilogue starts
    pop r25                         ; Restore all conflict registers from the stack
    pop r26
    pop r27
    pop r28
    pop r29
	pop r30
	pop r31
    pop temp2
    pop temp1
    out SREG, temp1
    reti                            ; Return from the interrupt

; Close button
EXT_INT0: 
    in temp1, SREG						;for close button, we need check 2 part :if debouce counter on, the door is open or not
    push temp1							;
	push temp2
	CompareStatus door_open
    brne END_INT0
	CompareStatus debounce_counter_on   
    breq END_INT0

	StatusOn debounce_counter_on
    StatusOn door_open
	StatusOn doormoving_on
    clear SecondCounter

END_INT0:
	pop temp2
    pop temp1
    out SREG, temp1
    reti

EXT_INT1: ; "Open" button				for close button, we need check 3 part :if debouce counter on, the door is open or not
    in temp1, SREG						; and if the lift is moving or not, if moving can not open.
    push temp1
	push temp2

	CompareStatus liftmoving_on
    breq END_INT1
    CompareStatus door_open
    breq END_INT1
	CompareStatus debounce_counter_on    
    breq END_INT1
    StatusOn debounce_counter_on
    StatusOn doormoving_on
    StatusOff door_close

END_INT1:
	pop temp2
    pop temp1
    out SREG, temp1
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
    CompareStatus debounce_counter_on
    breq jmp_main0
    StatusOn debounce_counter_on
    CompareStatus emergencystatu_on 
    breq AfterEmergency
    rjmp DuringEmergency

jmp_main0:
    jmp main

;Emergency Part
AfterEmergency:
	lcd_setup 
    lcd_floor  
    PrintFloorNumber
	clr strobelight
    out PORTB, strobelight
	StatusOff emergencystatu_off 
    rjmp main

DuringEmergency:
	lcd_setup
    lcd_emergency
	StatusOn emergencystatu_on // enable emergency
    lds r31, FloorNumber 
    cpi r31, 0
    brne StartEmergency
    StatusOn door_open
    rjmp main

    StartEmergency:
		CompareStatus door_open		;check if door is opened or not
		brne CloseTheDoor
		StatusOn doormoving_on

	CloseTheDoor:
        CompareStatus doormoving_on ;check if door is moving or not
        brne ToGround
        StatusOn door_open

    ToGround:
		clear SecondCounter				;set the floor queue to 0 and clr all floor in queue
		ldi r24, 0b00000001
		sts FloorQueue, r24
		ldi r24, 0b00000000
		sts FloorQueue+1, r24
		StatusOff moving_down
		rjmp main

convert_end:
    CompareStatus emergencystatu_on
    breq jmp_main
    CompareStatus debounce_counter_on
    breq jmp_main
    StatusOn debounce_counter_on
    lds r1, FloorNumber
	cp r1, temp1				//if different add to queue
    breq jmp_main 
    rcall AddToQueue
   
jmp_main:
    jmp main

AddToQueue:
	push r26
	push r27
	push r28
	push r29

	clr r26
	clr r27
	clr r28
	clr r29
    lds r26, FloorQueue
    lds r27, FloorQueue+1

    ldi r28, 0b00000001			//start the bit at the 0th position	then keep left shifting until we find the right position
    TransferNumberToBits r28, r29
	or r26, r28
    or r27, r29
	sts FloorQueue, r26
    sts FloorQueue+1, r27

    pop r29
    pop r28
	pop r27
	pop r26

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