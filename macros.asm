.equ LCD_RS = 7
.equ LCD_E  = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

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

.macro do_lcd_data_reg
	 mov temp1, @0
    rcall lcd_data
    rcall lcd_wait
.endmacro

.macro lcd_setup
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
    do_lcd_data 'l'
    do_lcd_data 'o'
    do_lcd_data 'o'
    do_lcd_data 'r'
    do_lcd_data ':'
    do_lcd_data ' '
.endmacro


.macro lcd_emergency
    do_lcd_data ' '
    do_lcd_data ' '
	do_lcd_data '"'
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
	do_lcd_data '"'
.endmacro

.macro clear
    ldi YL, low(@0)        ; load the memory address to Y
    ldi YH, high(@0)
    clr temp1
    st Y+, temp1           ; clear the two bytes at @0 in SRAM
    st Y, temp1
.endmacro

.macro TransferNumberToBits
    push temp1
	DuringTransfer:
    cpi temp1, 1
    brlt FinishTranfer
    dec temp1
    lsl @0
    rol @1
    rjmp DuringTransfer
	FinishTranfer:
    pop temp1
.endmacro

.macro Print_Number
    push temp2
    mov temp2, @0
    subi temp2, -'0'
    do_lcd_data_reg temp2
    pop temp2
.endmacro

.macro PrintFloorNumber
    lds r3, FloorNumber
    Print_Number r3
.endmacro

.macro New_FloorNumber
    lds r1, FloorNumber
    ldi r31, @0
    add r1, r31
    sts FloorNumber, r1
.endmacro


.macro StatusOn								;We use ori and andi bitwise operators to set the statu register(r23)
    ori Streg, @0
.endmacro

.macro StatusOff
    andi Streg, @0
.endmacro

.macro CompareStatus				;alway compare with statu on to make it clear
    mov temp2, Streg
    andi temp2, @0
    cpi temp2, @0
.endmacro