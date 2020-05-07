; SimpleRobotProgram.asm
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.


ORG        &H000       ;Begin program at x000
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
	
Menu: OUT	RESETPOS
Me:	CALL	Wait1
	LOAD	Zero
	STORE	Temp
	IN     	XIO; XIO contains KEYs
	AND    	MaskT
	SUB 	Six
	JZERO	Circle; If Key1 is pressed
	ADDI	1
	JZERO   Polygon; If Key2 is pressed
	ADDI 	2
	JZERO	Scribbler; If Key3 is pressed
	JUMP	Me
	
;***************************************************************
;* Main code
;***************************************************************
Circle:	
		LOAD 		FMid
		OUT 		RVELCMD
		LOAD 		FSlow
		OUT 		LVELCMD
		IN			LPOS
		SUB 		Spin
		JNEG 		Circle	
		LOAD 		Zero
		OUT 		RVELCMD
		LOAD		Zero
		OUT 		LVELCMD
		CALL		Wait1
		OUT			RESETPOS
CircleTwo:	
		LOAD 		FSlow
		OUT 		RVELCMD
		LOAD 		FMid
		OUT 		LVELCMD
		IN			RPOS
		SUB 		Spin
		JNEG 		CircleTwo
		LOAD Zero
		OUT RVELCMD
		LOAD Zero
		OUT LVELCMD
		JUMP		Menu
		
Scribbler:
		LOAD		Circum; Loads circumference of half meter circle in 1.05 mm units
		STORE 		Length
		LOAD		Hundred
		STORE		NEW_Y
		IN	 		SWITCHES
		STORE 		Temp
		LOAD 		Temp; Saves Switch values
		AND 		Mask0
		JPOS 		Triangle; If first switch is active, draw triangle
		LOAD 		Temp
		AND 		Mask1
		JPOS 		Square; If second switch is active, draw square
		LOAD 		Temp
		AND 		Mask2
		JPOS 		Pentagon; If third switch is active, draw pentagon
		LOAD 		Temp
		AND 		Mask3
		JPOS 		Hexagon; If fourth switch is active, draw hexagon
		LOAD		Bad
		OUT			SSEG1
		JUMP		Scribbler; If none of the switches are active, return to start loop
Triangle:	
		LOAD		Length
		DIV			Three
		STORE 		Length
		LOAD		Three
		STORE		Sides
		LOAD		Zero
		SUB			HexX
		STORE 		NEW_X
		CALL		arctan
		LOAD	    RATIO
		OUT			SSEG2
		CALL		Side
		JUMP		Menu
Square:	
		LOAD		Length
		DIV			Four
		STORE 		Length
		LOAD		Four
		STORE		Sides
		LOAD 		Zero
		STORE 		NEW_X
		CALL		arctan
		LOAD	  	RATIO
		OUT			SSEG2
		CALL		Side
		JUMP		Menu
Pentagon:	
		LOAD		Length
		DIV			Five
		STORE 		Length
		LOAD		Five
		STORE		Sides
		LOAD 		PenX
		STORE 		NEW_X
		CALL		arctan
		LOAD	  	RATIO
		OUT			SSEG2
		CALL 		Side
		JUMP		Menu
Hexagon:
		LOAD		Length
		DIV			Six
		STORE 		Length
		LOAD		Six
		STORE		Sides
		LOAD		HexX
		STORE 		NEW_X
		CALL		arctan
		LOAD	  	RATIO
		OUT			SSEG2
		CALL 		Side
		JUMP		Menu
		
Polygon:
		IN	 		SWITCHES; load values from switches
		AND 		MaskS; Use a Mask so only the first four switches are read
		ADD 		Three; Add three, so smallest is triangle (largest is 18-gon)
		STORE 		Sides; Store number of sides
		LOAD		Circum; Loads circumference of half meter circle in 1.05 mm units
		DIV 		Sides; Divide by number of sides
		STORE 		Length; Store as length of each side
		LOAD 		Sides; Loads number of sides
		OUT			SSEG1
		SUB 		Two; Sum of interior angles of a polygon is 180*(n-2)
		MULT 		Deg180; Multiply by 180
		DIV 		Sides; Divide sum of interior angles by # of sides
		STORE		RATIO; Store value of interior angle
		LOAD		Deg180;
		SUB 		RATIO; Subtract interior angle by 180 to get exterior angle
		OUT			SSEG2
		STORE 		RATIO; Store exterior angle of each side
		CALL 		Side
		JUMP 		Menu

	;Figure out if were are in 0-45 or 45-90
arctan:		
			LOAD NEW_X
			JNEG Left
Right: 		LOAD NEW_Y
			JNEG BottomRight
			JUMP TopRight
Left: 		LOAD NEW_Y
			JNEG BottomLeft
			JUMP TopLeft
TopRight:	LOADI 1
			STORE QUAD
			LOAD NEW_Y
			SUB NEW_X
			JZERO FortyFive
			JNEG Regular
			LOADI 1
			STORE INVERSE
			JUMP InvRegular
BottomRight: LOADI 4
			STORE QUAD
			LOAD NEW_Y
 			ADD NEW_X
 			JZERO FortyFive
			JNEG InvRegular
			LOADI 1
			STORE INVERSE
 			JUMP Regular
TopLeft:	LOADI 2
			STORE QUAD
			LOAD NEW_Y
			ADD NEW_X
			JZERO FortyFive
			JPOS InvRegular
			LOADI 1
			STORE INVERSE
 			JUMP Regular
BottomLeft: LOADI 3
			STORE QUAD
			LOAD NEW_Y
			SUB NEW_X
			JZERO FortyFive
			JPOS Regular
			LOADI 1
			STORE INVERSE
			JUMP InvRegular
Regular:	LOAD NEW_Y
			JNEG InvYR
			LOAD NEW_X
			JNEG InvXR
			LOAD NEW_Y
			SHIFT 6
			DIV NEW_X
			STORE RATIO
			LOAD INVERSE
			JPOS InvLookup
			JUMP Lookup
InvXR:		LOAD NEW_X
			XOR AllOne
			ADDI 1
			STORE NEW_X
			JUMP Regular
InvYR:		LOAD NEW_Y
			XOR AllOne
			ADDI 1
			STORE NEW_Y
			JUMP Regular
InvRegular: LOAD NEW_Y
			JNEG InvYI
			LOAD NEW_X
			JNEG InvXI
			LOAD NEW_X
			SHIFT 6
			DIV NEW_Y
			STORE RATIO
			LOAD INVERSE
			JPOS InvLookup
			JUMP Lookup
InvXI:		LOAD NEW_X
			XOR AllOne
			ADDI 1
			STORE NEW_X
			JUMP InvRegular
InvYI:		LOAD NEW_Y
			XOR AllOne
			ADDI 1
			STORE NEW_Y
			JUMP InvRegular
Lookup: 	LOAD RATIO
			ADD LUT
			STORE RATIO
			ILOAD RATIO
			STORE RATIO
			JUMP Adjust
			JUMP Lookup
InvLookup:	LOAD RATIO
			ADD LUT
			STORE RATIO
			ILOAD RATIO
			STORE RATIO 
			LOADI 90
			SUB RATIO
			STORE RATIO
			JUMP Adjust
Adjust:		LOAD QUAD
			ADDI -1
			JZERO First
			ADDI -1 
			JZERO Second
			ADDI -1
			JZERO Third
			ADDI -1
			JZERO Fourth
First:		RETURN
Second:		LOADI 90
			ADD RATIO
			STORE RATIO
			RETURN
Third:		LOAD RATIO
			ADDI 90
			ADDI 90
			STORE RATIO
			RETURN
Fourth:		LOADI 270
			ADD RATIO
			STORE RATIO
			RETURN
FortyFive:  LOADI 45
			STORE RATIO
			JUMP Adjust

;***************************************************************
;* Subroutines
;***************************************************************
Side:	
		LOAD		Sides
		OUT			LCD
		LOAD		Zero
		OUT			RESETPOS
		LOAD		Zero
		OUT			THETA
		CALL 		Turn
		CALL		Wait1
		CALL 		Move
		LOAD 		Sides
		SUB 		One
		OUT			LCD
		STORE 		Sides
		LOAD 		Sides
		JPOS 		Side
		RETURN

Turn:		LOAD RATIO
			OUT SSEG2
			SUB Deg180
			JPOS TurnRight
TurnLeft:	
			LOAD FSlow
			OUT RVELCMD
			LOAD RSlow
			OUT LVELCMD
			IN 		THETA
			ADDI	-359
			OUT		THETA
			JZERO 	TurnLeft
			IN THETA
			OUT SSEG1
			SUB RATIO
			JNEG TurnLeft
			LOAD Zero
			OUT RVELCMD
			LOAD Zero
			OUT LVELCMD
			RETURN
TurnRight:	LOAD RSlow
			OUT RVELCMD
			LOAD FSlow
			OUT LVELCMD
			IN THETA
			OUT SSEG1
			SUB RATIO
			JPOS TurnRight
			LOAD Zero
			OUT RVELCMD
			LOAD Zero
			OUT LVELCMD
End:		RETURN

Move:	LOAD 		FSlow
		OUT 		RVELCMD
		LOAD 		FSlow
		OUT 		LVELCMD
		IN 			LPOS
		SUB 		Length
		JNEG 		Move
		LOAD 		Zero
		OUT 		RVELCMD
		LOAD 		Zero
		OUT 		LVELCMD
		RETURN

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; Subroutine to send AC value through the UART,
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN
;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful
Sides:     DW 0 ; Used to store the number of sides
Length:     DW 0 ; Used to store the length of each side

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
; Masks
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10
Bad:	  DW 2989
Circum:	  DW 3022
Hundred: DW 100
PenX:     DW 32
HexX:     DW 58
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
MaskS:    DW &B00001111
MaskT:    DW &B00000111
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111
AllOne:   DW &HFFFF
LUT:	  DW &H200
Spin:     DW &H201
; some useful movement values
OneMeter: DW 961       ; ~1m in 1.05mm units
HalfMeter: DW 481      ; ~0.5m in 1.05mm units
TwoFeet:  DW 586       ; ~2ft in 1.05mm units
Deg45:	  DW 45
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0

NEW_X: DW 3
NEW_Y: DW 1
RATIO: DW 0
QUAD: DW 0
INVERSE: DW 0
ORG        &H200
LOOKUP_TABLE:  DW 0
DW 1
DW 2
DW 3
DW 4
DW 4
DW 5
DW 6
DW 7
DW 8
DW 9
DW 10
DW 11
DW 11
DW 12
DW 13
DW 14
DW 15
DW 16
DW 17
DW 17
DW 18
DW 19
DW 20
DW 21
DW 21
DW 22
DW 23
DW 24
DW 24
DW 25
DW 26
DW 27
DW 27
DW 28
DW 29
DW 29
DW 30
DW 31
DW 31
DW 32
DW 33
DW 33
DW 34
DW 35
DW 35
DW 36
DW 36
DW 37
DW 37
DW 38
DW 39
DW 39
DW 40
DW 40
DW 41
DW 41
DW 42
DW 42
DW 43
DW 43
DW 44
DW 44
DW 45

