;*****************************************************************
;* COE 538 - EEBOT Project - Version 2, Date: 29 Nov 2023        *
;* Aaron Gaba (501062030), Keedon Cokely (501124817),            * 
;* Mohammed Furqaan Shaikh (501034654)                           *
;*                                                               *.                                                 
;* This file contains all the guidance code for the EEBOT.       *
;*****************************************************************

;* The thresholds and timing in this program are for EEBOT #102932 *
;* The reverse-turn state has the EEBOT perform a 180-degree turn. *
;* The forward-turn state has the EEBOT perform a forward left 90-degree turn.*

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

;****************************************************************
; Displaying battery voltage and bumper states (s19c32) *
;****************************************************************
        
; Definitions
LCD_DAT    EQU     PORTB   ; LCD data port, bits - PB7,...,PB0
LCD_CNTR   EQU     PTJ     ; LCD control port, bits - PE7(RS),PE4(E)
LCD_E      EQU     $80     ; LCD E-signal pin
LCD_RS     EQU     $40     ; LCD RS-signal pin

FLW_INT       EQU     3    ; 130ms interval (at 23Hz)
REV_INT       EQU     18   ; 1 second interval
FWD_TRN_INT   EQU     34   ; 1.478s interval
REV_TRN_INT   EQU     36   ; 1.565s interval

;Definitions of various states
START         EQU     0
FLW           EQU     1    ; Follow State
REV           EQU     2
ALL_STP       EQU     3
ADJUST        EQU     4    ; Adjust State
REV_TRN       EQU     5
FWD_TRN       EQU     6

; Variable/data section
           ORG $3850

TOF_COUNTER dc.b 0
CRNT_STATE  dc.b 3          ; Current State
T_FLW       ds.b 1
T_REV       ds.b 1
T_FWD_TRN   ds.b 1
T_REV_TRN   ds.b 1
           
TEN_THOUS  RMB     1       ; 10,000 digit
THOUSANDS  RMB     1       ; 1,000 digit
HUNDREDS   RMB     1       ; 100 digit
TENS       RMB     1       ; 10 digit
UNITS      RMB     1       ; 1 digit
NO_BLANK   RMB     1       ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE  RMB     2       ; Extra space for decimal point and string terminator
ADDATA     RMB     8       ; Storage for A/D converter results

SENSOR_LINE     FCB $01              ; Storage for guider sensor readings
SENSOR_BOW      FCB $23              ; Initialized to test values
SENSOR_PORT     FCB $45
SENSOR_MID      FCB $67
SENSOR_STBD     FCB $89              ; Starboard (STBD) Sensor

SENSOR_NUM      RMB 1                ; The currently selected sensor

SENSOR_STATE    RMB 1                ; Byte holding binary state of each sensor
                                     ; b4 = BOW, b3 = PORT, b2 = MID, b1 = STBD, b0 = LINE
                                     
MOTOR_PWR_STATE FCB %00011000        ; Byte holding motor power configuration                                    
                                     ; Default Value: Port Motor ON, Starboard Motor ON
                                     
TEMP            RMB 1                ; Temporary location


;*****************************************************************
;* Code Section                                                  *
;*****************************************************************
            ORG   $4000

Entry:
_Startup:
            CLI
            LDS   #$4000            ; Initialize Stack Pointer
            
            
            BSET  DDRA, %00000011   ; STAR_DIR, PORT_DIR
            BSET  DDRT, %00110000   ; STAR_SPEED, PORT_SPEED
            
           
            JSR   initAD            ; Initialize ATD converter
            JSR   initLCD           ; Initialize LCD
            JSR   clrLCD            ; Clear LCD & Home Cursor
            
            LDX   #msg1             ; Display Msg1
            JSR   putsLCD
            
            LDAA  #$C0              ; Move LCD cursor to the 2nd row
            JSR   cmd2LCD
            
            LDX   #msg2             ; Display Msg2
            JSR   putsLCD
            
            JSR   ENABLE_TOF        ; Jump to TOF initialization
            
            JSR INIT             ; Initialize ports
            JSR openADC          ; Initialize the ATD
                        
            
MAIN:       JSR   UPDT_DISPL
            LDAA  CRNT_STATE
            JSR   DISPATCHER
            BRA   MAIN

;---------------------------------------------------------------------------
;               Initialize ports

INIT            BCLR DDRAD,$FF       ; Make PORTAD an input (DDRAD @ $0272)
                BSET DDRA,$FF        ; Make PORTA an output (DDRA @ $0002)
                BSET DDRB,$FF        ; Make PORTB an output (DDRB @ $0003)
                BSET DDRJ,$C0        ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A) 
                RTS

;---------------------------------------------------------------------------
;               Initialize the ADC

openADC         MOVB #$80,ATDCTL2    ; Turn on ADC (ATDCTL2 @ $0082)
                LDY  #1              ; Wait for 50 us for ADC to be ready
                JSR  del_50us        ;    - " -
                MOVB #$20,ATDCTL3    ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                MOVB #$97,ATDCTL4    ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                RTS


;*****************************************************************
;* Data Section                                                  *
;*****************************************************************

msg1        dc.b "Battery volt ", 0
msg2        dc.b "State ", 0
tab         dc.b "START  ", 0
            dc.b "FOLLOW ", 0
            dc.b "REV    ", 0
            dc.b "ALL_STP", 0
            dc.b "ADJUST ", 0
            dc.b "REV_TRN", 0
            dc.b "FWD_TRN", 0

;*****************************************************************
;* State Dispatcher                                              *
;*****************************************************************  
;This routine calls the appropriate state handler based on the
;current state.

DISPATCHER:        CMPA #START           ;If it's the START state               
                   BNE  NOT_START        ;then call the START routine
                   JSR  START_ST         ;and exit
                   JMP  DISP_EXIT

NOT_START:         CMPA #FLW             ;Else if it's the FOLLOW (LINE) state
                   BNE  NOT_FORWARD      ;Then call the FOLLOW routine 
                   JSR  FLW_ST           ;and exit
                   JMP  DISP_EXIT

NOT_FORWARD:       CMPA #REV             ;Else if it's the REVERSE state
                   BNE  NOT_REVERSE      ;Then call the REVERSE routine
                   JSR  REV_ST           ;and exit
                   JMP  DISP_EXIT

NOT_REVERSE:       CMPA #ALL_STP         ;Else if it's the ALL-STOP state
                   BNE  NOT_ALL_STOP     ;Then call the ALL-STOP routine
                   JSR  ALL_STP_ST       ;and exit
                   JMP  DISP_EXIT
                                         
NOT_ALL_STOP:      CMPA #ADJUST           ;Else if it's the ADJUST state
                   BNE  NOT_ADJUST        ;Then call the ADJUST routine
                   JSR  ADJUST_ST
                   JMP  DISP_EXIT
         
NOT_ADJUST:        CMPA #REV_TRN           ;Else if it's the REVERSE_TRN state
                   BNE  NOT_REVERSE_TURN   ;Then call the REVERSE_TURN state
                   JSR  REV_TRN_ST         ;and exit
                   JMP  DISP_EXIT
                                            ;Else if it's the FORWARD-TRN state
NOT_REVERSE_TURN:  CMPA #FWD_TRN            ;Then call the FORWARD_TRN state
                   BNE  NO_MORE_STATES      ;and exit
                   JSR  FWD_TRN_ST
                   JMP  DISP_EXIT

NO_MORE_STATES:    SWI                     ; Otherwise, we're in an invalid state.  Finish program. 

DISP_EXIT:         RTS                     ; Exit from the state dispatcher



;*****************************************************************
;* Subroutine Section                                            *
;*****************************************************************  

;*****************************************************************
;* Encode Sensor Data                                            *
;* Converts the raw sensors values produced by the ADC into bits.*
;* Sensor State is a one byte value that holds the state of the  *
;* EEBOT's sensors.                                              *
;*
;* Sensor State:  b7 (MSB), b6, b5, b4, b3, b2, b1, b0 (LSB)     *
;* If a sensor registers dark, its corresponding bit is set to 1.*
;* Bits 5 through 7 are unused and they are set to 0.
;***************************************************************** 


ENCODE_SENSOR_DATA: MOVB  #0, SENSOR_STATE  ; Initialize Sensor_State to 0  // So all bits are set to zero

ENCODE_BOW_DATA:    LDAA  #$CA              ; Dark threshold for Sensor A (BOW), $CA = 202 
                    CMPA  SENSOR_BOW        ; Compare threshold to raw value from sensor
                    BHS   ENCODE_PORT_DATA   ;
                    BSET  SENSOR_STATE, #16 ; If Bow Sensor is DARK, set bit 4 to 1

ENCODE_PORT_DATA:   LDAA  #$CA              ; Dark threshold for Sensor B (PORT)           
                    CMPA  SENSOR_PORT       ; Compare threshold to raw value for sensor
                    BHS   ENCODE_MID_DATA  
                    BSET  SENSOR_STATE, #8  ; If Port Sensor is DARK, set bit 3 to 1

ENCODE_MID_DATA:    LDAA  #$CA              ; Dark threshold for Sensor C (MID)          
                    CMPA  SENSOR_MID       ; Compare threshold to raw value for sensor
                    BHS   ENCODE_STBD_DATA
                    BSET  SENSOR_STATE, #4  ; If Mid Sensor is DARK, set bit 2 to 1

ENCODE_STBD_DATA:   LDAA  #$AA              ; Dark threshold for Sensor D (STBD)          
                    CMPA  SENSOR_STBD       ; Compare threshold to raw value for sensor
                    BHS   ENCODE_LINE_DATA
                    BSET  SENSOR_STATE, #2  ; If STBD Sensor is DARK, set bit 1 to 1  

ENCODE_LINE_DATA:   LDAA  #$23                   ; Lower threshold for Sensors E-F (LINE)          
                    CMPA  SENSOR_LINE            ; Compare threshold to raw value for sensor
                    BHS   ENCODE_LINE_NONUNI 
ENCODE_LINE_UNI:    LDAA  #$90                   ; Higher threshold for Sensors E-F (LINE)
                    CMPA  SENSOR_LINE
                    BLS   ENCODE_LINE_NONUNI                    
                    BSET  SENSOR_STATE, #1        ; If Line Sensor is UNIFORM, set bit 0 to 1
                    RTS   
ENCODE_LINE_NONUNI: BCLR  SENSOR_STATE, #1     ; If Port Sensor is NON-UNIFORM, set bit 0 to 0
                    RTS                           ; Return


;*****************************************************************
;* Adjust Line Tracking                                          *
;* This subroutine configures the EEBOT's motor power controls   *
;* based upon how the bot is currently following the line.       *
;*****************************************************************

ADJUST_LINE_TRACKING:  
                       BRSET SENSOR_STATE, #%00010100, ALL_MOTORS_GO       ; If BOW and MID sensors are dark, we're centered
                                                                           ; Then we go straight.  Otherwise, we're crooked.
                       BRA ADJUST_MOTORS

ALL_MOTORS_GO:         
                       LDAA #%00110000           ; Port Motor ON, Starboard Motor ON
                       STAA MOTOR_PWR_STATE
                       RTS


ADJUST_MOTORS:         LDAA   #$23               ; If LINE Sensor < Low Threshold
                       CMPA   SENSOR_LINE        ; Then adjust the bot left
                       BHS    ADJUST_LEFT         

                       LDAA #$90                ; High Threshold (Previously, we used $A0)
                       CMPA SENSOR_LINE         ; Check if SENSOR_LINE > 60
                       BLS  ADJUST_RIGHT
                         
                       LDAA #%00011000          ; Otherwise, E-F SENSOR is not askew
                       STAA MOTOR_PWR_STATE
                       RTS                      ; Keep both motors ON and return
                        
ADJUST_LEFT:           LDAA #%00010000          ; Port Motor ON, Starboard Motor OFF
                       STAA MOTOR_PWR_STATE
                       RTS
                       
ADJUST_RIGHT:          LDAA #%00100000          ; Port Motor OFF, Starboard Motor ON
                       STAA MOTOR_PWR_STATE
                       RTS


;*****************************************************************  
START_ST:     BRCLR PORTAD0,  $04, NO_FWD        ; If FWD_BMP
              JSR   INIT_FLW                     ; Initialize the FOLLOW state
              MOVB  #FLW, CRNT_STATE             ; Go into the FOLLOW state
              BRA   START_EXIT
              
NO_FWD:       NOP                                ; Else
START_EXIT    RTS                                ; Return to the MAIN routine

;*****************************************************************
; FLW_ST is the FOLLOW state that coordinates  
; the EEBOT's line-following activities.
;*****************************************************************
FLW_ST:       BRSET  PORTAD0, $04, NO_FWD_BUMP   ; If FWD_BUMP then
              JSR    INIT_REV                    ; initialize the REVERSE routine
              MOVB   #REV, CRNT_STATE            ; set the state to REVERSE
              JMP    FLW_EXIT                    ; and return 
            
NO_FWD_BUMP:  BRSET  PORTAD0, $08, NO_REAR_BUMP  ; If REAR_BUMP, then we should stop
              JSR    INIT_ALL_STP                ; So initialize the ALL_STOP state
              MOVB   #ALL_STP, CRNT_STATE        ; and change state to ALL_STOP
              JMP    FLW_EXIT                    ; and return

NO_REAR_BUMP: BRSET  SENSOR_STATE, #%00001100, IS_INTERSECT  ; If PORT & MID sensors active, make a left turn
              LDAA   TOF_COUNTER                             ; Else if Tc > Tflw, then
              CMPA   T_FLW                                   ; the robot should read sensors and adjust
              BNE    NO_FWD_TURN                             ; so
              JSR    INIT_ADJUST                             ; initialize the ADJUST state
              MOVB   #ADJUST, CRNT_STATE                    ; and go to that state
              JMP    FLW_EXIT  

IS_INTERSECT: JSR    INIT_FWD_TRN                       ; Initialize motors to make a left turn
              MOVB   #FWD_TRN, CRNT_STATE               ; Set state to FWD_TRN state
              JMP    FLW_EXIT              
            
NO_FWD_TURN  NOP                                 ; Else
FLW_EXIT     RTS                                 ; Return to the MAIN routine

;*****************************************************************
REV_ST:      LDAA TOF_COUNTER                    ; If Tc > Trev, then
             CMPA T_REV                          ; the robot should make a FORWARD turn
             BNE  NO_REV_TRN
             JSR  INIT_REV_TRN                   ; So initialize the REV_TRN state
             MOVB #REV_TRN, CRNT_STATE           ; Set state to REV_TRN
             BRA  REV_EXIT                       ; and return

NO_REV_TRN:  NOP                                 ; Else
REV_EXIT:    RTS                                 ; Return to the MAIN routine

;*****************************************************************
ALL_STP_ST:   BRSET PORTAD0, $04, NO_START      ; If FWD_BUMP
              BCLR  PTT, %00110000              ; Initialize the START state (both motors off)
              MOVB  #START, CRNT_STATE          ; Set the state to START
              BRA   ALL_STP_EXIT                ; And return

NO_START:     NOP                               ; Else
ALL_STP_EXIT: RTS                               ; Return to the MAIN routine

;*****************************************************************           
ADJUST_ST:   JSR   INIT_FLW                    ; With adjustments made to the motors,
             MOVB  #FLW, CRNT_STATE            ; then set state to FOLLOW
             RTS                               ; And return to the MAIN routine

;*****************************************************************
REV_TRN_ST:   LDAA  TOF_COUNTER               ; If Tc > T(revturn) then
              CMPA  T_REV_TRN                 ; the robot should go follow line
              BNE   NO_FWD_RT                   
              JSR   INIT_FLW                  ; So initialize the FOLLOW state
              MOVB  #FLW, CRNT_STATE          ; Set state to FOLLOW
              BRA   REV_TRN_EXIT              ; and return

NO_FWD_RT:    NOP                             ; Else
REV_TRN_EXIT: RTS                             ; Return to the MAIN routine

;*****************************************************************
INIT_FLW:     BCLR  PORTA, %00000011          ; Set FWD direction for both motors
              LDAA  MOTOR_PWR_STATE           ; Set Port T (Motor Power) with MOTOR_PWR_STATE variable
              STAA  PTT
              LDAA  TOF_COUNTER               ; Mark the follow time T(flw)
              ADDA  #FLW_INT
              STAA  T_FLW
              RTS

;*****************************************************************
INIT_REV:     BSET  PORTA, %00000011          ; Set REV direction for both motors
              BSET  PTT,   %00110000          ; Turn on the drive motors
              LDAA  TOF_COUNTER               ; Maek the reverse time Trev
              ADDA  #REV_INT
              STAA  T_REV
              RTS
              
;*****************************************************************
INIT_ALL_STP: BCLR  PTT, %00110000            ; Turn off the drive motors
              RTS

;*****************************************************************
;* This method performs a forward LEFT 90-degree turn.           *
;*****************************************************************
INIT_FWD_TRN:      BCLR  PORTA, %00000010    ; Set Starboard Motor FWD
                    BSET  PTT,   %00100000    ; Turn on Starboard Motor
                    BCLR  PTT,   %00010000    ; Turn off Port Motor
                    LDAA  TOF_COUNTER              ; Mark the fwd_turn time, T_FWD_TRN
                    ADDA  #FWD_TRN_INT 
                    STAA  T_FWD_TRN 
                    RTS
;*******************************************************************
FWD_TRN_ST: 
                 LDAA  TOF_COUNTER                 ; If Tc > T(fwdtrn) then
                 CMPA  T_FWD_TRN                   ; the robot should go FORWARD
                 BNE   NO_FWD_FT
                 JSR   INIT_ADJUST                 ; Make adjustments to the motors based on sensor data
                 JSR   INIT_FLW                    ; Then initialize the FOLLOW state
                 MOVB  #FLW, CRNT_STATE            ; Set state to FOLLOW
                 BRA   FWD_TRN_EXIT              ; And return

NO_FWD_FT:      NOP                              ; Else
                RTS
                
FWD_TRN_EXIT:   MOVB #FLW, CRNT_STATE
                RTS   

;*****************************************************************
;INIT_ADJUST pauses the motors and gets readings 
;from all the EEBOT's photosensors.  It then encodes the data
;and adjusts the motors accordingly.
;*****************************************************************
INIT_ADJUST:  BCLR PTT, %00110000  ; Turn off motors while making decision
              JSR G_LEDS_ON        ; Enable the guider LEDs
              
              LDY #$600            ; A 30ms delay (previously 258)
              JSR del_50us         ; to give photosensors enough time to react
              
              JSR READ_SENSORS     ; Read the 5 guider sensors
              JSR G_LEDS_OFF       ; Disable the guider LEDs
              JSR ENCODE_SENSOR_DATA   ; Encode the sensor data 
              JSR ADJUST_LINE_TRACKING ; Adjust motors based upon sensor data
              
              RTS 

;*****************************************************************
INIT_REV_TRN: BCLR  PORTA, %00000010        ; Set FWD direction for STARBOARD (right) motor      
              LDAA  TOF_COUNTER             ; Mark the rev_turn time, T_REV_TRN
              ADDA  #REV_TRN_INT
              STAA  T_REV_TRN
              RTS
;*****************************************************************

;*****************************************************************            
;* Initialization of the LCD: 4-bit data width, 2-line display   *
;* turn on display, cursor and blinking off. Shift cursor right. *
;*****************************************************************

initLCD:    BSET   DDRB, %11111111    ; Configure Pins PB7, PB6, PB5, PB4 for Output
            BSET   DDRJ, %11000000    ; Configure Pins PJ7, PJ4 for output
            LDY    #2000              ; Wait for LCD to be ready
            JSR    del_50us           ; -"-
            LDAA   #$38               ; Set 8-bit data, 2-line display
            JSR    cmd2LCD            ;
            LDAA   #$0C               ; Display on, cursor off, blinking off
            JSR    cmd2LCD            ;  -"-
            LDAA   #$06               ; Move cursor to the right after entering a character
            JSR    cmd2LCD            ;  -"-
            RTS

;*****************************************************************           
;* Clear display and home cursor                                 *
;*****************************************************************
clrLCD:     LDAA   #$01               ; Clear cursor and return to home position
            JSR    cmd2LCD            ;  -"-
            LDY    #40                ; Wait until "clear cursor" command is complete
            JSR    del_50us           ;  -"-
            RTS


;*****************************************************************           
;* ([Y] * 50us)-delay routine. E-clk=41,67 ns                    *
;*****************************************************************
del_50us:   PSHX                     ;2 E-clk
eloop:      LDX    #300              ;2 E-clk
iloop:      NOP     
            DBNE   X,iloop           ;3 E-clk
            DBNE   Y,eloop           ;3 E-clk
            PULX                     ;3 E-clk
            RTS                      ;5 E-clk


;*****************************************************************
;* This function sends a command in accumulator A to the LCD     *
;*****************************************************************   
cmd2LCD:    BCLR   LCD_CNTR, LCD_RS  ;Select the LCD Instruction Register (IR)
            JSR    dataMov           ;Send Data to IR
            RTS    

;*****************************************************************
;* This function outputs a NULL-terminated string pointed to by X*
;*****************************************************************   
putsLCD:    LDAA   1,X+              ; Get one character from the string
            BEQ    donePS            ; Reach NULL character?
            JSR    putcLCD
            BRA    putsLCD
donePS:     RTS


;*****************************************************************
;* This function outputs the character in accumulator A to LCD   *
;*****************************************************************   
putcLCD:    BSET   LCD_CNTR, LCD_RS  ; Select the LCD Data Register
            JSR    dataMov           ; Send data to DR
            RTS
            
;*****************************************************************
;* This function sends data to the LCD IR or DR depending on RS  *
;*****************************************************************   
dataMov:    BSET   LCD_CNTR, LCD_E   ; Pull the LCD E-signal high
            STAA   LCD_DAT           ; Send the upper 4 bits of data to LCD
            BCLR   LCD_CNTR, LCD_E   ; Pull the LCD E-signal low to complete the write operation
            
            LDY    #1                ; Adding this delay will complete internal ops for most instructions
            JSR    del_50us
            RTS

;*****************************************************************
;* This function converts a 16 bit binary number in .D into      *
;* BCD digits in BCD_BUFFER                                      *
;*****************************************************************  

INT2BCD: XGDX        ; Save the binary number into .X
         LDAA #0     ; Clear the BCD_BUFFER
         STAA TEN_THOUS
         STAA THOUSANDS
         STAA HUNDREDS
         STAA TENS
         STAA UNITS
         STAA BCD_SPARE
         STAA BCD_SPARE+1
*
         CPX #0      ; Check for a zero input and if so, exit
         BEQ CON_EXIT 
*
         XGDX        ; Not zero, get the binary number back to .D as dividend
         LDX #10     ; Setup 10 (Decimal!) as the divisor
         IDIV        ; Divide: Quotient is now in .X, remainder in .D
         STAB UNITS  ; Store remainder
         CPX #0      ; If quotient is zero, then exit
         BEQ CON_EXIT 
*
         XGDX        ; else swap first quotient back into .D
         LDX #10     ; and setup for another divide by 10
         IDIV
         STAB TENS
         CPX #0
         BEQ CON_EXIT
*
         XGDX        ; Swap quotient back into .D
         LDX #10     ; and setup for another divide by 10
         IDIV
         STAB HUNDREDS
         CPX #0
         BEQ CON_EXIT
*
         XGDX        ; Swap quotient back into .D
         LDX #10     ; and setup for another divide by 10
         IDIV
         STAB THOUSANDS
         CPX #0
         BEQ CON_EXIT
*
         XGDX        ; Swap quotient back into .D
         LDX #10     ; and setup for another divide by 10
         IDIV
         STAB TEN_THOUS
*
CON_EXIT RTS ; We're done the conversion


;*****************************************************************
;* This function converts the BCD number in the BCD_BUFFER       *
;* into ascii format, with leading zero suppression.             *
;***************************************************************** 

BCD2ASC:    LDAA #0         ; Initialize the blanking flag
            STAA NO_BLANK
*
C_TTHOU    LDAA TEN_THOUS  ; Check the 'ten_thousands' digit
           ORAA NO_BLANK
           BNE  NOT_BLANK1
*
ISBLANK1   LDAA #' '       ; It's blank, so store a space
           STAA TEN_THOUS  ; and check the 'thousands' digit
           BRA  C_THOU 
*
NOT_BLANK1 LDAA TEN_THOUS  ; Get the 'ten_thousands' digit
           ORAA #$30       ; Convert to ascii
           STAA TEN_THOUS
           LDAA #$1        ; Signal that we have seen a 'non-blank' digit
           STAA NO_BLANK
*
C_THOU     LDAA THOUSANDS  ; Check the thousands digit for blankness
           ORAA NO_BLANK   ; If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK2
*
ISBLANK2   LDAA #' '       ; Thousands digit is blank
           STAA THOUSANDS  ; so store a space
           BRA C_HUNS      ; and check the hundreds digit
*
NOT_BLANK2 LDAA THOUSANDS  ; (similar to 'ten_thousands' case)
           ORAA #$30
           STAA THOUSANDS
           LDAA #$1
           STAA NO_BLANK
*
C_HUNS     LDAA HUNDREDS   ; Check the hundreds digit for blankness
           ORAA NO_BLANK   ; If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK3
*
ISBLANK3   LDAA #' '       ; Hundreds digit is blank
           STAA HUNDREDS   ; so store a space
           BRA C_TENS      ; and check the tens digit
*
NOT_BLANK3 LDAA HUNDREDS   ; (similar to 'ten_thousands' case)
           ORAA #$30
           STAA HUNDREDS
           LDAA #$1
           STAA NO_BLANK
*
C_TENS     LDAA TENS       ; Check the tens digit for blankness
           ORAA NO_BLANK   ; If it's blank and 'no-blank' is still zero
           BNE NOT_BLANK4
*
ISBLANK4   LDAA #' '       ; Tens digit is blank
           STAA TENS       ; so store a space
           BRA C_UNITS     ; and check the units digit
*
NOT_BLANK4 LDAA TENS       ; (similar to 'ten_thousands' case)
           ORAA #$30
           STAA TENS
*
C_UNITS    LDAA UNITS      ; No blank check necessary, convert to ascii.
           ORAA #$30
           STAA UNITS
*
           RTS             ; We're done


;*****************************************************************
;* Initialize AD converter                                       *
;*****************************************************************  
initAD:     MOVB #$C0, ATDCTL2       ; Power up AD, select fast flag clear
            JSR  del_50us            ; Wait for 50 us
            MOVB #$00, ATDCTL3       ; 8 conversions in a sequence
            MOVB #$85, ATDCTL4       ; Res = 8, Conv-Clks = 2, Prescal = 12
            BSET ATDDIEN, $0C        ; Configure pins AN03, AN02 as Digital Inputs
            RTS

;*****************************************************************
ENABLE_TOF:  
            LDAA  #%10000000   
            STAA  TSCR1        ; Enable TCNT
            STAA  TFLG2        ; Clear TOF
            LDAA  #%10000100   ; Enable TOI and select prescale factor equal to 16 (#%10000100)
            STAA  TSCR2
            RTS
 
;*****************************************************************
TOF_ISR:  
            INC   TOF_COUNTER
            LDAA  #%10000000   ; Clear
            STAA  TFLG2        ; TOF
            RTI
            
;*****************************************************************
DISABLE_TOF:
            LDAA  #%00000100   ; Disable TOI and leave prescale fator at 16
            STAA  TSCR2
            RTS

;*****************************************************************
;* Update Display (Battery Voltage + Current State)              *
;*****************************************************************  
UPDT_DISPL: MOVB  #$90, ATDCTL5     ; r.just., unsign., sing.conv., mult., ch0, start conv.                         
            BRCLR  ATDSTAT0, $80, *  ; Wait until the conversion sequence is complete 
            
            LDAA  ATDDR4L            ; load the ch4 result into AccA
            LDAB  #39               ; AccB = 39
            MUL                     ; AccD = (1st Result) * 39
            ADDD  #600               ; AccD = (1st Result) * 39 + 600
            
            JSR   INT2BCD
            JSR   BCD2ASC
            
            LDAA  #$90             ; Move LCD cursor to the 1st row, end of msg1
            JSR   cmd2LCD
            
            LDAA  TEN_THOUS         ; Output the TEN_THOUS ASCII character
            JSR   putcLCD           ; "
            
            LDAA  THOUSANDS         ; Output the THOUSANDS ASCII character
            JSR   putcLCD           ; "
            
            LDAA  #'.'              ; Output the decimal point ASCII character
            JSR   putcLCD           ; "
            
            LDAA  HUNDREDS          ; Output the HUNDREDS ASCII character
            JSR   putcLCD           ; "
            
            LDAA  TENS              ; Output the TENS ASCII character
            JSR   putcLCD           ; "
            
            LDAA  UNITS             ; Output the UNITS ASCII character
            JSR   putcLCD           ; "
            
            LDAA  #$C6             ; move LCD cursor to the 2nd row, end of msg2
            JSR   cmd2LCD  
            
            LDAB  CRNT_STATE       ; Display current state
            LSLB  
            LSLB 
            LSLB  
            LDX   #tab
            ABX
            JSR   putsLCD
            RTS

;---------------------------------------------------------------------------
;               Guider LEDs ON

; This routine enables the guider LEDs so that readings of the sensor
;  correspond to the 'illuminated' situation.

; Passed:  Nothing
; Returns: Nothing
; Side:    PORTA bit 5 is changed

G_LEDS_ON       BSET PORTA,%00100000 ; Set bit 5
                RTS

;ญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญญ
;               Guider LEDs OFF

; This routine disables the guider LEDs. Readings of the sensor
;  correspond to the 'ambient lighting' situation.

; Passed:  Nothing
; Returns: Nothing
; Side:    PORTA bit 5 is changed

G_LEDS_OFF      BCLR PORTA,%00100000 ; Clear bit 5
                RTS

;---------------------------------------------------------------------------
;               Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
;  registers.

; Note: Do not confuse the analog multiplexer on the Guider board with the
;  multiplexer in the HCS12. The guider board mux must be set to the
;  appropriate channel using the SELECT_SENSOR routine. The HCS12 always
;  reads the selected sensor on the HCS12 A/D channel AN1.

; The A/D conversion mode used in this routine is to read the A/D channel
;  AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
;  used in this routine is the value from AN1, read from ATDDR0. However,
;  other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.

; Passed:       None
; Returns:      Sensor readings in:
;               SENSOR_LINE	(0) (Sensor E/F)
;               SENSOR_BOW  (1) (Sensor A)
;               SENSOR_PORT	(2) (Sensor B)
;               SENSOR_MID  (3) (Sensor C)
;               SENSOR_STBD	(4) (Sensor D)
; Note:
;   The sensor number is shown in brackets
;
; Algorithm:
;        Initialize the sensor number to 0
;        Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop   Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
;        Repeat
;          	Read ATDSTAT0
;        Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
;        Store the contents of ATDDR0L at the pointer
;        If the pointer is at the last entry in Sensor Array, then 
;           Exit
;        Else 
;           Increment the sensor number
;           Increment the pointer
;        Loop again.

READ_SENSORS    CLR  SENSOR_NUM      ; Select sensor number 0
                LDX  #SENSOR_LINE    ; Point at the start of the sensor array

RS_MAIN_LOOP    LDAA SENSOR_NUM	     ; Select the correct sensor input
                JSR  SELECT_SENSOR   ;  on the hardware
                LDY  #400            ; 20 ms delay to allow the
                JSR  del_50us        ;  sensor to stabilize
                
                LDAA #%10000001      ; Start A/D conversion on AN1
                STAA ATDCTL5
                BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

                LDAA ATDDR0L         ; A/D conversion is complete in ATDDR0L
                STAA 0,X             ;  so copy it to the sensor register
                CPX  #SENSOR_STBD    ; If this is the last reading
                BEQ  RS_EXIT         ; Then exit

                INC  SENSOR_NUM      ; Else, increment the sensor number
                INX                  ;  and the pointer into the sensor array
                BRA  RS_MAIN_LOOP    ;  and do it again

RS_EXIT	        RTS

;---------------------------------------------------------------------------
;               Select Sensor

; This routine selects the sensor number passed in ACCA. The motor direction
;  bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
;  same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
;  which selects the guider sensor to be connected to AN1.

; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed

; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear 
;        the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
;        11100011. The zeros in the mask clear the corresponding bits in the
;        TEMP. The 1's have no effect.
; Next, move the sensor selection number left two positions to align it
;        with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with 
;  the mask 00011100. The zeros in the mask clear everything except
;        the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
;  The effect is that only bits 2,3,4 are changed in the TEMP, and these
;  bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.

SELECT_SENSOR   PSHA                 ; Save the sensor number for the moment

                LDAA PORTA           ; Clear the sensor selection bits to zeros
                ANDA #%11100011      ;
                STAA TEMP            ; and save it into TEMP

                PULA                 ; Get the sensor number
                ASLA                 ; Shift the selection number left, twice
                ASLA                 ; 
                ANDA #%00011100      ; Clear irrelevant bit positions

                ORAA TEMP            ; OR it into the sensor bit positions
                STAA PORTA           ; Update the hardware
                RTS
;
            
;*****************************************************************
;* Motor Power & Direction Subroutines                           *
;*****************************************************************  

STARON      LDAA  PTT
            ORAA  #%00100000
            STAA  PTT
            RTS
            
STAROFF     LDAA  PTT
            ANDA  #%11011111
            STAA  PTT
            RTS

PORTON      LDAA  PTT
            ORAA  #%00010000
            STAA  PTT
            RTS
            
PORTOFF     LDAA  PTT
            ANDA  #%11101111
            STAA  PTT
            RTS


STARFWD     LDAA  PORTA
            ANDA  #%11111101
            STAA  PORTA
            RTS

STARREV     LDAA  PORTA
            ORAA  #%00000010
            STAA  PORTA
            RTS

PORTFWD     LDAA  PORTA
            ANDA  #%11111110
            STAA  PORTA
            RTS

PORTREV     LDAA  PORTA
            ORAA  #%00000001
            STAA  PORTA
            RTS

            
;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            FDB   Entry           ; Reset Vector
            
            ORG   $FFDE           ; Timer Overflow Interrupt Vector
            DC.W  TOF_ISR

