;*****************************************************************
;* COE 538 - EEBOT Project - Version 1, Date: 15 Nov 2023        *
;* Aaron Gaba (501062030), Keedon Cokely (501124817),            * 
;* Mohammed Furqaan Shaikh (501034654) 
;*                                                               *.                                                 
;*****************************************************************

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

; Time intervals
MOVE_INT      EQU     3      ; 150us second delay (at 23Hz)
REV_INT       EQU     23     ; 1 second interval
FWD_TRN_INT   EQU     34
REV_TRN_INT   EQU     37

; Motor commands
PORT_ON_STBD_ON   EQU   %00110000
PORT_ON_STBD_OFF  EQU   %00010000
PORT_OFF_STBD_ON  EQU   %00100000
PORT_OFF_STBD_OFF EQU   %00000000

PORT_FWD_STBD_FWD EQU   %00000000
PORT_FWD_STBD_REV EQU   %00000010
PORT_REV_STBD_FWD EQU   %00000001
PORT_REV_STBD_REV EQU   %00000011

;Definitions of various states
START         EQU     0
OBSERVE       EQU     1
REV           EQU     2
ALL_STP       EQU     3
FWD_TRN       EQU     4
REV_TRN       EQU     5
FOLLOW_LINE   EQU     6



; Variable/data section
           ORG $3850

TOF_COUNTER dc.b 0          ; Timer, incremented at 23Hz
CRNT_STATE  dc.b 3          ; Current State
T_MOVE       ds.b 1          ; FWD time
T_REV       ds.b 1          ; REV time
T_FWD_TRN   ds.b 1          ; FWD_TURN time
T_REV_TRN   ds.b 1          ; REV_TURN time
LAST_INTERSCT dc.b 0        ; Last intersection

           
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

MOTOR_PWR_CTRL    dc.b 0
PREV_INTERSECTION dc.b 0             ; 0 (no prev intersection), 1 (left 90), 2 (right 90), 3 (T-intersection)
                
TEMP            RMB 1                ; Temporary location


;*****************************************************************
;* Code Section                                                  *
;*****************************************************************
            ORG   $4000

Entry:
_Startup:
            CLI
            LDS   #$4000            ; Initialize Stack Pointer
            
            ;JSR   initAD            ; Initialize ATD converter (this may conflict w/openADC)
            
            JSR   INIT_PORTS        ; Initialize ports
            JSR   openADC           ; Initalize ADC
            JSR   initLCD           ; Initialize LCD
            JSR   clrLCD            ; Clear LCD & Home Cursor
            
            LDX   #msg1             ; Display Msg1
            JSR   putsLCD
            
            LDAA  #$C0              ; Move LCD cursor to the 2nd row
            JSR   cmd2LCD
            
            LDX   #msg2             ; Display Msg2
            JSR   putsLCD
            
            JSR   ENABLE_TOF        ; Jump to TOF initialization
            
MAIN:       ; Observe and then act 
            JSR   UPDT_DISPL
            JSR   OBSERVE_ENV
            LDAA  CRNT_STATE
            JSR   DISPATCHER
            BRA   MAIN

;---------------------------------------------------------------------
;Initialize ports
;---------------------------------------------------------------------
INIT_PORTS: BCLR  DDRAD, $FF        ; Make PortAD an input
            BSET  DDRA,  $FF        ; Make PortA an output
            BSET  DDRB,  $FF        ; Make PortB an output
            BSET  DDRJ,  $C0        ; Make Pins 7,6 of PTJ outputs
            RTS

;---------------------------------------------------------------------
;Turn on ADC
;---------------------------------------------------------------------
openADC:    MOVB  #$80, ATDCTL2     ; Turn on ADC (ATDCTL2)
            LDY   #1                ; Wait 50 us for ADC to be ready
            JSR   del_50us          
            MOVB  #$20, ATDCTL3     ; 4 Conversions on Channel AN1
            MOVB  #$97, ATDCTL4     ; 8-bit resolution, prescaler=48
            RTS

;*****************************************************************
;* Data Section                                                  *
;*****************************************************************

msg1        dc.b "Battery volt ", 0
msg2        dc.b "State ", 0
tab         dc.b "START  ", 0
            dc.b "OBSERVE", 0
            dc.b "REV    ", 0
            dc.b "ALL_STP", 0
            dc.b "FWD_TRN", 0
            dc.b "REV_TRN", 0
            dc.b "FOLLOW ", 0


;*****************************************************************
;* Environment Observer & Encoder                                *
;*****************************************************************  
OBSERVE_ENV:  JSR G_LEDS_ON              ; Enable the guider LEDs
              
              LDY #1000                  ; 50 ms delay for the photosensors to respond
              JSR del_50us
              
              JSR READ_SENSORS           ; Read the 5 guider sensors
              JSR ENCODE_SENSOR_DATA
              JSR G_LEDS_OFF
              RTS


;*****************************************************************
;* State Dispatcher                                              *
;*****************************************************************  
;This routine calls the appropriate state handler based on the
;current state.

DISPATCHER:        CMPA #START           ;If it's the START state               
                   BNE  NOT_START        ;then call the START routine
                   JSR  START_ST         ;and exit
                   JMP  DISP_EXIT

NOT_START:         CMPA #OBSERVE         ;Else if it's the FORWARD state
                   BNE  NOT_OBSERVE      ;Then call the FORWARD routine 
                   JSR  OBSERVE_ST       ;and exit
                   JMP  DISP_EXIT

NOT_OBSERVE:       CMPA #FOLLOW_LINE     ;Else if it's the FOLLOW-LINE state
                   BNE  NOT_FOLLOW_LINE  ;Then call the FOLLOW-LINE routine
                   JSR  FOLLOW_LINE_ST   ;and exit
                   JMP  DISP_EXIT

NOT_FOLLOW_LINE:   CMPA #REV             ;Else if it's the REVERSE state
                   BNE  NOT_REVERSE      ;Then call the REVERSE routine
                   JSR  REV_ST           ;and exit
                   JMP  DISP_EXIT


NOT_REVERSE:       CMPA #ALL_STP         ;Else if it's the ALL-STOP state
                   BNE  NOT_ALL_STOP     ;Then call the ALL-STOP routine
                   JSR  ALL_STP_ST       ;and exit
                   JMP  DISP_EXIT
                                         
NOT_ALL_STOP:      CMPA #FWD_TRN           ;Else if it's the FORWARD_TURN state
                   BNE  NOT_FORWARD_TURN   ;Then call the FORWARD_TURN routine
                   JSR  FWD_TRN_ST         ;and exit
                   JMP  DISP_EXIT
         
NOT_FORWARD_TURN:  CMPA #REV_TRN           ;Else if it's the REVERSE_TRN state
                   BNE  NOT_REVERSE_TURN   ;Then call the REVERSE_TURN state
                   JSR  REV_TRN_ST         ;and exit
                   JMP  DISP_EXIT

NOT_REVERSE_TURN:  SWI                     ; Else, the current state is NOT defined, so stop

DISP_EXIT:         RTS                     ; Exit from the state dispatcher


;*****************************************************************
;* Subroutine Section                                            *
;*****************************************************************  


;*****************************************************************
;Adjust motor control based upon line following conditions       *
;*****************************************************************
GET_MOTOR_CONTROL:  BCLR  PTT, #%00110000  ; Turn off both motors
                    BRSET SENSOR_STATE, #%00010101, ALL_MOTORS_ON
                    BRSET SENSOR_STATE, #%00000100, ADJUST_ANGLE                     
                    RTS

ALL_MOTORS_ON:      BCLR  PORTA, #%00000011  ; Have both motors go forward
                    BSET  PTT,   #%00110000  ; Turn on both motors 
                    RTS

ADJUST_ANGLE:       LDAA  SENSOR_LINE
                    CMPA  #$44
                    BLS   ADJUST_BOT_LEFT

ADJUST_BOT_RIGHT:   JSR   PORTOFF
                    JSR   STARFWD
                    JSR   STARON
                    RTS
                    
ADJUST_BOT_LEFT:    JSR   STAROFF
                    JSR   PORTFWD
                    JSR   PORTON
                    RTS


;*****************************************************************
;* Encode all sensors into the SENSOR_STATE variable             *
;*****************************************************************

ENCODE_SENSOR_DATA: MOVB  #0, SENSOR_STATE  ; Initialize Sensor_State to 0

ENCODE_BOW_DATA:    LDAA  #$CA              ; Dark threshold for Sensor A (BOW), $CA = 202 
                    CMPA  SENSOR_BOW        ; Compare threshold to raw value from sensor
                    BLS   ENCODE_BOW_DARK   ;
ENCODE_BOW_LIGHT:   BCLR  SENSOR_STATE, #16 ; If Bow Sensor is LIGHT, set bit 4 to 0
                    JMP   ENCODE_PORT_DATA 

ENCODE_BOW_DARK:    BSET  SENSOR_STATE, #16 ; If Bow Sensor is DARK, set bit 4 to 1

ENCODE_PORT_DATA:   LDAA  #$CA              ; Dark threshold for Sensor B (PORT)          
                    CMPA  SENSOR_PORT       ; Compare threshold to raw value for sensor
                    BLS   ENCODE_PORT_DARK  
ENCODE_PORT_LIGHT:  BCLR  SENSOR_STATE, #8  ; If Port Sensor is LIGHT, set bit 3 to 0
                    JMP   ENCODE_MID_DATA   
ENCODE_PORT_DARK:   BSET  SENSOR_STATE, #8  ; If Port Sensor is DARK, set bit 3 to 1

ENCODE_MID_DATA:    LDAA  #$CA              ; Dark threshold for Sensor C (MID)          
                    CMPA  SENSOR_MID       ; Compare threshold to raw value for sensor
                    BLS   ENCODE_MID_DARK  
ENCODE_MID_LIGHT:   BCLR  SENSOR_STATE, #4  ; If Mid Sensor is LIGHT, set bit 2 to 0
                    JMP   ENCODE_STBD_DATA   
ENCODE_MID_DARK:    BSET  SENSOR_STATE, #4  ; If Mid Sensor is DARK, set bit 2 to 1

ENCODE_STBD_DATA:   LDAA  #$CA              ; Dark threshold for Sensor D (STBD)          
                    CMPA  SENSOR_STBD       ; Compare threshold to raw value for sensor
                    BLS   ENCODE_STBD_DARK  
ENCODE_STBD_LIGHT:  BCLR  SENSOR_STATE, #2  ; If STBD Sensor is LIGHT, set bit 1 to 0
                    JMP   ENCODE_LINE_DATA   
ENCODE_STBD_DARK:   BSET  SENSOR_STATE, #2  ; If STBD Sensor is DARK, set bit 1 to 1

ENCODE_LINE_DATA:   LDAA  #$44                   ; Lower threshold for Sensors E-F (LINE)          
                    CMPA  SENSOR_LINE            ; Compare threshold to raw value for sensor
                    BHS   ENCODE_LINE_NONUNI 
ENCODE_LINE_UNI:    LDAA  #$61                   ; Higher threshold for Sensors E-F (LINE)
                    CMPA  SENSOR_LINE
                    BLS   ENCODE_LINE_NONUNI                    
                    BSET  SENSOR_STATE, #1        ; If Line Sensor is UNIFORM, set bit 0 to 1
                    RTS   
ENCODE_LINE_NONUNI: BCLR  SENSOR_STATE, #1     ; If Port Sensor is NON-UNIFORM, set bit 0 to 0
                    RTS                           ; Return

;*****************************************************************  
START_ST:     BRCLR PORTAD0,  $04, NO_FWD        ; If FWD_BMP
              MOVB  #OBSERVE, CRNT_STATE         ; Go into the OBSERVE state
              BRA   START_EXIT
              
NO_FWD:       NOP                                ; Else
START_EXIT    RTS                                ; Return to the MAIN routine

;*******************************************************************
OBSERVE_ST:   BRSET  PORTAD0, $04, NO_FWD_BUMP   ; If FWD_BUMP then we have a collision,
              JSR    INIT_REV                    ; initialize the REVERSE routine
              MOVB   #REV, CRNT_STATE            ; set the state to REVERSE
              JMP    OBS_EXIT                    ; and return 
            
NO_FWD_BUMP:  BRSET  PORTAD0, $08, NO_INTERSECT  ; If REAR_BUMP, then we should stop
              JSR    INIT_ALL_STP                ; So initialize the ALL_STOP state
              MOVB   #ALL_STP, CRNT_STATE        ; and change state to ALL_STOP
              JMP    OBS_EXIT                    ; and return

NO_INTERSECT: BRSET  SENSOR_STATE, %00000111, HANDLE_RT_90  ;Handle Right 90 deg corner, if detected
              JSR    INIT_FOLLOW_LINE
              MOVB   #FOLLOW_LINE, CRNT_STATE
              JMP    OBS_EXIT

HANDLE_RT_90: JSR    INIT_FWD_TRN                ; If right angle encountered, 
              MOVB   #FWD_TRN, CRNT_STATE        ; initialize bot to make 90 degree turn
              JMP    OBS_EXIT
              
OBS_EXIT     RTS                                 ; Return to the MAIN routine


;*****************************************************************
INIT_FOLLOW_LINE: JSR   GET_MOTOR_CONTROL
                  LDAA  TOF_COUNTER               ; Mark the fwd time T(fwd)
                  ADDA  #MOVE_INT
                  STAA  T_MOVE
                  RTS
;*****************************************************************
FOLLOW_LINE_ST:   LDAA  TOF_COUNTER
                  CMPA  T_MOVE
                  BNE   STILL_MOVING
                  MOVB  #OBSERVE, CRNT_STATE
                  RTS

STILL_MOVING:     NOP
                  RTS
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
FWD_TRN_ST:   LDAA  TOF_COUNTER                 ; If Tc > T(fwdtrn) then
              CMPA  T_FWD_TRN                   ; the robot should observe again
              BNE   NO_FWD_FT
              MOVB  #OBSERVE, CRNT_STATE        ; Set state to FWD
              BRA   FWD_TRN_EXIT                ; And return

NO_FWD_FT:    NOP                              ; Else
FWD_TRN_EXIT: RTS                              ; Return to the MAIN routine



;*****************************************************************
REV_TRN_ST:   LDAA  TOF_COUNTER               ; If Tc > T(revturn) then
              CMPA  T_REV_TRN                 ; the robot should go FWD
              BNE   NO_FWD_RT                   
              MOVB  #OBSERVE, CRNT_STATE      ; Set state to FWD
              BRA   REV_TRN_EXIT              ; and return

NO_FWD_RT:    NOP                             ; Else
REV_TRN_EXIT: RTS                             ; Return to the MAIN routine

;*****************************************************************
INIT_REV:     BSET  PORTA, %00000011          ; Set REV direction for both motors
              BSET  PTT,   %00110000          ; Turn on the drive motors
              LDAA  TOF_COUNTER               ; Mark the fwd time Trev
              ADDA  #REV_INT
              STAA  T_REV
              RTS
              
;*****************************************************************
INIT_ALL_STP: BCLR  PTT, %00110000            ; Turn off the drive motors
              RTS
    
;*****************************************************************
INIT_FWD_TRN: JSR   PORTFWD
              JSR   STAROFF
              JSR   PORTON
              LDAA  TOF_COUNTER              ; Mark the fwd_turn time, T_FWD_TRN
              ADDA  #FWD_TRN_INT 
              STAA  T_FWD_TRN
              RTS 

;*****************************************************************
INIT_REV_TRN: 
              JSR   STARFWD
              JSR   PORTREV
              JSR   STARON
              JSR   PORTON
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
            LDAA  #%10000100   ; Enable TOI and select prescale factor equal to 16
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



;*****************************************************************
;* Update Display (Battery Voltage + Current State)              *
;*****************************************************************  
UPDT_DISPL: ;MOVB  #$90, ATDCTL5     ; r.just., unsign., sing.conv., mult., ch0, start conv.                         
            ;BRCLR  ATDSTAT0, $80, *  ; Wait until the conversion sequence is complete 
            
            ;LDAA  ATDDR4L            ; load the ch4 result into AccA
            LDAA  #40               ; Dummy value (since we're ignoring the ADC subsystem)
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

;­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­
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
;* Binary to ASCII                                               *
;*****************************************************************   
leftHLF:    LSRA                     ; Shift data to right
            LSRA                     ;
            LSRA
            LSRA

rightHLF:   ANDA   #$0F              ; Mask top half
            ADDA   #$30              ; Convert to ASCII
            CMPA   #$39              ;
            BLE    out               ; Jump if 0-9
            ADDA   #$07              ; Convert to Hex A-F

out:        RTS                   
            
            
;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            FDB   Entry           ; Reset Vector
            
            ORG   $FFDE           ; Timer Overflow Interrupt Vector
            DC.W  TOF_ISR

