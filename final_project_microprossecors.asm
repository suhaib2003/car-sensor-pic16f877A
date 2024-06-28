
_Init:

;final_project_microprossecors.c,27 :: 		void Init() {
;final_project_microprossecors.c,28 :: 		TRISD = 0b10101010; // Set Trig pins as outputs and Echo pins as inputs
	MOVLW      170
	MOVWF      TRISD+0
;final_project_microprossecors.c,29 :: 		ADCON1 = 0x07;      // Configure all AN pins as digital
	MOVLW      7
	MOVWF      ADCON1+0
;final_project_microprossecors.c,30 :: 		Lcd_Init();         // Initialize LCD
	CALL       _Lcd_Init+0
;final_project_microprossecors.c,31 :: 		Lcd_Cmd(_LCD_CLEAR);  // Clear display
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;final_project_microprossecors.c,32 :: 		Lcd_Cmd(_LCD_CURSOR_OFF); // Cursor off
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;final_project_microprossecors.c,33 :: 		TRISC = 0b00000000;
	CLRF       TRISC+0
;final_project_microprossecors.c,34 :: 		}
L_end_Init:
	RETURN
; end of _Init

_Measure_Distance:

;final_project_microprossecors.c,36 :: 		unsigned int Measure_Distance(char trigPin, char echoPin) {
;final_project_microprossecors.c,39 :: 		PORTD &= ~(1 << trigPin); // Clear Trig pin    This line clears the trigPin. It ensures the pin starts from a low state. The ~(1 << trigPin) generates a bitmask where all bits are set to 1 except for the bit at the position of trigPin, which is set to 0. The AND operation then sets only the trigPin to low, without affecting other pins of PORTD.
	MOVF       FARG_Measure_Distance_trigPin+0, 0
	MOVWF      R1+0
	MOVLW      1
	MOVWF      R0+0
	MOVF       R1+0, 0
L__Measure_Distance16:
	BTFSC      STATUS+0, 2
	GOTO       L__Measure_Distance17
	RLF        R0+0, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Measure_Distance16
L__Measure_Distance17:
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;final_project_microprossecors.c,40 :: 		Delay_us(2);                               // Delays execution for 2 microseconds to ensure the pin is fully cleared.
	MOVLW      3
	MOVWF      R13+0
L_Measure_Distance0:
	DECFSZ     R13+0, 1
	GOTO       L_Measure_Distance0
;final_project_microprossecors.c,41 :: 		PORTD |= (1 << trigPin);  // Set Trig pin        This line sets the trigPin high by shifting a 1 to the bit position of trigPin, creating a mask where all bits are 0 except for the trigPin bit, which is 1. The OR operation sets this pin high.
	MOVF       FARG_Measure_Distance_trigPin+0, 0
	MOVWF      R1+0
	MOVLW      1
	MOVWF      R0+0
	MOVF       R1+0, 0
L__Measure_Distance18:
	BTFSC      STATUS+0, 2
	GOTO       L__Measure_Distance19
	RLF        R0+0, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Measure_Distance18
L__Measure_Distance19:
	MOVF       R0+0, 0
	IORWF      PORTD+0, 1
;final_project_microprossecors.c,42 :: 		Delay_us(10);             // 10us pulse      The pin remains high for 10 microseconds, which is the typical pulse duration required to trigger an ultrasonic sensor.
	MOVLW      16
	MOVWF      R13+0
L_Measure_Distance1:
	DECFSZ     R13+0, 1
	GOTO       L_Measure_Distance1
	NOP
;final_project_microprossecors.c,43 :: 		PORTD &= ~(1 << trigPin); // Clear Trig pin    Immediately after the pulse, the trigPin is cleared again, setting up for the reception of the echo.
	MOVF       FARG_Measure_Distance_trigPin+0, 0
	MOVWF      R1+0
	MOVLW      1
	MOVWF      R0+0
	MOVF       R1+0, 0
L__Measure_Distance20:
	BTFSC      STATUS+0, 2
	GOTO       L__Measure_Distance21
	RLF        R0+0, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Measure_Distance20
L__Measure_Distance21:
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;final_project_microprossecors.c,46 :: 		while (!(PORTD & (1 << echoPin)));  // Wait for the echo to start   This loop waits (blocks) until the echoPin goes high, indicating the start of the echo pulse received from the ultrasonic sensor.
L_Measure_Distance2:
	MOVF       FARG_Measure_Distance_echoPin+0, 0
	MOVWF      R2+0
	MOVLW      1
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__Measure_Distance22:
	BTFSC      STATUS+0, 2
	GOTO       L__Measure_Distance23
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Measure_Distance22
L__Measure_Distance23:
	MOVF       PORTD+0, 0
	ANDWF      R0+0, 1
	MOVLW      0
	ANDWF      R0+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L_Measure_Distance3
	GOTO       L_Measure_Distance2
L_Measure_Distance3:
;final_project_microprossecors.c,47 :: 		T1CON = 0x10; // Timer1 settings, prescaler = 1     Configures Timer1 with appropriate settings. Here, a prescaler of 1 is set (assuming 0x10 corresponds to these settings in your specific PIC microcontroller), which allows for higher resolution timing.
	MOVLW      16
	MOVWF      T1CON+0
;final_project_microprossecors.c,48 :: 		TMR1H = 0; TMR1L = 0;  // Clear Timer1        Clears the Timer1 registers to start counting from zero.
	CLRF       TMR1H+0
	CLRF       TMR1L+0
;final_project_microprossecors.c,49 :: 		T1CON.TMR1ON = 1;  // Start Timer1      Starts Timer1 to begin timing the duration of the echo pulse.
	BSF        T1CON+0, 0
;final_project_microprossecors.c,51 :: 		while (PORTD & (1 << echoPin));  // Wait for the echo to end   Continues to loop as long as the echoPin remains high. Once the pin goes low, it indicates the end of the echo pulse.
L_Measure_Distance4:
	MOVF       FARG_Measure_Distance_echoPin+0, 0
	MOVWF      R2+0
	MOVLW      1
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__Measure_Distance24:
	BTFSC      STATUS+0, 2
	GOTO       L__Measure_Distance25
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__Measure_Distance24
L__Measure_Distance25:
	MOVF       PORTD+0, 0
	ANDWF      R0+0, 1
	MOVLW      0
	ANDWF      R0+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Measure_Distance5
	GOTO       L_Measure_Distance4
L_Measure_Distance5:
;final_project_microprossecors.c,52 :: 		T1CON.TMR1ON = 0;  // Stop Timer1      Stops Timer1 as the echo pulse has finished.
	BCF        T1CON+0, 0
;final_project_microprossecors.c,55 :: 		count = (TMR1L | TMR1H << 8);  // Read Timer1     Reads the value from Timer1, which represents the time elapsed from the start to the end of the echo pulse. TMR1L holds the lower 8 bits, and TMR1H holds the upper 8 bits of the 16-bit timer count. The left shift and OR operation combines these two into a full 16-bit value.
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       R0+0, 0
	IORWF      TMR1L+0, 0
	MOVWF      Measure_Distance_count_L0+0
	MOVLW      0
	IORWF      R0+1, 0
	MOVWF      Measure_Distance_count_L0+1
	MOVLW      0
	MOVWF      Measure_Distance_count_L0+2
	MOVWF      Measure_Distance_count_L0+3
;final_project_microprossecors.c,56 :: 		return (count / 58);  // Convert to cm (using 58 to approximate for cm)  Converts the timer count into a distance in centimeters. The number 58 is used as a conversion factor, based on the speed of sound and the timer frequency. Specifically, this factor converts the time of flight of the ultrasound pulse into a spatial measurement (distance) assuming a speed of sound in air around 343 meters per second at room temperature.
	MOVLW      58
	MOVWF      R4+0
	CLRF       R4+1
	CLRF       R4+2
	CLRF       R4+3
	MOVF       Measure_Distance_count_L0+0, 0
	MOVWF      R0+0
	MOVF       Measure_Distance_count_L0+1, 0
	MOVWF      R0+1
	MOVF       Measure_Distance_count_L0+2, 0
	MOVWF      R0+2
	MOVF       Measure_Distance_count_L0+3, 0
	MOVWF      R0+3
	CALL       _Div_32x32_U+0
;final_project_microprossecors.c,57 :: 		}
L_end_Measure_Distance:
	RETURN
; end of _Measure_Distance

_main:

;final_project_microprossecors.c,59 :: 		void main() {
;final_project_microprossecors.c,62 :: 		Init();
	CALL       _Init+0
;final_project_microprossecors.c,63 :: 		while (1) {
L_main6:
;final_project_microprossecors.c,64 :: 		distance1 = Measure_Distance(Trig1, Echo1);  // Measure distance for Sensor 1
	CLRF       FARG_Measure_Distance_trigPin+0
	MOVLW      1
	MOVWF      FARG_Measure_Distance_echoPin+0
	CALL       _Measure_Distance+0
	MOVF       R0+0, 0
	MOVWF      main_distance1_L0+0
	MOVF       R0+1, 0
	MOVWF      main_distance1_L0+1
	CLRF       main_distance1_L0+2
	CLRF       main_distance1_L0+3
;final_project_microprossecors.c,65 :: 		distance2 = Measure_Distance(Trig2, Echo2);  // Measure distance for Sensor 2
	MOVLW      2
	MOVWF      FARG_Measure_Distance_trigPin+0
	MOVLW      3
	MOVWF      FARG_Measure_Distance_echoPin+0
	CALL       _Measure_Distance+0
	MOVF       R0+0, 0
	MOVWF      main_distance2_L0+0
	MOVF       R0+1, 0
	MOVWF      main_distance2_L0+1
	CLRF       main_distance2_L0+2
	CLRF       main_distance2_L0+3
;final_project_microprossecors.c,66 :: 		distance3 = Measure_Distance(Trig3, Echo3);  // Measure distance for Sensor 3
	MOVLW      4
	MOVWF      FARG_Measure_Distance_trigPin+0
	MOVLW      5
	MOVWF      FARG_Measure_Distance_echoPin+0
	CALL       _Measure_Distance+0
	MOVF       R0+0, 0
	MOVWF      main_distance3_L0+0
	MOVF       R0+1, 0
	MOVWF      main_distance3_L0+1
	CLRF       main_distance3_L0+2
	CLRF       main_distance3_L0+3
;final_project_microprossecors.c,67 :: 		distance4 = Measure_Distance(Trig4, Echo4);  // Measure distance for Sensor 4
	MOVLW      6
	MOVWF      FARG_Measure_Distance_trigPin+0
	MOVLW      7
	MOVWF      FARG_Measure_Distance_echoPin+0
	CALL       _Measure_Distance+0
	MOVF       R0+0, 0
	MOVWF      main_distance4_L0+0
	MOVF       R0+1, 0
	MOVWF      main_distance4_L0+1
	CLRF       main_distance4_L0+2
	CLRF       main_distance4_L0+3
;final_project_microprossecors.c,69 :: 		if ((distance1 <= 50) || (distance2 <= 50) || (distance3 <= 50) || (distance4 <= 50)) {
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      main_distance1_L0+3, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main27
	MOVF       main_distance1_L0+2, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main27
	MOVF       main_distance1_L0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main27
	MOVF       main_distance1_L0+0, 0
	SUBLW      50
L__main27:
	BTFSC      STATUS+0, 0
	GOTO       L__main13
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      main_distance2_L0+3, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main28
	MOVF       main_distance2_L0+2, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main28
	MOVF       main_distance2_L0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main28
	MOVF       main_distance2_L0+0, 0
	SUBLW      50
L__main28:
	BTFSC      STATUS+0, 0
	GOTO       L__main13
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      main_distance3_L0+3, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main29
	MOVF       main_distance3_L0+2, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main29
	MOVF       main_distance3_L0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main29
	MOVF       main_distance3_L0+0, 0
	SUBLW      50
L__main29:
	BTFSC      STATUS+0, 0
	GOTO       L__main13
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      main_distance4_L0+3, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main30
	MOVF       main_distance4_L0+2, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main30
	MOVF       main_distance4_L0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main30
	MOVF       main_distance4_L0+0, 0
	SUBLW      50
L__main30:
	BTFSC      STATUS+0, 0
	GOTO       L__main13
	GOTO       L_main10
L__main13:
;final_project_microprossecors.c,70 :: 		PORTC = 255;  // Set all bits in PORTC, assuming PORTC is configured as output
	MOVLW      255
	MOVWF      PORTC+0
;final_project_microprossecors.c,71 :: 		} else {
	GOTO       L_main11
L_main10:
;final_project_microprossecors.c,72 :: 		PORTC = 0;    // Clear all bits in PORTC
	CLRF       PORTC+0
;final_project_microprossecors.c,73 :: 		}
L_main11:
;final_project_microprossecors.c,76 :: 		distance1 = distance1/2 ;
	RRF        main_distance1_L0+3, 1
	RRF        main_distance1_L0+2, 1
	RRF        main_distance1_L0+1, 1
	RRF        main_distance1_L0+0, 1
	BCF        main_distance1_L0+3, 7
	BTFSC      main_distance1_L0+3, 6
	BSF        main_distance1_L0+3, 7
	BTFSS      main_distance1_L0+3, 7
	GOTO       L__main31
	BTFSS      STATUS+0, 0
	GOTO       L__main31
	INCF       main_distance1_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance1_L0+1, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance1_L0+2, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance1_L0+3, 1
L__main31:
;final_project_microprossecors.c,77 :: 		distance2 = distance2/2 ;
	RRF        main_distance2_L0+3, 1
	RRF        main_distance2_L0+2, 1
	RRF        main_distance2_L0+1, 1
	RRF        main_distance2_L0+0, 1
	BCF        main_distance2_L0+3, 7
	BTFSC      main_distance2_L0+3, 6
	BSF        main_distance2_L0+3, 7
	BTFSS      main_distance2_L0+3, 7
	GOTO       L__main32
	BTFSS      STATUS+0, 0
	GOTO       L__main32
	INCF       main_distance2_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance2_L0+1, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance2_L0+2, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance2_L0+3, 1
L__main32:
;final_project_microprossecors.c,78 :: 		distance3 = distance3/2 ;
	RRF        main_distance3_L0+3, 1
	RRF        main_distance3_L0+2, 1
	RRF        main_distance3_L0+1, 1
	RRF        main_distance3_L0+0, 1
	BCF        main_distance3_L0+3, 7
	BTFSC      main_distance3_L0+3, 6
	BSF        main_distance3_L0+3, 7
	BTFSS      main_distance3_L0+3, 7
	GOTO       L__main33
	BTFSS      STATUS+0, 0
	GOTO       L__main33
	INCF       main_distance3_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance3_L0+1, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance3_L0+2, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance3_L0+3, 1
L__main33:
;final_project_microprossecors.c,79 :: 		distance4 = distance4/2 ;
	RRF        main_distance4_L0+3, 1
	RRF        main_distance4_L0+2, 1
	RRF        main_distance4_L0+1, 1
	RRF        main_distance4_L0+0, 1
	BCF        main_distance4_L0+3, 7
	BTFSC      main_distance4_L0+3, 6
	BSF        main_distance4_L0+3, 7
	BTFSS      main_distance4_L0+3, 7
	GOTO       L__main34
	BTFSS      STATUS+0, 0
	GOTO       L__main34
	INCF       main_distance4_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance4_L0+1, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance4_L0+2, 1
	BTFSC      STATUS+0, 2
	INCF       main_distance4_L0+3, 1
L__main34:
;final_project_microprossecors.c,83 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;final_project_microprossecors.c,84 :: 		LongToStr(distance1, dis1);
	MOVF       main_distance1_L0+0, 0
	MOVWF      FARG_LongToStr_input+0
	MOVF       main_distance1_L0+1, 0
	MOVWF      FARG_LongToStr_input+1
	MOVF       main_distance1_L0+2, 0
	MOVWF      FARG_LongToStr_input+2
	MOVF       main_distance1_L0+3, 0
	MOVWF      FARG_LongToStr_input+3
	MOVLW      main_dis1_L0+0
	MOVWF      FARG_LongToStr_output+0
	CALL       _LongToStr+0
;final_project_microprossecors.c,85 :: 		LongToStr(distance2, dis2);
	MOVF       main_distance2_L0+0, 0
	MOVWF      FARG_LongToStr_input+0
	MOVF       main_distance2_L0+1, 0
	MOVWF      FARG_LongToStr_input+1
	MOVF       main_distance2_L0+2, 0
	MOVWF      FARG_LongToStr_input+2
	MOVF       main_distance2_L0+3, 0
	MOVWF      FARG_LongToStr_input+3
	MOVLW      main_dis2_L0+0
	MOVWF      FARG_LongToStr_output+0
	CALL       _LongToStr+0
;final_project_microprossecors.c,86 :: 		LongToStr(distance3, dis3);
	MOVF       main_distance3_L0+0, 0
	MOVWF      FARG_LongToStr_input+0
	MOVF       main_distance3_L0+1, 0
	MOVWF      FARG_LongToStr_input+1
	MOVF       main_distance3_L0+2, 0
	MOVWF      FARG_LongToStr_input+2
	MOVF       main_distance3_L0+3, 0
	MOVWF      FARG_LongToStr_input+3
	MOVLW      main_dis3_L0+0
	MOVWF      FARG_LongToStr_output+0
	CALL       _LongToStr+0
;final_project_microprossecors.c,87 :: 		LongToStr(distance4, dis4);
	MOVF       main_distance4_L0+0, 0
	MOVWF      FARG_LongToStr_input+0
	MOVF       main_distance4_L0+1, 0
	MOVWF      FARG_LongToStr_input+1
	MOVF       main_distance4_L0+2, 0
	MOVWF      FARG_LongToStr_input+2
	MOVF       main_distance4_L0+3, 0
	MOVWF      FARG_LongToStr_input+3
	MOVLW      main_dis4_L0+0
	MOVWF      FARG_LongToStr_output+0
	CALL       _LongToStr+0
;final_project_microprossecors.c,90 :: 		Lcd_Out(1, 1, dis1);      // (row,colomn,initialize or print char)
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_dis1_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,91 :: 		Lcd_Out(1, 1, "D1:");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr1_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,92 :: 		Lcd_Out(1, 15, "cm");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      15
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr2_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,95 :: 		Lcd_Out(2, 1, dis2);
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_dis2_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,96 :: 		Lcd_Out(2, 1, "D2:");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr3_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,97 :: 		Lcd_Out(2, 15, "cm");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      15
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr4_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,99 :: 		Lcd_Out(3, 1, dis3);
	MOVLW      3
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_dis3_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,100 :: 		Lcd_Out(3, 1, "D3:");
	MOVLW      3
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr5_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,101 :: 		Lcd_Out(3, 15, "cm");
	MOVLW      3
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      15
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr6_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,104 :: 		Lcd_Out(4, 1, dis4);
	MOVLW      4
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_dis4_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,105 :: 		Lcd_Out(4, 1, "D4:");
	MOVLW      4
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr7_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,106 :: 		Lcd_Out(4, 15, "cm");
	MOVLW      4
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      15
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr8_final_project_microprossecors+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;final_project_microprossecors.c,107 :: 		Delay_ms(1000);  // Refresh every second
	MOVLW      26
	MOVWF      R11+0
	MOVLW      94
	MOVWF      R12+0
	MOVLW      110
	MOVWF      R13+0
L_main12:
	DECFSZ     R13+0, 1
	GOTO       L_main12
	DECFSZ     R12+0, 1
	GOTO       L_main12
	DECFSZ     R11+0, 1
	GOTO       L_main12
	NOP
;final_project_microprossecors.c,108 :: 		}
	GOTO       L_main6
;final_project_microprossecors.c,109 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
