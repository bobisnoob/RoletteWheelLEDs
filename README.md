# RoletteWheelLEDs

This program is a demostration of using a state-machine.
  * 				Roulette LED wheel of 8 LEDS are used where LEDs are turned
  * 				on and off in sequential(Current LED is on previous LED is off)
  * 				and "click" sound is produced when and LED is turned on.
  * 				The roulette LED wheel has three states as follows:
  *					1) When user pushbutton is not pressed(normal mode) LEDs spinning
  *					   speed is in normal speed (200ms) between LED off and on.
  *					2) When user pushbutton is pressed and kept pressed(fast mode)
  *					   LEDs spinning speed between on and off is fast (50ms)in our
  *					   case.
  *					3) When user pushbutton is released (slow mode) LEDs spinning
  *					   speed is slowed down incrementally by 200ms until it reaches
  *					   2000ms then it stops completely. The last LED where it stopped
  *					   at will blink 4 times and stays on for 5 seconds then to goes
  *					   back to the first state(normal mode) and so on.
  *
  *					Note: "Click" sound effect will be produced according to the current state
  *						   of roulette LED wheel. The sound speed is coherent with
  *						   the speed of roulette LED wheel.
  
