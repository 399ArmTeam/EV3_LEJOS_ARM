package ev3;

import control.Move;
import lejos.hardware.Button;
import lejos.utility.Matrix;

/**
 * Test program for moving three joints using EV3 buttons.
 * Joint 1: LEFT, RIGHT
 * Joint 2: UP, DOWN
 * Joint 3: ESC, ENTER
 * 
 * @author nicholasmayne
 *
 */
public class JointTest {
	private static Matrix del_q = new Matrix(3,1);	// Vector of joint angles in radians
	
	public static void main(String[] args) {
		//Move.getInstance();			// Instantiate Move singleton
		del_q.set(0, 0, Math.PI/2);	// theta 1 for J1
		del_q.set(1, 0, Math.PI/8);	// theta 2 for J2
		del_q.set(2, 0, Math.PI/16);	// theta 3 for J3
			
		while(true) {
			switch(Button.waitForAnyPress()) {
				case 1: Move.J2(del_q, false, 0);			break;	// UP Button
				case 2: Move.J3(del_q.times(-1), false, 0); break;	// ENTER Button
				case 4:	Move.J2(del_q.times(-1), false, 0); break;	// DOWN Button
				case 8:	Move.J1(del_q.times(-1), false, 0); break;	// RIGHT Button
				case 16:	Move.J1(del_q, false, 0); 			break;	// LEFT Button
				case 32:Move.J3(del_q, false, 0); 			break;	// ESC Button
			}
		}
	}
}
