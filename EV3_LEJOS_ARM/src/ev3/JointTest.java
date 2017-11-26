package ev3;

import control.Move;
import lejos.hardware.Button;
import lejos.utility.Matrix;

/**
 * A button based test program for moving three joints.
 * 
 * @author nicholasmayne
 *
 */
public class JointTest {
	private static Matrix del_q = new Matrix(3,1);	// Vector of joint angles in radians
	
	public static void main(String[] args) {
		del_q.set(0, 0, Math.PI/8);	// theta 1 for J1
		del_q.set(1, 0, Math.PI/8);	// theta 2 for J2
		del_q.set(2, 0, Math.PI/8);	// theta 3 for J3
		
				
		while(true) {
			switch(Button.waitForAnyPress()) {
				case 1:
					System.out.printf("Button: 1\n");
					break;
				case 2:
					System.out.printf("Button: 2\n");
					break;
				case 3:
					System.out.printf("Button: 3\n");
					break;
				case 4:
					System.out.printf("Button: 4\n");
					break;
				case 5:
					System.out.printf("Button: 5\n");
					break;
				case 6:
					System.out.printf("Button: 6\n");
					break;
				default:
					System.out.printf("DEFAULT\n");
					break;
			}
		}
			
//		Move.J1(del_q);
//		Move.J2(del_q);
//		Move.J3(del_q);
//		Move.J123(del_q);
	}

}
