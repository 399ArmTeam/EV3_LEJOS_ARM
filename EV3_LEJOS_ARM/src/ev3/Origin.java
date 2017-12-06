package ev3;

import control.Move;
import lejos.utility.Delay;
import lejos.utility.Matrix;
import lejos.hardware.Button;


public class Origin {
	private static Matrix del_q = new Matrix(3,1);	// Vector of joint angles in radians
//	private static EV3TouchSensor J2_Touch = new EV3TouchSensor(SensorPort.S2);
//	static float[] J2_Sample = new float[1];

	public static void main(String[] args) {
		Move.getInstance();				// Instantiate Move singleton
		del_q.set(1, 0, (-4) * Math.PI);	// theta 2 for J2
		del_q.set(2, 0, Math.PI);		// theta 3 for J3
		
		Move.J2(del_q, true, 3000);
		Move.J3(del_q, true, 3000);

		Move.SERVOS[1].resetTachoCount();
		Move.SERVOS[2].resetTachoCount();
		
		System.out.printf("Origin: J1: %d J2: %d", Move.SERVOS[1].getTachoCount(), Move.SERVOS[2].getTachoCount());

		Button.waitForAnyPress();

	}

}
