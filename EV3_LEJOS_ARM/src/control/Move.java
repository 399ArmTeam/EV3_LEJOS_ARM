package control;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Matrix;

/**
 * A move controller singleton class.
 * Use PID controllers to move three servo motors.
 * @author nicholasmayne
 *
 */
public class Move {
	
	private static Move INSTANCE = null;
	private static UnregulatedMotor[] SERVOS = null;
	
	// PID CONTROLLER PARAMS
	private static final double[][] K = {{2, 0, 1}, {2, 0, 1}, {2, 0, 1}};	// K gain terms for PID control of motors at joints 1, 2, & 3
	private static final int P[] = {100, 100, 100};								// power maximum for PID control 0 ≤ p ≤ 100
	private static final int TIMEOUT = 3000;									// kill PID if set point not reached within this many milliseconds

	/**
	 * Singleton Constructor
	 */
	private Move() {
		// Setup joint motors
		SERVOS = new UnregulatedMotor[3];
		SERVOS[0] = new UnregulatedMotor(MotorPort.A);	// Joint 1
		SERVOS[1] = new UnregulatedMotor(MotorPort.B);	// Joint 2
		SERVOS[2] = new UnregulatedMotor(MotorPort.C);	// Joint 3
	}
	
	/**
	 * Instantiate a Move singleton
	 * 
	 * @return this Move object
	 */
	public static Move getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Move();
		}
		return INSTANCE;
	}
		
	/**
	 * Move Joint 1 by theta1 radians in del_q.
	 */
	public static void J1(Matrix del_q) {
		Thread JOINT_1 = new Thread(new PIDController(SERVOS[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], TIMEOUT));
		JOINT_1.start();
		while(JOINT_1.isAlive()) {}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joint 2 by theta2 radians in del_q.
	 */
	public static void J2(Matrix del_q) {
 		Thread JOINT_2 = new Thread(new PIDController(SERVOS[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], TIMEOUT));
		JOINT_2.start();
		while(JOINT_2.isAlive()) {}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joint 3 by theta3 radians in del_q.
	 */
	public static void J3(Matrix del_q) {
 		Thread JOINT_3 = new Thread(new PIDController(SERVOS[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], TIMEOUT));
		JOINT_3.start();
		while(JOINT_3.isAlive()) {}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joints 1, 2 & 3 by the radians in del_q.
	 */
	public static void J123(Matrix del_q) {
		Thread JOINT_1 = new Thread(new PIDController(SERVOS[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], TIMEOUT));
 		Thread JOINT_2 = new Thread(new PIDController(SERVOS[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], TIMEOUT));
 		Thread JOINT_3 = new Thread(new PIDController(SERVOS[2], (int) Math.toDegrees(del_q.get(3, 0)), K[2], P[2], TIMEOUT));
		JOINT_1.start();
		JOINT_2.start();
		JOINT_3.start();
		while(JOINT_1.isAlive() || JOINT_2.isAlive() || JOINT_3.isAlive()) {}	// wait for joint movement threads to finish
	}
}
