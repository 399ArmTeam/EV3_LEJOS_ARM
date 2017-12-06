package control;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Matrix;

/**
 * Move controller singleton class.
 * Uses PIDController threads to move three servo motors.
 * @author nicholasmayne
 *
 */
public class Move {
	
	private static Move INSTANCE = null;
	public static UnregulatedMotor[] SERVOS = null;
	
	// PID CONTROLLER PARAMS
	// K gain terms for PID control of motors at joints 1, 2, 3
	private static final double[][] K = {{4, 0.05, 2}, {4, 0.01, 2}, {5, 0.05, 2}};
	private static final int P[] = {100, 100, 100};	// power maximum for PID control 0 ≤ p ≤ 100

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
	 * Move Joint 1 by theta1 radians in del_q, i.e. del_q.get(0, 0)
	 * @param del_q a Matrix of joint angles
	 * @param wait wait for movement to finish before returning, or not if false
	 * @param timeout the number of milliseconds to wait before givingup 
	 */
	public static void J1(Matrix del_q, Boolean wait, int timeout) {
		Thread JOINT_1 = new Thread(new PIDController(SERVOS[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], timeout));
		JOINT_1.start();
		if (wait) {while(JOINT_1.isAlive()) {}}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joint 2 by theta2 radians in del_q, i.e. del_q.get(1, 0)
	 * @param del_q a Matrix of joint angles
	 * @param wait wait for movement to finish before returning, or not if false
	 * @param timeout the number of milliseconds to wait before givingup 
	 */
	public static void J2(Matrix del_q, Boolean wait, int timeout) {
 		Thread JOINT_2 = new Thread(new PIDController(SERVOS[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], timeout));
		JOINT_2.start();
		if (wait) {while(JOINT_2.isAlive()) {}}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joint 3 by theta3 radians in del_q, i.e. del_q.get(2, 0)
	 * @param del_q a Matrix of joint angles
	 * @param wait wait for movement to finish before returning, or not if false
	 * @param timeout the number of milliseconds to wait before givingup 
	 */
	public static void J3(Matrix del_q, Boolean wait, int timeout) {
 		Thread JOINT_3 = new Thread(new PIDController(SERVOS[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], timeout));
		JOINT_3.start();
		if (wait) {while(JOINT_3.isAlive()) {}}	// wait for joint movement threads to finish
	}
	
	/**
	 * Move Joints 1, 2, 3 by the radians in del_q.
	 * @param del_q a Matrix of joint angles
	 * @param wait wait for movement to finish before returning, or not if false
	 * @param timeout the number of milliseconds to wait before givingup 
	 */
	public static void J123(Matrix del_q, Boolean wait, int timeout) {
		Thread JOINT_1 = new Thread(new PIDController(SERVOS[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], timeout));
 		Thread JOINT_2 = new Thread(new PIDController(SERVOS[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], timeout));
 		Thread JOINT_3 = new Thread(new PIDController(SERVOS[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], timeout));
		JOINT_1.start();
		JOINT_2.start();
		JOINT_3.start();
		if (wait) {while(JOINT_1.isAlive() || JOINT_2.isAlive() || JOINT_3.isAlive()) {}}	// wait for joint movement threads to finish
	}
}
