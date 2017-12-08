package control;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
//import lejos.utility.Matrix;
import Jama.Matrix;

/**
 * Move controller singleton class.
 * Uses PIDController threads to move three servo motors.
 * @author nicholasmayne
 *
 */
public class Move {
	
	private static Move INSTANCE = null;
	public static UnregulatedMotor[] SERVOS = null;
	public static Boolean stopAllBalancing;
	
	// PID CONTROLLER PARAMS
	// K gain terms for PID control of motors at joints 1, 2, 3
	private static final double[][] K = {{4, 0, 1}, {4, 0.01, 1.5}, {2, 0, 1}};
	private static final int P[] = {45, 70, 50};	// power maximum for PID control 0 ≤ p ≤ 100

	/**
	 * Singleton Constructor
	 */
	private Move(UnregulatedMotor[] SERVOS) {
		Move.SERVOS = SERVOS;
	}
	
	/**
	 * Instantiate a Move singleton
	 * 
	 * @return this Move object
	 */
	public static Move getInstance(UnregulatedMotor[] SERVOS) {
		if (INSTANCE == null) {
			INSTANCE = new Move(SERVOS);
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
		Boolean J1_BalanceFlag = false;
		Boolean J2_BalanceFlag = false;
		Boolean J3_BalanceFlag = false;
		
		stopAllBalancing = true;
		
		if((int) Math.toDegrees(del_q.get(1, 0))<0) {
			K[1][0] = 1.5;
		}
		if((int) Math.toDegrees(del_q.get(2, 0))<0) {
			K[2][0] = 0.55;
		}
		
		Thread JOINT_1 = new Thread(new PIDController(SERVOS[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], timeout));
 		Thread JOINT_2 = new Thread(new PIDController(SERVOS[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], timeout));
 		Thread JOINT_3 = new Thread(new PIDController(SERVOS[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], timeout));
		
		Balance J1_Balance = new Balance(SERVOS[0], -3);
		Balance J2_Balance = new Balance(SERVOS[1], -6);
		Balance J3_Balance = new Balance(SERVOS[2], -6);
		
		Thread BalanceAll = new Thread(balanceAllThread);
		
 		JOINT_1.start();
		JOINT_2.start();
		JOINT_3.start();

		// Balance the Joints
		while(JOINT_1.isAlive() || JOINT_2.isAlive() || JOINT_3.isAlive()) {
			if (!JOINT_1.isAlive() && !J1_BalanceFlag) {
				Thread J1_Balance_Thread = new Thread(J1_Balance);
				J1_Balance_Thread.start();
				J1_BalanceFlag = true;
			}
			if (!JOINT_2.isAlive() && !J2_BalanceFlag) {

				Thread J2_Balance_Thread = new Thread(J2_Balance);
				J2_Balance_Thread.start();
				J2_BalanceFlag = true;
			}
			if (!JOINT_3.isAlive() && !J3_BalanceFlag) {

				Thread J3_Balance_Thread = new Thread(J3_Balance);
				J3_Balance_Thread.start();
				J3_BalanceFlag = true;
			}
		}	// wait for joint movement threads to finish
		
		J1_Balance.stopBalance = true;
		J2_Balance.stopBalance = true;
		J3_Balance.stopBalance = true;
		
		Delay.msDelay(200); // Delay to wait for threads to stop
		
		BalanceAll.start();
		
		K[1][0] = 4;
		K[2][0] = 2;
	}
	
	public static Runnable balanceAllThread = new Runnable() {
        public void run() {
            balance();
        }
    };
	
	/**
	 * Set the Robot Arm to an origin position
	 */
	public static void setOrigin() {
		Matrix origin_angles = new Matrix(3,1);	// Vector of joint angles in radians	
		origin_angles.set(1, 0, (-4) * Math.PI);	// theta 2 for J2
		origin_angles.set(2, 0, Math.PI);		// theta 3 for J3
		
		Move.J2(origin_angles, true, 1000);
		Move.J3(origin_angles, true, 1000);

		resetAllTachs();
	}
	
	/**
	 * Reset all the tachometers for the three servos.
	 */
	public static void resetAllTachs() {
		Move.SERVOS[0].resetTachoCount();
		Move.SERVOS[1].resetTachoCount();
		Move.SERVOS[2].resetTachoCount();
	}
	
	/**
	 * Balance all the joints.
	 */
	public static void balance() {
		resetAllTachs();
		
		Balance J1_Balance = new Balance(SERVOS[0], -2);
		Balance J2_Balance = new Balance(SERVOS[1], -4);
		Balance J3_Balance = new Balance(SERVOS[2], -4);

		Thread J1_Balance_Thread = new Thread(J1_Balance);
		Thread J2_Balance_Thread = new Thread(J2_Balance);
		Thread J3_Balance_Thread = new Thread(J3_Balance);
		
		J1_Balance_Thread.start();
		J2_Balance_Thread.start();
		J3_Balance_Thread.start();

		stopAllBalancing = false;
		
		while (!stopAllBalancing) {}
		
		System.out.println("STOPED BALANCE");
		
		J1_Balance.stopBalance = true;
		J2_Balance.stopBalance = true;
		J3_Balance.stopBalance = true;
	}
}
