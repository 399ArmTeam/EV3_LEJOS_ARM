package model;
import control.PIDController;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.Port;



/**
 * 
 * @author nicholasmayne
 *
 */
public class Kinematics {
	private UnregulatedMotor[] MOTOR;
	
	private final double[][] K = {{3, 0, 1}, {4, 0.01, 2}, {3, 0, 1}};
	private final int TIMEOUT = 5000;
	private final double[] R = {7.3, 3, 0.8};	// Transmission gearing ratios

	/**
	 *  Instantiate the three motors.
	 * @param m1 Port: motor port for joint 1
	 * @param m2 Port: motor port for joint 2
	 * @param m3 Port: motor port for joint 3
	 */
	public Kinematics(Port m1, Port m2, Port m3) {
		MOTOR = new UnregulatedMotor[3];
		MOTOR[0] = new UnregulatedMotor(m1);
		MOTOR[1] = new UnregulatedMotor(m2);
		MOTOR[2] = new UnregulatedMotor(m3);
	}
		
	
	/**
	 * Move to Set Points in SP, at Powers in P
	 * @param SP int[]: Set Points for Joints 1, 2, and 3
	 * @param P int[]: Power Max for Joints 1, 2, and 3
	 */
	public void Move (int[] SP, int[] P) {
		for (int i = 0; i < 1; i++) {
			Thread JOINT_1 = new Thread(new PIDController(MOTOR[0], (int) (SP[0] * R[0]), K[0], P[0], TIMEOUT));
	 		Thread JOINT_2 = new Thread(new PIDController(MOTOR[1], (int) (SP[1] * R[1]), K[1], P[1], TIMEOUT));
	 		Thread JOINT_3 = new Thread(new PIDController(MOTOR[2], (int) (SP[2] * R[2]), K[2], P[2], TIMEOUT));
	 		
	 		JOINT_1.start();
	 		JOINT_2.start();
	 		JOINT_3.start();
	 		
			while(JOINT_1.isAlive() || JOINT_2.isAlive() || JOINT_3.isAlive()) {}		// wait for all JOINT threads to finish
		}	
	}
}
