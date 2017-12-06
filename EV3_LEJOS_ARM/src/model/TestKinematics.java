package model;

import control.PIDController;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Matrix;
import vision.TrackerReader;


public class TestKinematics {
	//public static TrackerReader tracker;
	public static boolean stopThread = false;
	public static boolean stopBaseThread = false;
	public static boolean stopShoulderThread = false;
	public static boolean stopElbowThread = false;
	
	public static UnregulatedMotor base = new UnregulatedMotor(MotorPort.A);
	public static UnregulatedMotor shoulder = new UnregulatedMotor(MotorPort.B); //The base joint
	public static UnregulatedMotor elbow = new UnregulatedMotor(MotorPort.C);//elbow joint
	
	// JOINT MOTORS
	private static UnregulatedMotor[] MOTOR;
	
	// PID CONTROLLER
	private static final double[][] K = {{5, 0, 1}, {4.0, 0, 1}, {3, 0, 1}};	// K gain terms for PID control of motors at joints 1 and 2
	private static final int P[] = {40, 80, 80};						// power maximum for PID control 0 ≤ p ≤ 100
	private static final int TIMEOUT = 3000;						// kill PID if SP not reached within this many milliseconds
	
	// CV TRACKER READER
	private static TrackerReader tracker;
	private static TrackerReader tracker2;

	// DATA MATRICES
	private static Matrix del_e = new Matrix(4,1);		
	private static Matrix del_q = new Matrix(3,1);
	private static Matrix J = new Matrix(2,2);
	private static Matrix JTest = new Matrix(4,3);
	
	public static Runnable balanceAllThread = new Runnable() {
        public void run() {
            balance();
        }
    };
    
    public static Runnable balanceBaseThread = new Runnable() {
        public void run() {
            balanceBase();
        }
    };
    
    public static Runnable balanceShoulderThread = new Runnable() {
        public void run() {
            balanceShoulder();
        }
    };
    
    public static Runnable balanceElbowThread = new Runnable() {
        public void run() {
            balanceElbow();
        }
    };

	public static void main(String args[]){
		base.resetTachoCount();
		shoulder.resetTachoCount();
		elbow.resetTachoCount();
		
		// Setup joint motors and visual tracking
				MOTOR = new UnregulatedMotor[3];
				MOTOR[0] = base;			// Joint 1
				MOTOR[1] = shoulder;			// Joint 2
				MOTOR[2] = elbow;			// Joint 3
		
		stopAllMotors();
		Delay.msDelay(1000);
		
		del_q.set(0, 0, Math.PI * 2); //base
        del_q.set(1, 0, -Math.PI * 1.4); //shoulder
        del_q.set(2, 0, -Math.PI /2); //elbow
        
        new Thread(balanceAllThread).start();
        
        //moveJ1();
        Delay.msDelay(1000);
        moveJ2();
        Delay.msDelay(1000);
        moveJ3();
        Delay.msDelay(1000);
		
        //System.out.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
		//Button.waitForAnyPress();
		
		tracker = new TrackerReader();
		tracker.start();
		
		tracker2 = new TrackerReader();							// visual servoing tracker client
        //tracker2.start();
		
		//start self balancing thread
		new Thread(balanceAllThread).start();

		/**
		 * UVS Code for L3 a 2DOF SCARA Arm
		 * @author laurapetrich, nicholasmayne
		 *
		 */

		/**
		 * Uncalibrated visual servoing using orthogonal motions and Broyden's Method Jacobian update.
		 * @param args
		 */
        
		// Instantiate del_q with radian angles for deriving initial Jacobian
        del_q.set(0, 0, Math.PI * 2); //base
        del_q.set(1, 0, Math.PI /2); //shoulder
        del_q.set(2, 0, Math.PI /2); //elbow

        // Wait for tracker to be selected, then grab it's values to get the initial position
        Delay.msDelay(1000);
		System.out.println("\n\n1) SELECT TRACKER\n2) PRESS A BUTTON");
		Sound.beep();
        Button.waitForAnyPress();

        // Get del_e from tracker by recording the starting coordinates of tracker and then moving
        // each motor by del_q and taking the difference wrt the new tracker position
		del_e = getTrackerData();
		//System.out.println(del_e.getRowDimension());
		//Button.waitForAnyPress();
		
        //del_e.setMatrix(0, 1, 0, 0, getTrackerData());
       			
        // MOVE J1
		moveJ1();
        del_e = getTrackerData().minus(del_e);
        
		// Calcualte first col of Jacobian
		JTest.setMatrix(0, 1, 0, 0, mtrxDiv(del_e, del_q.get(0, 0)));
	
        // MOVE J2
		moveJ2();
        del_e = getTrackerData().minus(del_e);
		
		// Calcualte second col of Jacobian
		JTest.setMatrix(0, 1, 1, 1, mtrxDiv(del_e, del_q.get(1, 0)));
		
		// MOVE J2
		moveJ3();
		del_e = getTrackerData().minus(del_e);
		
		// Calcualte third col of Jacobian
		JTest.setMatrix(0, 1, 2, 2, mtrxDiv(del_e, del_q.get(2, 0)));
		
		JTest.set(2, 0, 1);
		JTest.set(3, 0, 1);
		
		JTest.set(2, 1, 1);
		JTest.set(3, 1, 1);
		
		JTest.set(2, 2, 1);
		JTest.set(3, 2, 1);
		

		// Follow Target
		while(true) {
			System.out.println("\n\n1) MOVE TARGET\n2) PRESS A BUTTON");
			Sound.beep();
			Button.waitForAnyPress();
			broydensUpdate();
		}
	}
			
	/**
	 * Move Joint 1 by theta1 radians in del_q.
	 */
	private static void moveJ1() {
		Thread JOINT_1 = new Thread(new PIDController(MOTOR[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], TIMEOUT));
		
		stopAllMotors();
		
		JOINT_1.start();
		
		boolean baseThread = false;
		
		
		new Thread(balanceShoulderThread).start();
		new Thread(balanceElbowThread).start();
		
		while(JOINT_1.isAlive()) {
			/*
			if (!JOINT_1.isAlive() && !baseThread) {
				//new Thread(balanceBaseThread).start();
				baseThread = true;
			}
			*/
			
		}	// wait for joint movement threads to finish
		
		stopAllMotors();
		
		System.out.println("-----------------------------------------------------------------------------------------------------");
		
		new Thread(balanceAllThread).start();
	}
	
	/**
	 * Move Joint 2 by theta2 radians in del_q.
	 */
	private static void moveJ2() {
		if ( del_q.get(1, 0) < 0) {
			K[1][0] = 0.5;
		} else {
			K[1][0] = 3;
		}
 		Thread JOINT_2 = new Thread(new PIDController(MOTOR[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], TIMEOUT));
		
 		stopAllMotors();
 		
 		JOINT_2.start();
		
		boolean shoulderThread = false;
		
		new Thread(balanceBaseThread).start();
		new Thread(balanceElbowThread).start();
		
		while(JOINT_2.isAlive()) {
			/*
			if (!JOINT_2.isAlive() && !shoulderThread) {
				new Thread(balanceShoulderThread).start();
				shoulderThread = true;
			}
			*/
		}	// wait for joint movement threads to finish
		
		stopAllMotors();
		
		System.out.println("-----------------------------------------------------------------------------------------------------");
		
		new Thread(balanceAllThread).start();
	}
	
	private static void moveJ3() {
		if ( del_q.get(2, 0) < 0) {
			K[2][0] = 0.5;
		} else {
			K[2][0] = 4;
		}
 		Thread JOINT_3 = new Thread(new PIDController(MOTOR[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], TIMEOUT));
		
 		stopAllMotors();
 		
 		JOINT_3.start();
		
		boolean elbowThread = false;
		
		new Thread(balanceBaseThread).start();
		new Thread(balanceShoulderThread).start();
		
		while(JOINT_3.isAlive()) {
			/*
			if (!JOINT_3.isAlive() && !elbowThread) {
				new Thread(balanceElbowThread).start();
				elbowThread = true;
			}
			*/
		}	// wait for joint movement threads to finish
		
		stopAllMotors();
		
		System.out.println("-----------------------------------------------------------------------------------------------------");
		
		new Thread(balanceAllThread).start();
	}
	
	/**
	 * Move Joints 1 & 2 by the radians in del_q.
	 */
	private static void moveJ1J2() {
		Thread JOINT_1 = new Thread(new PIDController(MOTOR[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], TIMEOUT));
 		Thread JOINT_2 = new Thread(new PIDController(MOTOR[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], TIMEOUT));

 		stopAllMotors();
 		
		JOINT_1.start();
		JOINT_2.start();

		boolean baseThread = false;
		boolean shoulderThread = false;
		
		new Thread(balanceElbowThread).start();
		
		while(JOINT_1.isAlive() || JOINT_2.isAlive()) {
			if (!JOINT_1.isAlive() && !baseThread) {
				new Thread(balanceBaseThread).start();
				baseThread = true;
			}
			if (!JOINT_2.isAlive() && !shoulderThread) {
				new Thread(balanceShoulderThread).start();
				shoulderThread = true;
			}
		}	// wait for joint movement threads to finish
		
		stopAllMotors();
		
		new Thread(balanceAllThread).start();
	}
	
	/**
	 * Move Joints 1 & 2 by the radians in del_q.
	 */
	private static void moveJ1J2J3() {
		if ( del_q.get(2, 0) < 0) {
			K[2][0] = 0.5;
		} else {
			K[2][0] = 3;
		}
		if ( del_q.get(1, 0) < 0) {
			K[1][0] = 1.5;
		} else {
			K[1][0] = 4;
		}
		Thread JOINT_1 = new Thread(new PIDController(MOTOR[0], (int) Math.toDegrees(del_q.get(0, 0)), K[0], P[0], TIMEOUT));
 		Thread JOINT_2 = new Thread(new PIDController(MOTOR[1], (int) Math.toDegrees(del_q.get(1, 0)), K[1], P[1], TIMEOUT));
 		Thread JOINT_3 = new Thread(new PIDController(MOTOR[2], (int) Math.toDegrees(del_q.get(2, 0)), K[2], P[2], TIMEOUT));

 		stopAllMotors();
 		
		JOINT_1.start();
		JOINT_2.start();
		JOINT_3.start();

		boolean baseThread = false;
		boolean shoulderThread = false;
		boolean elbowThread = false;
		
		while(JOINT_1.isAlive() || JOINT_2.isAlive() || JOINT_3.isAlive()) {
			if (!JOINT_1.isAlive() && !baseThread) {
				new Thread(balanceBaseThread).start();
				baseThread = true;
			}
			if (!JOINT_2.isAlive() && !shoulderThread) {
				new Thread(balanceShoulderThread).start();
				shoulderThread = true;
			}
			if (!JOINT_3.isAlive() && !elbowThread) {
				new Thread(balanceShoulderThread).start();
				elbowThread = true;
			}
		}	// wait for joint movement threads to finish
		
		stopAllMotors();
		
		System.out.println("-----------------------------------------------------------------------------------------------------");
		
		new Thread(balanceAllThread).start();
	}
	
	private static void stopAllMotors() {
	
		stopThread = true;
		stopBaseThread = true;
		stopShoulderThread = true;
		stopElbowThread = true;
		Delay.msDelay(15);
		
	}
	
	/**
	 * Online Broyden's Method IBVS update of Jacobian
	 */
	private static void broydensUpdate() {
		final double alpha = 0.8;
		final double epsilon = 5;
		final double threshold = 10;
		final double step_i = 5;
		double step = step_i;
		Matrix Fx = new Matrix(4,1);
		Matrix target = new Matrix(4,1);
		Matrix target_new = new Matrix(4,1);
		Matrix numerator = new Matrix(4,1);
		double denominator = 1;
		Fx = getTrackerData();

		do {
			target_new = getTargetData();
			
			if (Math.abs(target_new.minus(target).normF()) > threshold) {
				step = step_i;
				Sound.beepSequenceUp();
			}
			target = target_new;

			// (un)comment to solve by way points
			/*
			System.out.print(JTest.getRowDimension());
			System.out.print(JTest.getColumnDimension());
			System.out.print(" ");
			System.out.print(mtrxDiv(target.minus(Fx), step).getRowDimension());
			System.out.print(mtrxDiv(target.minus(Fx), step).getColumnDimension());
			System.out.print(" ");
			*/
			//Button.waitForAnyPress();
			//del_q = JTest.solve(mtrxDiv(target.minus(Fx), step));
			del_q = JTest.inverse().times(mtrxDiv(target.minus(Fx), step));
			
			del_q = del_q.getMatrix(0, 2, 0, 0);

//					// (un)comment to solve by dyamic tracking
//					del_q = J.solve(target.minus(Fx));
			
			//moveJ1J2();
			moveJ1J2J3();
	        del_e = getTrackerData().minus(Fx);
	        
	        /*
	        System.out.print(JTest.getRowDimension());
			System.out.print(JTest.getColumnDimension());
			System.out.print(" ");
	        System.out.print(numerator.getRowDimension());
			System.out.print(numerator.getColumnDimension());
			System.out.print(" ");
			System.out.print(del_q.getRowDimension());
			System.out.print(del_q.getColumnDimension());
			System.out.print(" ");
			Button.waitForAnyPress();
			System.out.print((del_e.minus(JTest.times(del_q))).getRowDimension());
			System.out.print((del_e.minus(JTest.times(del_q))).getColumnDimension());
			System.out.print(" ");
			Button.waitForAnyPress();
			System.out.print((del_e.minus(JTest.times(del_q))).times(del_q.transpose()).getRowDimension());
			System.out.print((del_e.minus(JTest.times(del_q))).times(del_q.transpose()).getColumnDimension());
			System.out.print(" ");
			Button.waitForAnyPress();
			*/
	        
	        numerator = (del_e.minus(JTest.times(del_q))).times(del_q.transpose());
	        
	        /*
	        System.out.print(denominator);
			System.out.print(del_q.transpose().times(del_q).get(0, 0));
			Button.waitForAnyPress();
			*/
			
     		denominator = del_q.transpose().times(del_q).get(0, 0);
        		JTest = JTest.plus(mtrxDiv(numerator, denominator).times(alpha));
        		
    			Fx = getTrackerData();  			
			step--;

//			    		System.out.printf("POS ERROR:%f.4\n", Math.abs(target.normF() - Fx.normF()));
			
		// (un)comment to solve by way points
		} while (Math.abs(target.normF() - Fx.normF()) > epsilon && step > 0);			
			
//				// (un)comment to solve by dyamic tracking
//				} while (Math.abs(target.normF() - Fx.normF()) > epsilon);
	}

	/**
	 * Divide every element in Matrix num, by the value of denom
	 * 
	 * @param num numerator Matrix
	 * @param denom denominator value
	 * @return the element-wise quotient
	 */
	private static Matrix mtrxDiv(Matrix num, double denom) {
		Matrix temp = num.copy();
		for (int i = 0; i < temp.getRowDimension(); i++) {
			for (int j = 0; j < temp.getColumnDimension(); j++) {
				temp.set(i, j, num.get(i, j) / denom);
			}
		}
		return temp;
	}	
	
	/**
	 * Get u & v values of tracker
	 * 
	 * @return Matrix with tracker u and v values
	 */
	private static Matrix getTrackerData() {
		Matrix M = new Matrix(4,1);
		Delay.msDelay(2500);
	    	try {
	    		Thread.sleep(500);
	    			double u = tracker.x;
	    			double v = tracker.y;
//			    			System.out.printf("\nGOT TRACKER DATA\nU: %.1f V: %.1f\n\n", u, v);
	    			M.set(0, 0, u);
				M.set(1, 0, v);
	    	} catch (InterruptedException ex) {
	    		Thread.currentThread().interrupt();
	    	}
	    	
	    	M.set(2, 0, 1);
	    	M.set(3, 0, 1);
	    	return M;
	}
	
	/**
	 * Get u & v values of target
	 * 
	 * @return Matrix with target u and v values
	 */
	private static Matrix getTargetData() {
		Matrix M = new Matrix(4,1);
	    	try {
	    		Thread.sleep(500);
    				double u = tracker.targetx;
    				double v = tracker.targety;
//			    			System.out.printf("\nGOT TARGET DATA:\nU: %.1f V: %.1f\n\n", u, v);
	    			M.set(0, 0, u);
				M.set(1, 0, v);
	    	} catch (InterruptedException ex) {
	    		Thread.currentThread().interrupt();
	    	}
	    	
	    	M.set(2, 0, 1);
	    	M.set(3, 0, 1);
	    	return M;
	}
	
	public static void balance() {
		base.resetTachoCount();
		shoulder.resetTachoCount();
		elbow.resetTachoCount();
		
		new Thread(balanceShoulderThread).start();
		new Thread(balanceElbowThread).start();
		new Thread(balanceBaseThread).start();
		
		stopThread = false;
		
		while (!stopThread) {
			
		}
		
		stopThread = true;
		stopBaseThread = true;
		stopShoulderThread = true;
		stopElbowThread = true;
		
		return;
	}
	
	public static void balanceBase() {
		base.resetTachoCount();
		
		base.forward();
		
		int powerMultiplier = -3;
		
		stopBaseThread = false;
		
		while (!stopBaseThread) {
			
			//System.out.println("0(" + base.getTachoCount() + ")");
			
			//smoother more spring like, 7 is a good number, joints start to overcompinsate if higher and won't fully return if lower
			if (base.getTachoCount() >= 1 || base.getTachoCount() <= 1) {
				base.setPower(base.getTachoCount() * powerMultiplier);
			} else if (base.getTachoCount() >= 0) {
				base.setPower(-5);
			} else {
				base.setPower(5);
			}
		}
		
		return;
	}
	
	public static void balanceShoulder() {
		shoulder.resetTachoCount();
		
		shoulder.forward();
		
		int powerMultiplier = -6;
		
		stopShoulderThread = false;
		
		while (!stopShoulderThread) {
			
			//System.out.println("     1(" + shoulder.getTachoCount() + ")");
			
			//smoother more spring like, 7 is a good number, joints start to overcompinsate if higher and won't fully return if lower
			if (shoulder.getTachoCount() >= 1 || shoulder.getTachoCount() <= 1) {
				shoulder.setPower(shoulder.getTachoCount() * powerMultiplier);
			} else if (shoulder.getTachoCount() >= 0) {
				shoulder.setPower(-5);
			} else {
				shoulder.setPower(5);
			}
			
		}
		
		return;
	}
	
	public static void balanceElbow() {
		elbow.resetTachoCount();
		
		elbow.forward();
		
		int powerMultiplier = -6;
		
		stopElbowThread = false;
		
		while (!stopElbowThread) {
			
			//System.out.println("          2(" + elbow.getTachoCount() + ")");
			
			//smoother more spring like, 7 is a good number, joints start to overcompinsate if higher and won't fully return if lower
			if (elbow.getTachoCount() >= 1 || elbow.getTachoCount() <= 1) {
				elbow.setPower(elbow.getTachoCount() * powerMultiplier);
			} else if (elbow.getTachoCount() >= 0) {
				elbow.setPower(-5);
			} else {
				elbow.setPower(5);
			}
			
		}
		
		return;
	}
}
