package control;

import lejos.hardware.motor.UnregulatedMotor;

public class Balance implements Runnable{
	private UnregulatedMotor motor;
	private int powerMultiplier;
	public Boolean stopBalance;
	
	public Balance(UnregulatedMotor motor, int powerMultiplier) {
		this.motor = motor;
		this.powerMultiplier = powerMultiplier;
		stopBalance = false;
	}
	
	public void run() {
		
		motor.resetTachoCount();
		motor.forward();

		while (!stopBalance) {	
			if (motor.getTachoCount() >= 1 || motor.getTachoCount() <= 1) {
				motor.setPower(motor.getTachoCount() * powerMultiplier);
			} else if (motor.getTachoCount() >= 0) {
				motor.setPower(-5);
			} else {
				motor.setPower(5);
			}
		}
//		System.out.println("Stopped A Balance Joint Thread:\n" + motor.toString() + "\n");
		return;
	}	
}
