package view;
import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import model.Kinematics;

public class Main {
    public static Kinematics kinematics;

    /**
     * Run Kinematics for three motors
     * @param args
     */
    public static void main (String[] args) {
    		kinematics = new Kinematics(MotorPort.A, MotorPort.B, MotorPort.C);
    		final int[] P = {70, 50, 30}; // Motor Power Max

    		
    		int[] SP1 = {1, 1, 40};
    		kinematics.Move(SP1, P);
    		int[] SP2 = {1, 80, 1};
    		kinematics.Move(SP2, P);
    		int[] SP3 = {90, 1, 1};
    		kinematics.Move(SP3, P);
    		
    		
    		Delay.msDelay(1500);
    		int[] SP4 = {-90, -80, -30};
    		final int[] P4 = {100, 20, 20}; // Motor Power Max

    		kinematics.Move(SP4, P4);
    		Button.waitForAnyPress();
    }
}