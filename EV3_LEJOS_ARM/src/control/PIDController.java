package control;


import lejos.hardware.Sound;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Stopwatch;

/**
 * A PID Controller thread class for controlling LEGO UnregulatedMotors.
 * @author nicholasmayne
 *
 */
public class PIDController implements Runnable{
	private final UnregulatedMotor m;	// the motor
	private final Stopwatch timer;		// the timer
	private final double[] K;			// the K gain values in an array [p, i, d]
	private final int sampleRate = 10;	// enforced minimum sample rate, 
	private final int sp;				// set point: the target to move to 
	private final int pm;				// the power maximum 0 ≤ pm ≤100
	private final int to;				// the timeout value in milliseconds
	private double integral;				// the integral error
	private double derivative;			// the derivative error
	private double delta_t;				// delta t
	private double elapsed_t;			// elapsed t
	private int error;					// error
	private int prevError;				// error from previous sample
	private int pv;						// process variable: tachometer reading
	private int power;					// power to motor
	
	
	/**
	 * Initialize a PID controller.
	 * 
	 * @param m Initialized LEGO motor.
	 * @param sp Set point of motor rotation.
	 * @param K Array of Kp, Ki, Kd, gain terms.
	 * @param pm Power maximum for this PID task. The motor will not be driven with more power than this value, 0 ≤ pm ≤ 100.
	 * @param to Timeout value in milliseconds. The PID controller loop will terminate after this many milliseconds.
	 */
	public PIDController(UnregulatedMotor m, int sp, double[] K, int pm, int to) {
		this.timer = new Stopwatch();
		this.m = m;
		this.sp = sp;
		this.K = K;
		this.pm = pm;
		this.to = to;
	}
	
	/**
	 * Run the PID controller with its initialized parameters.
	 */
	public void run() {
		error = sp;			// error
		prevError = error;	// error from previous sample
		derivative = 0;		// the derivative error
		integral = 0;		// the integral error
		delta_t = 0;			// delta t
		elapsed_t = 0;		// elapsed t
		power = 0;			// power to motor
		
		m.resetTachoCount();
		timer.reset();

		while (error != 0 || prevError != 0) {
			// calculate and set motor power
			integral = integral + (error * delta_t / 100);
			derivative = (error - prevError) / (delta_t / 100);
			power = (int) Math.abs((K[0] * error) + (K[1] * integral) + (K[2] * derivative));
			m.setPower(isPos(error) * (Math.min(pm, power)));
			
			while (timer.elapsed() < sampleRate) {}		// enforce minimum sample rate
			
			delta_t = timer.elapsed();					// get delta t and error
			elapsed_t = elapsed_t + delta_t;
			timer.reset();
			prevError = error;
			pv = m.getTachoCount();
			error = sp - pv;
						
			if (elapsed_t > to) {
//				System.out.print("TIMEOUT: " + String.valueOf(sp) + "\n");
				Sound.buzz();
				break;}
		}
		m.setPower(0);
		if (error > 0) {
			Sound.beepSequence();
		}
//		System.out.print("E: " + error + "\n\n");
	}
	
	/**
	 * Return the sign of a double.
	 * 
	 * @param d the double to evaluate for its sign.
	 * @return the sign of a double.
	 */
	private int isPos(double d) {
		if (d < 0) {return -1;}
		return 1;}
}