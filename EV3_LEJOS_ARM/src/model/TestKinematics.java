package model;

import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import vision.TrackerReader;
import lejos.hardware.motor.UnregulatedMotor;
import Jama.Matrix;


public class TestKinematics {
	public static TrackerReader tracker;
	public static boolean stopThread = false;
	
	public static UnregulatedMotor base = new UnregulatedMotor(MotorPort.A);
	public static UnregulatedMotor shoulder = new UnregulatedMotor(MotorPort.B); //The base joint
	public static UnregulatedMotor elbow = new UnregulatedMotor(MotorPort.C);//elbow joint

	public static void main(String args[]){
		
		//start self balancing thread
		Runnable r = new Runnable() {
	         public void run() {
	             balance();
	         }
	    };

	    new Thread(r).start();
		
		tracker = new TrackerReader();
		tracker.start();
		//UnregulatedMotor base = new UnregulatedMotor(MotorPort.B); //The base joint
		//UnregulatedMotor elbow = new UnregulatedMotor(MotorPort.C);//elbow joint
		
		shoulder.resetTachoCount();
		elbow.resetTachoCount();
		
		//double	l1 = 6.5f,					//base length in cm
		//        l2 = 8.0f;					//elbow length in cm

		double x = 0f;						//target x
        double y = 14f;					//target y should be written as -y


        int t1 = 10; 					//current angle of base joint
        int t2 = 10;
        //int oldt1=base.getTachoCount();
        //int oldt2=base.getTachoCount();
        double rt1 = Math.toRadians(t1);
        double rt2 = Math.toRadians(t2);

        double x1;
        double y1;
        double[][] Jacob = new double[2][2];
        double[][] rt = new double [2][1];
        double tolerance = 10000;
        while(tracker.x == 0 && tracker.y == 0) {
    		//wait for click
    	}
        double oldx = tracker.x;
        double oldy = tracker.y;
        //oldx = tracker.x;
        //oldy = tracker.y;
        Delay.msDelay(300);
        int[] cangles = new int[2]; 
        double beforeX = tracker.getCurrentXY()[0];
        double beforeY = tracker.getCurrentXY()[1];
        stopThread = true;
        cangles = move(45, 0, 0, 0,0,0,shoulder,elbow);
        new Thread(r).start();
        //Jacob[0][0]= (tracker.x - oldx)/Math.toRadians(cangles[0]);
        //Jacob[1][0]= (tracker.y - oldy)/Math.toRadians(cangles[0]);
        Delay.msDelay(1000);
        System.out.println(cangles[0] + " shoulder " + tracker.x + " old: " +oldx);
        System.out.println( tracker.y + " old: " +oldy);
        //Button.waitForAnyPress();
        Jacob[0][0]= (tracker.x - beforeX)/Math.toRadians(cangles[0]);
        Jacob[1][0]= (tracker.y - beforeY)/Math.toRadians(cangles[0]);
        Delay.msDelay(1000);
        oldx = tracker.x;
        oldy = tracker.y;
        stopThread = true;
        cangles= move(cangles[0], 45,cangles[0], 0,0,0,shoulder,elbow);
        new Thread(r).start();
        Delay.msDelay(1000); //100ms delay
        System.out.println(cangles[1] + " elbow " + tracker.x + " old: " +oldx);
        System.out.println( tracker.y + " old: " +oldy);
        //Button.waitForAnyPress();
        Jacob[0][1]= (tracker.x - oldx)/Math.toRadians(cangles[1]);
        Jacob[1][1]= (tracker.y - oldy)/Math.toRadians(cangles[1]);
        

        /** */
        Matrix Jk1 = new Matrix(Jacob);		//J0
        //Jk1 = Jk1.transpose();
        double[][] dte = {{1},{1}};
        Matrix mdE = new Matrix(dte);
        double[][] dtq = {{Math.toRadians(45)},{Math.toRadians(45)}};
        Matrix mdQ = new Matrix(dtq);
        double alpha = 0.1;
       // double mousex = 0;
       // double mousey = 0;
       System.out.println(Math.round(Jk1.get(0,0)) + "  " + Math.round(Jk1.get(0,1)) + " \n" + Math.round(Jk1.get(1,0)) + " " +Math.round(Jk1.get(1,1)) + " \n");
        //int count = 0;v
       //Button.waitForAnyPress();
        
        double[][] s = {{0},{0}};
        Matrix mS = new Matrix(s);
        
        double[][] yChange = {{0},{0}};
        Matrix yMChange = new Matrix(yChange);
        
        Matrix oldQ = new Matrix(dtq);
        
        oldx = 0;
        oldy = 0;
        
        oldx = tracker.x;
        oldy = tracker.y;
        
        Button.waitForAnyPress();
        
        while (true) {
            /*
        	while(tracker.targetx < 0 && tracker.targety < 0) {
        		//wait for click
        		oldx = tracker.x;
                oldy = tracker.y;
        	}*/
        	
        	
            try {
                Thread.sleep(1000);         //1000 milliseconds is one second.
            } catch(InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            
            //Button.waitForAnyPress();
            
            //System.out.println(mdQ.transpose().times(mdQ));
            /*
            dtq[0][0] = base.getTachoCount();
	        dtq[1][0] = elbow.getTachoCount();
		    if(dtq[0][0] == 0){
		    	dtq[0][0]=1;
		    }tracker.movex - tracker.x
		    if(dtq[1][0] == 0){
		    	dtq[1][0]=1;
		    }
		    mdQ = new Matrix(dtq);
		    */
		    double[][] from = {{tracker.x},{tracker.y}};
		    double[][] to = {{tracker.targetx},{ tracker.targety}};
		    Matrix f = new Matrix(from);
		    Matrix t = new Matrix(to);
		    dte[0][0] = tracker.targetx - oldx; //yold[0] = tracker.x
	        dte[1][0] = tracker.targety - oldy; //yold[1] = tracker.y
		    if(dte[0][0] == 0){
		    	dte[0][0]=1;
		    }
		    if(dte[1][0] == 0){
		    	dte[1][0]=1;
		    }
		    mdE =t.plus(f); //pos - yold
		    //System.out.println("MDE: " +mdE.get(0, 0) + " \n" + mdE.get(1, 0));
		    System.out.println("xm,ym: " +tracker.targetx + " \n" + tracker.targety);
		    //Button.waitForAnyPress();
		    System.out.println("x,y: " +tracker.x + " \n" + tracker.y);
		    //Button.waitForAnyPress();
		    System.out.println("mde:  "+(mdE.get(0,0)) + ", " + (mdE.get(1,0)) );
		    //Button.waitForAnyPress();
		    Matrix jI = Jk1.inverse();
		    System.out.println((jI.get(0,0)) + "\n" + (jI.get(0,1)) + " \n" );
		    //Button.waitForAnyPress();
		    System.out.println( (jI.get(1,0)) + "\n" +(jI.get(1,1))+ " \n");
		    //Button.waitForAnyPress();
		    mS = (jI).times(mdE); //s
		    System.out.println("MS: " +mS.get(0, 0) + " \n" + mS.get(1, 0));
		    //Button.waitForAnyPress();
		    System.out.println("mdQ: " +mdQ.get(0, 0) + " \n" + mdQ.get(1, 0));
		   
		    oldQ = mdQ;
		    //Button.waitForAnyPress();
		    //mdQ = oldQ.plus(mS); //theta
		    mdQ = oldQ.minus(mS); //theta, need to mijnus because tracker returns negative angle for some reason now but it works so whatever
		    System.out.println("mdQ: " +mdQ.get(0, 0) + " \n" + mdQ.get(1, 0));
		    //Button.waitForAnyPress();
		   // System.out.println("before move");
		    stopThread = true;
		    move((int)(Math.toDegrees(mdQ.get(0,0))), (int)(Math.toDegrees(mdQ.get(1,0))), (int)Math.toDegrees(oldQ.get(0,0)), (int)Math.toDegrees(oldQ.get(1,0)),0,0,shoulder,elbow);
		    new Thread(r).start();
		    //Button.waitForAnyPress();
		   // System.out.println("after move");
		    Delay.msDelay(1000);
		    yChange[0][0] = tracker.x - oldx;		//new pos - old pos //ynew - yold
		    yChange[1][0] = tracker.y - oldy;
		    
		    yMChange = new Matrix(yChange);
            oldx = tracker.x;
            oldy = tracker.y; 
            //Button.waitForAnyPress();
            Jk1 = Jk1.plus(((yMChange.minus(Jk1.times(mdQ))).times(mdQ.transpose()).times(1/(mdQ.transpose().times(mdQ)).get(0,0))).times(alpha));
            //System.out.println(Jk1.get(0,0) + " " + Jk1.get(0,1) + " " + Jk1.get(1,0) + " " +Jk1.get(1,1) + " ");
           // System.out.println("x,y " + oldx + "," + oldy  );
           // System.out.println("Target: " + tracker.movex + "," + tracker.movey);
            if(Math.abs(tracker.x - tracker.targetx) < 15 && Math.abs(tracker.y - tracker.targety)<15 ){
            	System.out.println("done");
            	break;
            }
            
        }
		
	}
	
	public static int[] move(	int target1,
						int target2,
						int theta1,
						int theta2,
						double l1,
						double l2,
						UnregulatedMotor base,
						UnregulatedMotor elbow){
		System.out.println("err: " + (target1-theta1) + "," + (target2-theta2) );
		//System.out.println("t1 " + target1 +" theta1 " + theta1);
		//System.out.println("t2 " + target2 +" theta2 " + theta2);
		/*if(target1 < -180){
			target1 = target1 %360;
		}
		if(target1 > 180){
			target1 = (360-target1);
		}
		if(target2<-180){
			target2 = target2 % 360;
		}
		if(target2>180){
			target2 = (360 - target2);
		}
*/
		base.resetTachoCount();
		elbow.resetTachoCount();

		double rtarget1, rtarget2;
		int error = target1 - theta1; 		//distance to target angle for base
	//	System.out.println("t1 " + target1 +" theta1 " + theta1);
	//	System.out.println("t2 " + target2 +" theta2 " + theta2);
		int perror = error; 				//error of previous loop for base
		int error2 = target2 - theta2;		//distance to target angle for elbow
		int perror2 = error2;				//error of previous loop for elbow	
		
		double x = 0;
		double y = 0;
		
		int power = 0;
		int power2 = 0;
		double kp = 1.2f;
		double kd = 0.000000001f;
		long current = System.nanoTime();
		long prev = current - 1;
		//long diff = 0;
		base.setPower(0);
		elbow.setPower(0);
		base.forward();
		elbow.forward();
		while((Math.abs(error) + Math.abs(perror))/2 > 8 || (Math.abs(error2) + Math.abs(perror2))/2 > 8){
		//Loop until the end effector is at the position
		//if(current-prev)
			//System.out.println("Indicate");
			theta1 += base.getTachoCount();
			base.resetTachoCount();
			theta2 += elbow.getTachoCount();
			elbow.resetTachoCount();
			/*if(Math.abs(theta1) >=130){
				break;
			}
			if(Math.abs(theta2) >=160){
				break;
			}
			*/
			//set power
			power = (int) Math.round(error*kp + kd*(error-perror)*1000000000/(current-prev));
			power2 = (int) Math.round(error2*kp + kd*(error2-perror2)*1000000000/(current-prev));
			
			if(power * 3 > 100){						//for base
				power = 100;
			}
			else if(power * 3 < -100){
				power = -100;
			}
			base.setPower(power *  3);
			
			if(power2 * 3 > 100){						//for elbow
				power2 = 100;
			}
			else if(power2 * 3 < -100){
				power2 = -100;
			}
			elbow.setPower(power2 * 3);
			
			//Update time and error
			prev = current;
			current = System.nanoTime();
			perror = error;
			error = target1 - theta1;	
			perror2 = error2;
			error2 = target2 - theta2;
			//System.out.println("E1: " + error + " E2: " + error2);
		}
		
		//x = l2*Math.cos(2*theta2) + l1*Math.cos(theta1);
		//y = l2*Math.sin(2*theta2) + l1*Math.sin(theta1);
		//rtarget1 = Math.PI*target1/180;
		//rtarget2 = Math.PI*target2/180;
		//x = Math.cos(rtarget1)*l1 + Math.cos(rtarget1 + rtarget2)*l2;
		//y = -(Math.sin(rtarget1)*l1 + Math.sin(rtarget1 + rtarget2)*l2);
		//System.out.printf("( %.2f, %.2f )" ,x,y);
		//base.stop();
		//elbow.stop();
		//Button.waitForAnyPress();
		int[] ret = {theta1, theta2};
		return ret;
	}
	
	public static void balance() {
		base.resetTachoCount();
		shoulder.resetTachoCount();
		elbow.resetTachoCount();
		
		base.forward();
		shoulder.forward();
		elbow.forward();
		
		int powerMultiplier = -7;
		
		while (!stopThread) {
			
			//smoother more spring like, 7 is a good number, joints start to overcompinsate if higher and won't fully return if lower
			if (base.getTachoCount() != 0) {
				base.setPower(base.getTachoCount() * powerMultiplier);
			} else {
				base.setPower(5);
			}
			
			if (shoulder.getTachoCount() != 0) {
				shoulder.setPower(shoulder.getTachoCount() * powerMultiplier);
			} else {
				shoulder.setPower(5);
			}
			
			if (elbow.getTachoCount() != 0) {
				elbow.setPower(elbow.getTachoCount() * powerMultiplier);
			} else {
				elbow.setPower(5);
			}
			
			
		}
		stopThread = false;
		return;
	}
}
