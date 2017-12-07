package jcontroller;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;
import Jama.Matrix;
//import lejos.utility.Matrix;

import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import control.Move;


public class DirMotControl {
	public static UnregulatedMotor LR = new UnregulatedMotor(MotorPort.A);
    public static UnregulatedMotor FB = new UnregulatedMotor(MotorPort.B);
    public static UnregulatedMotor UD = new UnregulatedMotor(MotorPort.C);
    public static UnregulatedMotor base;
    public static UnregulatedMotor L1;
    public static UnregulatedMotor L2;
    public static UnregulatedMotor [] controller_motors = {LR, FB, UD};
    
    public static double l1 = JacCalc.l1;
    public static double l2 = JacCalc.l2;
    public static double[] l1_constraint = {0,180};
    public static double[] l2_constraint = {-270,0};
    public static double base_height = JacCalc.base_height;
    public static double[] offset = {8.5,0, 11.2};
    public static double[][] ratio_vals = {{2472/360,0,0},{0,122/90,0},{0,0,1}};
    public static Matrix ratio = new Matrix(ratio_vals);
    public static void main(String[] args) {	

    	//DirectControl();
    	IKControl();
	}
    
//REMOTE CONTROLLER METHOD 2---------------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * IKControl has each joystick motor controlling a degree of motion in the x,y, and z axis.
     * The robot performs IK calculations to determine the corresponding angles to get the end effector in the desired position.
     * For this function, the robot must begin at the neutral rest position defined in our report. The end effector
     * at this position will be treated as coordinate (0,0,0).
     */
    public static void IKControl(){
    	
    	String[] names = {"Happy", "qwert"};
	    Brick[] bricks = new Brick[names.length];
	   
	    
	    try {
	    	bricks[0] = BrickFinder.getLocal();
	        for(int i = 1; i < bricks.length; i++)
	        {
	            System.out.println("Connect " + names[i]);
	            bricks[i] = new RemoteEV3(BrickFinder.find(names[i])[0].getIPAddress());
	        }
	        GraphicsLCD[] glcds = new GraphicsLCD[bricks.length];
	        for(int i = 0; i < bricks.length; i++)
	            glcds[i] = bricks[i].getGraphicsLCD();
	        
	        int dispwidth = glcds[1].getWidth();
	        int dispheight = glcds[1].getHeight();
	        LR.resetTachoCount();
	        FB.resetTachoCount();
	        UD.resetTachoCount();
//	        try{
//	        base = new UnregulatedMotor(bricks[1].getPort("A"));
//	        L1 = new UnregulatedMotor(bricks[1].getPort("B"));
//	        L2 = new UnregulatedMotor(bricks[1].getPort("C"));
//
//	        base.resetTachoCount(); L1.resetTachoCount(); L2.resetTachoCount();
//
//	        UnregulatedMotor [] arm_motors = {base, L1, L2}; 
//	        Move.getInstance(arm_motors);
//	        Move.setOrigin();
	        
	        glcds[1].clear();
	        
	        //Calibrate arm to init position
	        double[][] eff_coord = {{offset[0]}, {offset[1]}, {offset[2]}};//Change to initial position
	        double[][] current_pos = {{offset[0]}, {offset[1]}, {offset[2]}};//change to initial position
	        double[][] theta = {{0},{-45},{135}};// Change to initial theta
	        Matrix dT = new Matrix(3,1);
	       
	        while(Button.ENTER.isUp())
	        {		        	
	        	UpdateEffector(eff_coord);// get new target coordinate
	        	glcds[1].drawString("Target:[x,y,z]",dispwidth / 2,dispheight / 2,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	//double error = Math.sqrt(Math.pow(eff_coord[0][0]-current_pos[0][0], 2)+ Math.pow(eff_coord[1][0]-current_pos[1][0], 2) + Math.pow(eff_coord[2][0]-current_pos[2][0], 2));
	        	
	        	dT = ratio.times(CalculateAngle(eff_coord, current_pos, theta));
	        	//Move.J123(dT.times(Math.PI/180), true, 2000);
	        	glcds[1].drawString(String.format("%.1f", eff_coord[0][0]),dispwidth * 1/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	glcds[1].drawString(String.format("%.1f", eff_coord[1][0]),dispwidth * 2/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	glcds[1].drawString(String.format("%.1f", eff_coord[2][0]),dispwidth * 3/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	
	        	glcds[1].drawString(String.format("%.1f", theta[0][0]),dispwidth * 1/4, dispheight,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	glcds[1].drawString(String.format("%.1f", theta[1][0]),dispwidth * 2/4, dispheight,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	glcds[1].drawString(String.format("%.1f", theta[2][0]),dispwidth * 3/4, dispheight,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	        	
	        	Delay.msDelay(100);
	        	glcds[1].clear();
	        	
	        }

	    }
	    catch (RemoteException e)
	    {
	        System.out.println("Got exception " + e);
	    }
	    catch (NotBoundException e)
	    {
	        System.out.println("Got exception " + e);
	    }
	    catch (MalformedURLException e)
	    {
	        System.out.println("Got exception " + e);
	    }
	    catch(Exception e){
	        System.out.println("Got exception " + e);
	    }
//	    finally {
//	    	base.close();
//	    	L1.close();
//	    	L2.close();  
//	    }
    	
    }
    
    public static Matrix CalculateAngle(double[][] eff_coord, double[][]current_pos, double[][] theta){
    	Matrix dT_Matrix = new Matrix(3,1);
    	double[] start_angles = {0, 0, 0};
    	start_angles[0] = theta[0][0];start_angles[1] = theta[1][0];start_angles[2] = theta[2][0];
    	
    	Matrix dT_return = new Matrix(3,1);
    	
    	double error = Math.sqrt(Math.pow(eff_coord[0][0]-current_pos[0][0], 2)+ Math.pow(eff_coord[1][0]-current_pos[1][0], 2) + Math.pow(eff_coord[2][0]-current_pos[2][0], 2));
    	
    	Matrix jacobian =  JacCalc.getJacobian(theta);
    	//jacobian.print(System.out);
    	Matrix invJacobian = jacobian.inverse();
    	
    	for(int i=0;i<200;i++){ // if effector target has moved, perform ik calculations and move
    		if(error > 0.05){
	    		double[][]d_pos = {{eff_coord[0][0]-current_pos[0][0]},
				   		   		   {eff_coord[1][0]-current_pos[1][0]},
				                   {eff_coord[2][0]-current_pos[2][0]}};
	    		Matrix dP_Matrix = new Matrix(d_pos);
	    		dT_Matrix = invJacobian.times(dP_Matrix);
	        	
	    		Matrix T = new Matrix(theta);
	    		//System.out.printf("%.1f %.1f %.1f\n",dT_Matrix.getArray()[0][0],dT_Matrix.getArray()[1][0],dT_Matrix.getArray()[2][0]);
	        	T.plusEquals(dT_Matrix);
	        	theta[0][0] = T.getArray()[0][0]; theta[1][0] = T.getArray()[1][0]; theta[2][0] = T.getArray()[2][0];
	        	UpdatePosition(current_pos, theta);
	        	error = Math.sqrt(Math.pow(eff_coord[0][0]-current_pos[0][0], 2)+ Math.pow(eff_coord[1][0]-current_pos[1][0], 2) + Math.pow(eff_coord[2][0]-current_pos[2][0], 2));
	        	//System.out.printf("%.2f", error);
    		}
    	}
    	dT_return.set(0, 0, (theta[0][0]-start_angles[0])%360);
    	dT_return.set(1, 0, (theta[1][0]-start_angles[1])%360);
    	dT_return.set(2, 0, (theta[2][0]-start_angles[2])%360);
    	//System.out.printf("%.1f %.1f %.1f\n",dT_return.getArray()[0][0],dT_return.getArray()[1][0],dT_return.getArray()[2][0]);
    	System.out.printf("%.1f %.1f %.1f\n %.1f %.1f %.1f\n %.1f %.1f %.1f\n",
    			jacobian.getArray()[0][0],jacobian.getArray()[0][1],jacobian.getArray()[0][2],
    			jacobian.getArray()[1][0],jacobian.getArray()[1][1],jacobian.getArray()[1][2],
    			jacobian.getArray()[2][0],jacobian.getArray()[2][1],jacobian.getArray()[2][2]);
    	return dT_return;
    }
    
    public static void angleInWorkSpace(double[][] theta, double[][]eff_coord, double[][] current_pos){
    	if(theta[1][0]<l1_constraint[0]){
    		theta[1][0] = l1_constraint[0];
    	}else if(theta[1][0]>l1_constraint[1]){
    		theta[1][0] = l1_constraint[1];
    	}
    	if(theta[2][0]<l2_constraint[0]){
    		theta[2][0] = l2_constraint[0];
    	}else if(theta[2][0]>l2_constraint[1]){
    		theta[2][0] = l2_constraint[1];
    	}
    	UpdatePosition(current_pos,theta);
    	double error = Math.sqrt(Math.pow(eff_coord[0][0]-current_pos[0][0], 2)+ Math.pow(eff_coord[1][0]-current_pos[1][0], 2) + Math.pow(eff_coord[2][0]-current_pos[2][0], 2));
    	//CCD adjustments	
    	
    }
    //Function to calculate new effector position
    public static void UpdatePosition(double[][] position, double [][] theta){
    	position[0][0] = Math.cos(Math.toRadians(theta[0][0]))*(l1* Math.cos(Math.toRadians(theta[1][0])) + l2* Math.cos(Math.toRadians(theta[1][0]+theta[2][0])));
    	position[1][0] = Math.sin(Math.toRadians(theta[0][0]))*(l1* Math.cos(Math.toRadians(theta[1][0])) + l2* Math.cos(Math.toRadians(theta[1][0]+theta[2][0])));
    	position[2][0] = l1* Math.sin(Math.toRadians(theta[1][0])) + l2* Math.sin(Math.toRadians(theta[1][0]+theta[2][0]));
    	
    }
    //Function to update target values based on joystick controller
    public static void UpdateEffector(double[][]coord){
    	//double [][] new_coord = {{coord[0][0]},{coord[1][0]},{coord[2][0]}};
    	//Change y-coordinate
    	if(LR.getTachoCount()<-15||LR.getTachoCount()>15){
    		coord[1][0] += LR.getTachoCount()*-0.05;
    	}
    	//Change x-coordinate
    	if(FB.getTachoCount()>15||FB.getTachoCount()<-15){
    		coord[0][0] += FB.getTachoCount()*-0.05;
    	} 
    	//Change z-coordinate
    	if(UD.getTachoCount()>15||UD.getTachoCount()<-15){
    		coord[2][0] += UD.getTachoCount()*0.05;
    	}
    	/*
    	if(isInWorkSpace(new_coord)){
    		return new_coord;
    	} else {
    		return coord;
    	}*/
    	coord = isInWorkSpace(coord);
    	System.out.printf("%.1f %.1f %.1f\n",coord[0][0],coord[1][0],coord[2][0]);
    }
    
    
    
    //Function to validate that the point does not exceed the outer bound of the arm's reach
    public static double[][] isInWorkSpace(double[][] point){
    	double norm = Math.sqrt(Math.pow(point[0][0], 2)+ Math.pow(point[1][0], 2) + Math.pow(point[2][0], 2));
    	double outer_radius = l1 + l2;// arm full extension
    	double inner_radius = Math.sqrt(Math.pow(offset[0], 2)+ Math.pow(offset[1], 2) + Math.pow(offset[2], 2)); //offset at initial
    	if(norm > outer_radius){
    		//find projection of point along outer bound
    		point[0][0] = point[0][0]*(outer_radius/norm);
    		point[1][0] = point[1][0]*(outer_radius/norm);
    		point[2][0] = point[2][0]*(outer_radius/norm);
    		return point;
    	} else if(norm <= inner_radius){
    		//find projection of point along inner bound
    		point[0][0] = point[0][0]*(inner_radius/norm);
    		point[1][0] = point[1][0]*(inner_radius/norm);
    		point[2][0] = point[2][0]*(inner_radius/norm);
    		return point;
    	} else {
    		return point;
    	}
    }
    
    

//REMOTE CONTROLLER METHOD 1---------------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * DirectControl connects each joystick motor to an associated motor in the arm, 1 to 1.  
     * Power fed to the arm motors is directly proportional to the angles of the joysticks.
     */
	public static void DirectControl(){

		String[] names = {"Happy", "qwert"};
	    Brick[] bricks = new Brick[names.length];
	    try {
	    	bricks[0] = BrickFinder.getLocal();
	        for(int i = 1; i < bricks.length; i++)
	        {
	            System.out.println("Connect " + names[i]);
	            bricks[i] = new RemoteEV3(BrickFinder.find(names[i])[0].getIPAddress());
	        }
	        GraphicsLCD[] glcds = new GraphicsLCD[bricks.length];
	        for(int i = 0; i < bricks.length; i++)
	            glcds[i] = bricks[i].getGraphicsLCD();
	        
	        int dispwidth = glcds[1].getWidth();
	        int dispheight = glcds[1].getHeight();
	        LR.resetTachoCount();
	        FB.resetTachoCount();
	        UD.resetTachoCount();
	        try{
	        	
	        	UnregulatedMotor base = new UnregulatedMotor(bricks[1].getPort("A"));
		        UnregulatedMotor L1 = new UnregulatedMotor(bricks[1].getPort("B"));
		        UnregulatedMotor L2 = new UnregulatedMotor(bricks[1].getPort("C"));
		        base.setPower(50);
		        L1.setPower(50);
		        L2.setPower(50);
		        glcds[1].clear();
		        while(Button.ENTER.isUp())
		        {		        	
		        	
		        	glcds[1].drawString("Controller Readings:[base, l1, l2]",dispwidth / 2,dispheight / 2,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Integer.toString(LR.getTachoCount()),dispwidth * 1/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Integer.toString(FB.getTachoCount()),dispwidth * 2/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Integer.toString(UD.getTachoCount()),dispwidth * 3/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	//Delay.msDelay(200);
		        	
		        	glcds[1].clear();
		        	
		        	if(LR.getTachoCount()<-15){
		        		base.setPower((int)((LR.getTachoCount()*1.5) % 100));
		        		base.backward();
		        	} else if (LR.getTachoCount()>15){
		        		base.setPower((int)((LR.getTachoCount()*1.5) % 100));
		        		base.backward();
		        	} else {
		        		base.stop();
		        	}
		        	
		        	if(FB.getTachoCount()>15){
		        		L1.setPower(Math.abs((int)((FB.getTachoCount()*1.5) % 100)));
		        		L1.forward();
		        	} else if (FB.getTachoCount()<-15){
		        		L1.setPower(Math.abs((int)((FB.getTachoCount()*1.5) % 100)));
		        		L1.backward();
		        	} else {
		        		L1.stop();
		        	}
		        	
		        	if(UD.getTachoCount()>15){
		        		L2.setPower(Math.abs((int)((UD.getTachoCount()*1.5) % 100)));
		        		L2.forward();
		        	} else if (UD.getTachoCount()<-15){
		        		L2.setPower(Math.abs((int)((UD.getTachoCount()*1.5) % 100)));
		        		L2.backward();
		        	} else {
		        		L2.stop();
		        	}
		        }
		        base.close();
		        L1.close();
		        L2.close();  
	        } 
	        catch(Exception e){
	        	System.out.println("Cannot get motor");
	        } 
	    }
	    catch (Exception e)
	    {
	        System.out.println("Got exception " + e);
	    }
	    
	}
}
