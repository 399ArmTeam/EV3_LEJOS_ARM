package jcontroller;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;
//import Jama.Matrix;
import lejos.utility.Matrix;

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
    public static double base_height = JacCalc.base_height;
    public static double[] offset = {8.5,0, 11.2};
    
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
	        	base = new UnregulatedMotor(bricks[1].getPort("A"));
		        L1 = new UnregulatedMotor(bricks[1].getPort("B"));
		        L2 = new UnregulatedMotor(bricks[1].getPort("C"));
		        
		        base.resetTachoCount(); L1.resetTachoCount(); L2.resetTachoCount();
		        
		        UnregulatedMotor [] arm_motors = {base, L1, L2}; 
		        Move.getInstance(arm_motors);
		        Move.setOrigin();
		        
		        glcds[1].clear();
		        
		        //Calibrate arm to init position
		        double[][] eff_coord = {{offset[0]}, {offset[1]}, {offset[2]}};//Change to initial position
		        double[][] current_pos = {{offset[0]}, {offset[1]}, {offset[2]}};//change to initial position
		        double[][] theta = {{15},{15},{15}};// Change to initial theta
		        
		        Matrix dP_Matrix = new Matrix(3,1);
		        while(Button.ENTER.isUp())
		        {		        	
		        	UpdateEffector(eff_coord);// get new target coordinate
		        	glcds[1].drawString("Target:[x,y,z]",dispwidth / 2,dispheight / 2,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	
		        	
		        	double error = Math.sqrt(Math.pow(eff_coord[0][0]-current_pos[0][0], 2)+ Math.pow(eff_coord[1][0]-current_pos[1][0], 2) + Math.pow(eff_coord[2][0]-current_pos[2][0], 2));
		        	
		        	if(error > 0.05){ // if effector target has moved, perform ik calculations and move
			        	double[][]d_pos = {{eff_coord[0][0]-current_pos[0][0]},
			        					   {eff_coord[1][0]-current_pos[1][0]},
			        					   {eff_coord[2][0]-current_pos[2][0]}};
			        	
			        	Matrix T = new Matrix(theta);
			        	dP_Matrix = new Matrix(d_pos);
			        	Matrix jacobian = new Matrix(JacCalc.getJacobian(theta));
			        	jacobian.print(System.out);
			        	Matrix invJacobian = jacobian.inverse();
			        	Matrix dT_Matrix = invJacobian.times(dP_Matrix);
			        	T.plusEquals(dT_Matrix);
			        	
			        	theta[0][0] = T.getArray()[0][0]; theta[1][0] = T.getArray()[1][0]; theta[2][0] = T.getArray()[2][0];
			        	
			        	Move.J1(dT_Matrix, true, 2000);
			        	UpdatePosition(current_pos, theta);
			        	
		       
		        	}
		        	glcds[1].drawString(String.format("%.1f", dP_Matrix.getArray()[0][0]),dispwidth * 1/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(String.format("%.1f", dP_Matrix.getArray()[1][0]),dispwidth * 2/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(String.format("%.1f", dP_Matrix.getArray()[2][0]),dispwidth * 3/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	
		        	Delay.msDelay(1000);
		        	glcds[1].clear();
		        	
		        }
//	        } 
//	        catch(Exception e){
//	        	System.out.println(e.getMessage());
//	        	
//	        }
//	        finally{
//	    		base.close();
//		        L1.close();
//		        L2.close();  
//	    	}
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

        finally {
    		base.close();
	        L1.close();
	        L2.close();  
        }
    	
    }
    
    public static double[][] CalculateAngle(){
    	double[][] new_theta = {{0},{0},{0}};
    	
    	return new_theta;
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
    	
    }
    
    //Function to validate that the point does not exceed the outer bound of the arm's reach
    public static boolean isInWorkSpace(double[][] point){
    	double norm = Math.sqrt(Math.pow(point[0][0], 2)+ Math.pow(point[1][0], 2) + Math.pow(point[2][0], 2));
    	double radius = l1 + l2;
    	if(norm <= radius){
    		return true;
    	} else {
    		return false;
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
