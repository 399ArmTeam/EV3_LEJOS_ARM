package jcontroller;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;


public class DirMotControl {
	public static UnregulatedMotor LR = new UnregulatedMotor(MotorPort.A);
    public static UnregulatedMotor FB = new UnregulatedMotor(MotorPort.B);
    public static UnregulatedMotor UD = new UnregulatedMotor(MotorPort.C);
    public static UnregulatedMotor [] controller_motors = {LR, FB, UD};
    
    public static double l1 = JacCalc.l1;
    public static double l2 = JacCalc.l2;
    public static double base_height = JacCalc.base_height;
    
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
	        try{
	        	UnregulatedMotor base = new UnregulatedMotor(bricks[1].getPort("A"));
		        UnregulatedMotor L1 = new UnregulatedMotor(bricks[1].getPort("B"));
		        UnregulatedMotor L2 = new UnregulatedMotor(bricks[1].getPort("C"));
		        UnregulatedMotor [] arm_motors = {base, L1, L2};
		        glcds[1].clear();
		        
		        double x_coord = 0;double y_coord = 0;double z_coord = 0;
		        double[][] eff_coord = {{x_coord}, {y_coord}, {z_coord}};
		        while(Button.ENTER.isUp())
		        {		        	
		        	UpdateEffector(eff_coord);
		        	glcds[1].drawString("End Eff. Coorinates:[x,y,z]",dispwidth / 2,dispheight / 2,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Double.toString(x_coord),dispwidth * 1/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Double.toString(y_coord),dispwidth * 2/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	glcds[1].drawString(Double.toString(z_coord),dispwidth * 3/4, dispheight* 3/4,GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
		        	
		        	Delay.msDelay(100);
		        	glcds[1].clear();
		        	
		        	
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
    
    //Function to update 
    public static void UpdateEffector(double[][]coord){
    	//double [][] new_coord = {{coord[0][0]},{coord[1][0]},{coord[2][0]}};
    	//Change y-coordinate
    	if(LR.getTachoCount()<-10||LR.getTachoCount()>10){
    		coord[1][0] += LR.getTachoCount()*0.1;
    	}
    	//Change x-coordinate
    	if(FB.getTachoCount()>10||FB.getTachoCount()<-10){
    		coord[0][0] += LR.getTachoCount()*0.1;
    	} 
    	//Change z-coordinate
    	if(UD.getTachoCount()>10||UD.getTachoCount()<-10){
    		coord[2][0] =+ LR.getTachoCount()*0.1;
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
    	
    	return true;
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
		        	
		        	if(LR.getTachoCount()<-10){
		        		base.setPower((int)((LR.getTachoCount()*1.5) % 100));
		        		base.backward();
		        	} else if (LR.getTachoCount()>10){
		        		base.setPower((int)((LR.getTachoCount()*1.5) % 100));
		        		base.backward();
		        	} else {
		        		base.stop();
		        	}
		        	
		        	if(FB.getTachoCount()>10){
		        		L1.setPower(Math.abs((int)((FB.getTachoCount()*1.5) % 100)));
		        		L1.forward();
		        	} else if (FB.getTachoCount()<-10){
		        		L1.setPower(Math.abs((int)((FB.getTachoCount()*1.5) % 100)));
		        		L1.backward();
		        	} else {
		        		L1.stop();
		        	}
		        	
		        	if(UD.getTachoCount()>10){
		        		L2.setPower(Math.abs((int)((UD.getTachoCount()*1.5) % 100)));
		        		L2.forward();
		        	} else if (UD.getTachoCount()<-10){
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
