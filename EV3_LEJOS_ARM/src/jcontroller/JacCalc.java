package jcontroller;
import Jama.Matrix;
import java.lang.Math;

//Class for handling 3x3 Jacobian calculations 
public class JacCalc {
	public static double base_height = 17.5; //cm
	public static double l2 = 22.1; //cm
	public static double l1 = 13.4; //cm
	
	public static double[][] getJacobian(double[][] theta){
		//pos:3x1, tar:3x1, theta:3x1
				
		double dx_dt1 = Math.sin(Math.toRadians(theta[0][0]))*(-1*(l1*Math.cos(Math.toRadians(theta[1][0]))+l2*Math.cos(Math.toRadians(theta[1][0]+theta[2][0]))));
		double dx_dt2 = -1*Math.cos(Math.toRadians(theta[0][0]))*(l1*Math.sin(Math.toRadians(theta[1][0]))+l2*Math.sin(Math.toRadians(theta[1][0]+theta[2][0])));
		double dx_dt3 = -1*l2*Math.cos(Math.toRadians(theta[0][0]))*Math.sin(Math.toRadians(theta[1][0]+theta[2][0]));
		double dy_dt1 = Math.cos(Math.toRadians(theta[0][0]))*(l1*Math.cos(Math.toRadians(theta[1][0])) + l2 * Math.cos(Math.toRadians(theta[1][0]+theta[2][0])));
		double dy_dt2 = -1*Math.sin(Math.toRadians(theta[0][0]))*(l1*Math.sin(Math.toRadians(theta[1][0])) + l2 * Math.sin(Math.toRadians(theta[1][0]+theta[2][0])));
		double dy_dt3 = -1*l2*Math.sin(Math.toRadians(theta[0][0]))*Math.sin(Math.toRadians(theta[1][0]+theta[2][0]));
		double dz_dt1 = 0;
		double dz_dt2 = l1*Math.cos(Math.toRadians(theta[1][0]))+l2*Math.cos(Math.toRadians(theta[1][0]+theta[2][0]));
		double dz_dt3 = l2*Math.cos(Math.toRadians(theta[1][0]+theta[2][0]));
		
		double[][] Jac_vals = {{dx_dt1,dy_dt1,dz_dt1},
							   {dx_dt2,dy_dt2,dz_dt2},
				  		  	   {dx_dt3,dy_dt3,dz_dt3}}; //3x3 matrix
		
		//Matrix Jacobian = new Matrix(Jac_vals);
		return Jac_vals;
	}
	
}
