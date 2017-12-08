package vision;

//http://www.binarytides.com/java-socket-programming-tutorial/
import java.io.*;
import java.net.*;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class TrackerReader2 extends Thread
{
    public volatile double x;
    public volatile double y;
    public volatile double a;
    public volatile double theta;
    public volatile double targetx;
    public volatile double targety;

    //public static void main(String args[])
    @SuppressWarnings("deprecation")
	public void run()
    {
        x = 0;
        y = 0;
        System.out.println("LISTENER");
        ServerSocket s = null;
        Socket conn = null;

        try
        {
            //1. creating a server socket - 1st parameter is port number and 2nd is the backlog
            s = new ServerSocket(5001 , 10);

            //2. Wait for an incoming connection
            System.out.println("Server socket created.Waiting for connection...");

            conn = s.accept();

            //print the hostname and port number of the connection
            System.out.println("Connection received from " + conn.getInetAddress().getHostName() + " : " + conn.getPort());
            String line = "";
            JSONParser parser=new JSONParser();
            try
            {
                //get socket writing and reading streams
                DataInputStream in = new DataInputStream(conn.getInputStream());

                //Now start reading input from client
                while((line = in.readLine()) != null && !line.equals("."))
                {
                    try{
                        //System.out.println(line);
                        Object obj=parser.parse(line);
                        JSONObject jsonObject = (JSONObject) obj;
                        x = (Double) jsonObject.get("x");
                        y = (Double) jsonObject.get("y");
                        a = (Double) jsonObject.get("a");
                        theta = (Double) jsonObject.get("theta");
                        targetx = (Double) jsonObject.get("targetx");
                        targety = (Double) jsonObject.get("targety");
                    }catch (ParseException e) {
                        e.printStackTrace();
                    }
                }

                //client disconnected, so close socket
                conn.close();
            }
            catch (IOException e)
            {
                System.out.println("IOException on socket : " + e);
                e.printStackTrace();
            }

        }

        catch(IOException e)
        {
            System.err.println("IOException");
        }

        try
        {
            s.close();
        }
        catch(IOException ioException)
        {
            System.err.println("Unable to close. IOexception");
        }
    }
    
    public double[] getCurrentXY() {
    	double[] xy = {targetx, targety};
    	return xy;
    }
}