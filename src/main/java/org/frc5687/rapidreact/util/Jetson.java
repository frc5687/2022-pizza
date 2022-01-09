package org.frc5687.rapidreact.util;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;

import org.frc5687.rapidreact.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jetson extends Thread{

    private ServerSocket serverSocket;
    private Socket socket;
    private InputStream stream;
    private DataInputStream input;
    private boolean running = false;
    private String line = " ";
    private Listener listener;
    private DriverInterface driver;
    private Helpers helpers;
    private boolean end = false;
    //Variables to stroe the robots position
    private double estX = 0.0;
    private double estY = 0.0;
    private double estTheta = 0.0;

    public Jetson(){
        driver = new DriverInterface();
        helpers = new Helpers();
    }

    public void startListening(){
        try {
            serverSocket = new ServerSocket(Constants.Jetson.PORT);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        listener = new Listener();
        listener.run();
    }

    public Pose2d getPose(){
        Rotation2d robotRot = new Rotation2d(estTheta);
        Pose2d robotPose = new Pose2d(estX, estY, robotRot);
        return robotPose;
    }

    public String getRawPacket(){
        //Get the raw
       return line;
    }

    public double[] getDataArray(){
        double[] hold = {estX, estY, estTheta};
        return hold;
    }

    public void proccessPacket(String packet){
        getPose();
        //{x, y, theta}
        try{
            if(packet.contains("T") == true){
                //It's a translation data packet
                //driver.warn("Translation Data Found: " + packet);
                //There is at less a million ways to do this that are better but for now if it works it works
                //T(Y, Z, X)
                packet = packet.replace("T", "");
                packet = packet.replace("(", "");
                packet = packet.replace(")", "");
                driver.warn(packet.toString());
                String[] tData = packet.split(",");
                estX = Double.parseDouble(tData[2]);
                estY = Double.parseDouble(tData[0]);
                SmartDashboard.putString("Filtered X: ", getPose().toString());
                SmartDashboard.putString("Translation Y: ", tData[0]);
                SmartDashboard.putString("Translation Z: ", tData[1]);
                SmartDashboard.putString("Translation X: ", tData[2]);
            }else{
                if(packet.contains("R") == true){
                    //It's a rotation data packet
                    packet = packet.replace("R", "");
                    packet = packet.replace("(", "");
                    packet = packet.replace(")", "");
                    String[] rData = packet.split(",");
                    estTheta = Double.parseDouble(rData[2]);
                    SmartDashboard.putString("Rotation Y: ", rData[0]);
                    SmartDashboard.putString("Rotation Z: ", rData[1]);
                    SmartDashboard.putString("Rotation X: ", rData[2]);
                }else{
                    //Cannot ID incomeing data packet
                    driver.error("Unidentified Data Packet Found: " + packet);
                }
            }
        }catch(Exception e){
            driver.warn(e.toString() + " No data to parse");
        }
    }

    public boolean isServerRunning(){
        //Check if the server is running
        return running;
    }

    public void stopp(){
        //Stop the server form running
        try{
            socket.close();
            serverSocket.close();
        }catch(Exception e){

        }
    }

    public void periodic(){
        if(isServerRunning() == true){
            getPose();
        }else{

        }
    }

    class Listener{
        //Start server
        public void run(){
            new Thread(() -> {
                try{
                    byte[] messageByte = new byte[1000];
                    String dataString = "";
                    driver.warn("Starting Jetson Proxy Server");
                    driver.warn("Waiting For Jetson Connection..");
                    socket = serverSocket.accept();
                    driver.warn("Jetson Connection Accepted");
                    DataInputStream in = new DataInputStream(socket.getInputStream());
                    driver.warn("Reading From Jetson...");
                    while(!end){
                        int bytesRead = in.read(messageByte);
                        dataString += new String(messageByte, 0, bytesRead);
                        //driver.warn(dataString);
                        proccessPacket(dataString);
                        //Flush pose data
                        dataString = "";
                    }
                }catch(Exception e){
                    DriverStation.reportWarning(e.toString(), true);
                    running = false;
                }
            }).start();
        }
    }
}