package org.frc5687.rapidreact.util;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonProxy {
    private DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[256];
    DatagramPacket packet;

    public JetsonProxy(){
        try {
            socket = new DatagramSocket(5687);
            SmartDashboard.putString("Starting jetson proxy", "started");
        } catch (SocketException e) {
            e.printStackTrace();
            SmartDashboard.putString("Socket", e.toString());
        }
    }

    public void run(){
        new Thread(() -> {
            try{
                running = true;
                while (running) {
                    SmartDashboard.putString("Listening", "true");
                    packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);
                    InetAddress address = packet.getAddress();
                    int port = packet.getPort();
                    packet = new DatagramPacket(buf, buf.length, address, port);
                    String received = new String(packet.getData(), 0, packet.getLength());
                    SmartDashboard.putString("Packet", received);
                    
                    if (received.equals("end")) {
                        running = false;
                        continue;
                    }
                }
                socket.close();
            } catch (IOException e) {
                SmartDashboard.putString("mke", "ionr");
                e.printStackTrace();
            }
            finally{
                SmartDashboard.putString("Debug2", "Debug2");
            }
        }).start();
    }
}
