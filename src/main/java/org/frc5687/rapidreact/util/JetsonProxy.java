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

    public JetsonProxy() throws SocketException{
        socket = new DatagramSocket(5687);
    }

    public void run() throws IOException {
        running = true;

        while (running) {
            packet = new DatagramPacket(buf, buf.length);
            socket.receive(packet);
            
            InetAddress address = packet.getAddress();
            int port = packet.getPort();
            packet = new DatagramPacket(buf, buf.length, address, port);
            String received 
              = new String(packet.getData(), 0, packet.getLength());
            SmartDashboard.putString("Packet", received);
            
            if (received.equals("end")) {
                running = false;
                continue;
            }
            socket.send(packet);
        }
        socket.close();
    }
}
