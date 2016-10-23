package org.usfirst.frc.team540.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class UDP implements Runnable
{
	byte[] data;
	String[] out;
	DatagramSocket socket = null;
	BufferedReader in = null;
	byte[] buf = new byte[256];
	DatagramPacket packet = new DatagramPacket(buf, buf.length);
	String lastDataReceived = "";
	public void run()
	{
		try {
			socket = new DatagramSocket(1234);
		} catch (SocketException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		packet.setLength(buf.length);
		try {
			socket.receive(packet);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		byte[] data = packet.getData();
		lastDataReceived = new String(data, 0, packet.getLength());
		while(true)
		{
			try {
				socket.receive(packet);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			data = packet.getData();
			lastDataReceived = new String(data, 0, packet.getLength());
			out = lastDataReceived.split(",");
			Robot.tableVals[0] = Double.parseDouble(out[0]);
			Robot.tableVals[1] = Double.parseDouble(out[1]);
		}
	}
}
