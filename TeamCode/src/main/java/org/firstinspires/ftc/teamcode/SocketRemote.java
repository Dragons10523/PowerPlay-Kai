package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.corundumstudio.socketio.AckRequest;
import com.corundumstudio.socketio.Configuration;
import com.corundumstudio.socketio.SocketIOClient;
import com.corundumstudio.socketio.SocketIOServer;
import com.corundumstudio.socketio.Transport;
import com.corundumstudio.socketio.listener.DataListener;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

@TeleOp(name = "SocketRemote", group = "Sensor")
public class SocketRemote extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        // SERVER CODE

        new SocketRemote().listen();

        // END SERVER CODE

    }


    public static final int port = 12345;
    private ServerSocket server;

    public void listen() {
        try {
            server = new ServerSocket(4444);
        } catch (IOException e) {
            System.out.println("Could not listen on port 4444");
            System.exit(-1);
        }
        while(true){

            try {
                System.out.println("Waiting for connection");
                final Socket socket = server.accept();

                final InputStream inputStream = socket.getInputStream();
                final InputStreamReader streamReader = new InputStreamReader(inputStream);
                BufferedReader br = new BufferedReader(streamReader);

                // readLine blocks until line arrives or socket closes, upon which it returns null
                String line = null;
                while ((line = br.readLine()) != null) {
                    System.out.println(line);
                }

            } catch (IOException e) {
                System.out.println("Accept failed: 4444");
                System.exit(-1);
            }
        }
    }
}
