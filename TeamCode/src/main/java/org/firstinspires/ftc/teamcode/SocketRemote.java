package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.*;
import java.net.*;

@TeleOp(name = "SocketRemote", group = "Sensor")
public class SocketRemote extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            ServerSocket serverSocket = new ServerSocket(6969);
            System.out.println("SocketManager: " + "Waiting for client on port 6969");
            waitForStart();
            while (opModeIsActive()) {
                Socket clientSocket = serverSocket.accept();
                System.out.println("SocketManager: " + "Connected client");
                PrintWriter out =
                        new PrintWriter(clientSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(
                        new InputStreamReader(clientSocket.getInputStream()));

                String inData = in.readLine();
                out.println("PONG: " + inData);
                System.out.println("Socket Manager: Client: " + inData);
            }

            serverSocket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
