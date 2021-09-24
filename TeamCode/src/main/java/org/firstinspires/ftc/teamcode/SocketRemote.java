package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

@TeleOp(name = "ReDUX_Server", group = "Sensor")
public class SocketRemote extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData(">", " READY FOR START");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", " ACTIVE");
        telemetry.update();

        // SERVER CODE

        new SocketRemote().listen();

        // END SERVER CODE

    }


    public static final int port = 4444;
    private ServerSocket server;

    public void listen() {
        try {
            server = new ServerSocket(port);
        } catch (IOException e) {
            System.out.println("Could not listen on port 4444");
            System.exit(-1);
        }
        while(true){

            try {
                System.out.println( "Waiting for connection");
                final Socket socket = server.accept();

                final InputStream inputStream = socket.getInputStream();
                final InputStreamReader streamReader = new InputStreamReader(inputStream);
                BufferedReader br = new BufferedReader(streamReader);

                // readLine blocks until line arrives or socket closes, upon which it returns null
                String line = null;
                while ((line = br.readLine()) != null) {
                    if(line.startsWith("DD")) {
                        String driveData = line.split("DD")[1];
                        System.out.println("Recieved Drive Data: " + driveData);
                    }
                }

            } catch (IOException e) {
                System.out.println("Accept failed: 4444");
                System.exit(-1);
            }
        }
    }
}
