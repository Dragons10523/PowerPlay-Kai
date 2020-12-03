package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import org.apache.commons.codec.DecoderException;
import org.apache.commons.codec.binary.Hex;

public class SloppetySocket extends Thread {
    private int port;
    private byte[] PASSWORD_HASH;

    private ServerSocket serverSocket;
    private Socket clientSocket;
    public BufferedReader in;
    public PrintWriter out;

    SloppetySocket(int port) {
        this.port = port;
        try {
            PASSWORD_HASH = Hex.decodeHex("cc12d2f761f4c47991e078a6c70138d1d98638f8d234cc0888c750ed60e8c9de");
        } catch(DecoderException ex) {
            ex.printStackTrace();
        }
    }

    public void run() {
        try {
            serverSocket = new ServerSocket(port);
            clientSocket = serverSocket.accept();
            in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
            out = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream()));
            authenticateClient();

        } catch (IOException e){
            e.printStackTrace();
        }
    }

    private boolean authenticateClient() throws IOException {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            out.println("Welcome to the Thalatte Remote Interface!");
            out.println("Please send your super secure password through this super secure connection.");
            String pass = in.readLine();
            byte[] hash = digest.digest(pass.getBytes(StandardCharsets.UTF_8));
            if (hash.equals(PASSWORD_HASH)){
                out.print("Authenticating . . .");
                sleep(1000);
                out.println("Authenticated.");
                out.println("Hello Dragons.");
                return true;
            } else {
                out.println("Who the heck are you? Go away.");
                clientSocket.close();
                return false;
            }
        } catch(InterruptedException e){
            out.println("Interrupted. Bye Bye.");
            serverSocket.close();
        } catch(NoSuchAlgorithmException e){
            out.println("Authentication Error.");
            serverSocket.close();
        }
        return false;
    }

}
