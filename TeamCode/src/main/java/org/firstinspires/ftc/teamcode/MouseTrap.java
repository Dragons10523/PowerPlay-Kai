package org.firstinspires.ftc.teamcode;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;

import java.util.Collection;

public class MouseTrap implements Runnable {
    private final double theta_constant    = 2 * Math.PI / 2000.0;
    private final double distance_constant = 1.0 / 400.0;

    private int mouse_x   = 0;
    private int mouse_y   = 0;

    protected double theta = Math.PI / 2;
    protected double x     = 0;
    protected double y     = 0;

    private Context             context;
    private UsbManager          usb;
    private UsbDevice           mouse;
    private UsbInterface        mouseInterface;
    private UsbDeviceConnection connection;
    private UsbEndpoint         endpoint;


    MouseTrap (Context context) {
        this.context = context;
        usb          = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        Collection<UsbDevice> deviceList = usb.getDeviceList().values();
        for(UsbDevice device : deviceList) {
            if(device.getInterface(0).getInterfaceClass() == UsbConstants.USB_CLASS_HID){
                mouse = device;
                break;
            }
        }

        PendingIntent intent = PendingIntent.getActivity(context,42, new Intent(context,MouseTrap.class),0);
        usb.requestPermission(mouse,intent);
        mouseInterface = mouse.getInterface(0);
        connection     = usb.openDevice(mouse);
        connection.claimInterface(mouseInterface, true);
        endpoint = mouseInterface.getEndpoint(0);
    }

    @Override
    public void run() {
        while(usb.hasPermission(mouse)){
            byte[] data = new byte[endpoint.getMaxPacketSize()];
            connection.bulkTransfer(endpoint,data,data.length,0);

            // TODO: reverse values on actual robot
            int x_sign = 1;
            int y_sign = 1;
            if((data[0] & 0x10) != 0) x_sign = -1;
            if((data[0] & 0x20) != 0) y_sign = -1;

            doMath(x_sign * data[1],y_sign * data[2]);
        }
    }

    private void doMath(int deltaX, int deltaY){
        mouse_x += deltaX;
        mouse_y += deltaY;
        theta   += theta_constant    * deltaX;
        x       += distance_constant * deltaY * Math.cos(theta);
        y       += distance_constant * deltaY * Math.sin(theta);
    }

    public int getMouseX(){
        return mouse_x;
    }

    public int getMouseY(){
        return mouse_y;
    }
}
