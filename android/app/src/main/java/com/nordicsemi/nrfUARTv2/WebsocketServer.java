package com.nordicsemi.nrfUARTv2;

import java.net.InetSocketAddress;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import com.nordicsemi.nrfUARTv2.MainActivity.ReceiveHandler;

import android.os.Message;
import android.util.Log;

public class WebsocketServer extends WebSocketServer {

    public static final byte TYPE_STM = 0x00;
    public static final byte TYPE_BLE = 0x01;
    public String TAG = "nRFUART_ws";

    public WebSocket mClientSession = null;
    private final int RECEIVE_MSG = 1;
    private ReceiveHandler mHandler;

    public WebsocketServer(int port, ReceiveHandler handler) {
        super(new InetSocketAddress(port));

        mHandler = handler;
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake arg1) {
        // TODO Auto-generated method stub
        Log.d(TAG, "Server onOpen getHostAddress:" + conn.getRemoteSocketAddress().getAddress().getHostAddress());
        mClientSession = conn;
    }

    @Override
    public void onMessage(WebSocket conn, String arg1) {
        // TODO Auto-generated method stub
        Log.d(TAG, "Server onMessage getHostAddress:" + conn.getRemoteSocketAddress().getAddress().getHostAddress());
        Log.d(TAG, "Server onMessage msg:" + arg1);

        Message msg = mHandler.obtainMessage();
        msg.what = RECEIVE_MSG;
        msg.obj = arg1;
        //mClientSession.send(arg1);//loopback
        mHandler.sendMessage(msg);
    }

    @Override
    public void onClose(WebSocket conn, int arg1, String arg2, boolean arg3) {
        // TODO Auto-generated method stub
        Log.d(TAG, "Server onClose getHostAddress:" + conn.getRemoteSocketAddress().getAddress().getHostAddress());

        mClientSession = null;
    }

    @Override
    public void onError(WebSocket conn, Exception arg1) {
        // TODO Auto-generated method stub
        Log.d(TAG, "Server client onError:" + arg1);

        mClientSession = null;
    }

    @Override
    public void onStart() {
        // TODO Auto-generated method stub
        Log.d(TAG, "Server client onStart");
    }
}
