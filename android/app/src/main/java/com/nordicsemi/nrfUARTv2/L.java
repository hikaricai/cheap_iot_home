package com.nordicsemi.nrfUARTv2;

/**
 * Created by hikari on 2018/1/4.
 */
import android.util.Log;


public class L {
    private static final boolean SHUT_UP = false;

    public static void d(Object o){
        if(BuildConfig.DEBUG && !SHUT_UP)
            Log.d("nRFUART_adk", String.valueOf(o));
    }
    public static void d(String s, Object ... args){
        if(BuildConfig.DEBUG && !SHUT_UP)
            Log.d("nRFUART_adk", String.format(s,args));
    }

    public static void e(Object o){
        if(BuildConfig.DEBUG && !SHUT_UP)
            Log.e("nRFUART_adk", String.valueOf(o));
    }
    final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes,int len) {
        char[] hexChars = new char[len * 2];
        for ( int j = 0; j < len; j++ ) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }
    public static byte[] hexString2Bytes(String src) {
        int l = src.length() / 2;
        byte[] ret = new byte[l];
        for (int i = 0; i < l; i++) {
            ret[i] = (byte) Integer
                    .valueOf(src.substring(i * 2, i * 2 + 2), 16).byteValue();
        }
        return ret;
    }
}