package uav.gcs.network;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

public class UAVUDP extends UAV {
    private static Logger Logger = LoggerFactory.getLogger(UAVUDP.class);
    private DatagramSocket socket;
    private String sendIP;
    private String udpLocalPort;
    private int sendPort;

    public UAVUDP(String udpLocalPort) {
        this.udpLocalPort = udpLocalPort;
    }

    @Override
    public void connect() {
        // 통신 방식에 따라 연결코드 작성
        try {
            socket = new DatagramSocket(Integer.parseInt(udpLocalPort));

            // 연결 후 공통 실행코드를 상위클래스에서 호출
            super.connect();
        } catch (SocketException e) {
            Logger.error(e.toString());
        }
    }

    @Override
    public void disconnect() {
        // 종료 전 공통 실행코드를 상위클래스에서 호출
        super.disconnect();

        // 통신 방식에 따라 종료코드 작성
        socket.close();
    }

    @Override
    public void receiveMessage() throws Exception {
        byte[] buffer = new byte[263];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        while(connected){
            socket.receive(packet);
            int readBytes = packet.getLength();
            sendIP = packet.getAddress().getHostAddress();
            sendPort = packet.getPort();
            for(int i = 0; i < readBytes; i++){
                parsingMAVLinkMessage(buffer[i]);
            }
        }
    }

    @Override
    public void sendMessage(byte[] bytes) throws Exception {
        if(sendIP != null) {
            DatagramPacket packet = new DatagramPacket(bytes, bytes.length, new InetSocketAddress(sendIP, sendPort));
            socket.send(packet);
        }
    }
}
