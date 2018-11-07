package uav.gcs.network;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

public class UAVTCP extends UAV {
    private static Logger Logger = LoggerFactory.getLogger(UAVTCP.class);
    private Socket socket;
    private String tcpServerIP;
    private String tcpServerPort;
    private InputStream inputStream;
    private OutputStream outputStream;

    public UAVTCP(String tcpServerIP, String tcpServerPort) {
        this.tcpServerIP = tcpServerIP;
        this.tcpServerPort = tcpServerPort;
    }

    @Override
    public void connect() {
        try {
            // 통신 방식에 따라 연결코드 작성
            socket = new Socket(tcpServerIP, Integer.parseInt(tcpServerPort));
            inputStream = socket.getInputStream();
            outputStream = socket.getOutputStream();

            // 연결 후 공통 실행코드를 상위클래스에서 호출
            super.connect();
        } catch (Exception e) {
            Logger.error(e.toString());
        }
    }

    @Override
    public void disconnect() {
        try {
            // 종료 전 공통 실행코드를 상위클래스에서 호출
            super.disconnect();

            // 통신 방식에 따라 종료코드 작성
            socket.close();

        } catch (Exception e) {
            // socket 이 null 이거나 이미 연결이 끊겨있을 경우 Exception 발생
            Logger.error(e.toString());
        }
    }

    @Override
    public void receiveMessage() throws Exception {
        byte[] buffer = new byte[1024];
        int readBytes = -1;
        while(connected) {
            readBytes = inputStream.read(buffer);
            if(readBytes == -1){
                throw new Exception("read() returned -1");
            }
            for(int i = 0; i < readBytes; i++){
                parsingMAVLinkMessage(buffer[i]);
            }
        }
    }

    @Override
    public void sendMessage(byte[] bytes) throws Exception {
        outputStream.write(bytes);
        outputStream.flush();
    }
}
