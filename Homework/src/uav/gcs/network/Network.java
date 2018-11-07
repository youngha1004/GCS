package uav.gcs.network;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Properties;

public class Network {
    private static Logger Logger = LoggerFactory.getLogger(Network.class); //loggerFactroty 자작한 클래스 선언.
    private static UAV uav; //static 붙으면 객체 없이 생성 가능
    public static String networkType;
    public static String udpLocalPort;
    public static String tcpServerIP;
    public static String tcpServerPort;

    static {
        try {
            Properties properties = new Properties(); //저장객체 만들고
            FileReader reader = new FileReader(
                    Network.class.getResource("network.properties").getPath()); //상대경로지정
            properties.load(reader);

            networkType = properties.getProperty("networkType");
            udpLocalPort = properties.getProperty("udpLocalPort");
            tcpServerIP = properties.getProperty("tcpServerIP");
            tcpServerPort = properties.getProperty("tcpServerPort");
        } catch (IOException e) {
            Logger.error(e.toString());
        }
    }

    public static void save(){
        try {
            // properties files are in out folder (relative paths are depending on class file)
            // 속성파일은 아웃 경로에 있고 (상대 경로는 클래스 파일에 따라 다름.)
            PrintWriter writer = new PrintWriter(Network.class.getResource("network.properties").getPath()); //상대경로 지정
            writer.println("networkType="+networkType);
            writer.println("udpLocalPort="+udpLocalPort);
            writer.println("tcpServerIP="+tcpServerIP);
            writer.println("tcpServerPort="+tcpServerPort);
            writer.flush(); //버퍼를 비워준다.
            writer.close(); // 종료.
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static UAV createUAV(){
        if(networkType.equals("UDP")){ //참조 값을 비교
            uav = new UAVUDP(udpLocalPort);
        }else if(networkType.equals("TCP")){
            uav = new UAVTCP(tcpServerIP, tcpServerPort);
        }
        return uav;
    }

    public static UAV getUAV(){
        return uav;
    }

    public static void destroyUAV(){
        if(uav != null) { //uav가 null 아니면 실행종료하고 uav null 값으로 만들어준다.
            uav.disconnect();
            uav = null;
        }
    }
}
