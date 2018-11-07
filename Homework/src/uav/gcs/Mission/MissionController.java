package uav.gcs.Mission;

import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.common.msg_mission_ack;
import com.MAVLink.common.msg_mission_item_int;
import com.MAVLink.enums.COPTER_MODE;
import com.MAVLink.enums.MAV_CMD;
import com.MAVLink.enums.MAV_COMPONENT;
import com.MAVLink.enums.MAV_FRAME;
import common.AlertDialog;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.concurrent.Worker;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.web.WebEngine;
import javafx.scene.web.WebView;
import javafx.stage.FileChooser;
import netscape.javascript.JSObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import uav.gcs.main.AppMain;
import uav.gcs.main.Mission;
import uav.gcs.network.Network;
import uav.gcs.network.UAV;

import java.io.File;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.ResourceBundle;

public class MissionController implements Initializable {
    private static Logger logger = LoggerFactory.getLogger(MissionController.class);
    public static MissionController instance;

    // Map Web
    @FXML private WebView webView;
    @FXML private Slider zoomSlider;
    private WebEngine webEngine;
    private JSObject jsproxy;

    // Manual
    @FXML private CheckBox checkManualMove;
    @FXML private CheckBox checkManualAlt;
    @FXML private TextField txtManualAlt;

    // Mission
    @FXML private Label lblInfo;
    private Thread infoThread;
    private Thread infoLabelThread;

    // Mission Table
    @FXML private TableView missionTable;

    // Mission Table Selection (for Mission Delete)
    private ObservableList<Mission> missions = FXCollections.observableArrayList();
    private Random random = new Random();
    private int selectedMissionSeq = -1;


    @Override
    public void initialize(URL location, ResourceBundle resources) {
        MissionController.instance = this;

        initWebView();
        initMissionTableView();

        // Map Slider Listener
        zoomSlider.valueProperty().addListener(
                (ObservableValue<? extends Number> observable, Number oldValue, Number newValue) ->
                        jsproxy.call("setMapZoom", newValue.intValue())
        );

        // Mission Table Selection Listener
        missionTable.getSelectionModel().selectedIndexProperty().addListener(
                (ObservableValue<? extends Number> observable, Number oldValue, Number newValue) ->
                        selectedMissionSeq = newValue.intValue()
        );
    }

    private void initWebView(){
        webEngine = webView.getEngine();
        webEngine.getLoadWorker().stateProperty().addListener(webEngineLoadStateListener);
        webEngine.load(MissionController.class.getResource("javascript/map.html").toExternalForm());
    }

    private ChangeListener<Worker.State> webEngineLoadStateListener =
            (ObservableValue<? extends Worker.State> observable,
             Worker.State oldValue, Worker.State newValue) -> {
                if(newValue == Worker.State.SUCCEEDED){
                    logger.info("Success downloading and encoding all htmls");
                    jsproxy = (JSObject) webEngine.executeScript("jsproxy");
                    jsproxy.setMember("java", MissionController.this);
                }
            };


    // Initialize Mission Table Column
    private void initMissionTableView(){
        TableColumn<Mission, Integer> column1 = new TableColumn<>("SEQ");
        TableColumn<Mission, String> column2 = new TableColumn<>("COMMAND");
        TableColumn<Mission, Double> column3 = new TableColumn<>("P1");
        TableColumn<Mission, Double> column4 = new TableColumn<>("P2");
        TableColumn<Mission, Double> column5 = new TableColumn<>("P3");
        TableColumn<Mission, Double> column6 = new TableColumn<>("P4");
        TableColumn<Mission, Double> column7 = new TableColumn<>("LAT");
        TableColumn<Mission, Double> column8 = new TableColumn<>("LNG");
        TableColumn<Mission, Double> column9 = new TableColumn<>("ALT");
        column1.setCellValueFactory(new PropertyValueFactory<>("seq"));
        column2.setCellValueFactory(new PropertyValueFactory<>("command"));
        column3.setCellValueFactory(new PropertyValueFactory<>("p1"));
        column4.setCellValueFactory(new PropertyValueFactory<>("p2"));
        column5.setCellValueFactory(new PropertyValueFactory<>("p3"));
        column6.setCellValueFactory(new PropertyValueFactory<>("p4"));
        column7.setCellValueFactory(new PropertyValueFactory<>("lat"));
        column8.setCellValueFactory(new PropertyValueFactory<>("lng"));
        column9.setCellValueFactory(new PropertyValueFactory<>("alt"));
        column1.setPrefWidth(40);
        column2.setPrefWidth(100);
        column3.setPrefWidth(60);
        column4.setPrefWidth(60);
        column5.setPrefWidth(60);
        column6.setPrefWidth(60);
        column7.setPrefWidth(60);
        column8.setPrefWidth(60);
        column9.setPrefWidth(60);
        missionTable.getColumns().add(column1);
        missionTable.getColumns().add(column2);
        missionTable.getColumns().add(column3);
        missionTable.getColumns().add(column4);
        missionTable.getColumns().add(column5);
        missionTable.getColumns().add(column6);
        missionTable.getColumns().add(column7);
        missionTable.getColumns().add(column8);
        missionTable.getColumns().add(column9);
    }

    public void handleMissionTakeOff(ActionEvent e){
        int seq = missions.size();
        String command = "TAKE OFF";
        double[] point = new double[4];
        double lat, lng, alt;

        for(int i = 0; i < point.length; i++){
            point[i] = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        }

        lat = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        lng = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));

        alt = 0;

        missions.add(new Mission(seq, command, point[0], point[1], point[2], point[3], lat, lng, alt));
        missionTable.setItems(missions);
    }

    public void handleMissionLand(ActionEvent e){
        int seq = missions.size();
        String command = "Land";
        double[] point = new double[4];
        double lat, lng, alt;

        for(int i = 0; i < point.length; i++){
            point[i] = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        }

        lat = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        lng = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));

        alt = 0;

        missions.add(new Mission(seq, command, point[0], point[1], point[2], point[3], lat, lng, alt));
        missionTable.setItems(missions);
    }


    public void handleMissionRTL(ActionEvent e){
        int seq = missions.size();
        String command = "RTL";
        double[] point = new double[4];
        double lat, lng, alt;

        for(int i = 0; i < point.length; i++){
            point[i] = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        }

        lat = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        lng = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));

        alt = 0;

        missions.add(new Mission(seq, command, point[0], point[1], point[2], point[3], lat, lng, alt));
        missionTable.setItems(missions);
    }


    public void handleMissionWaypoint(ActionEvent e){
        int seq = missions.size();
        String command = "WAYPOINT";
        double[] point = new double[4];
        double lat, lng, alt;

        for(int i = 0; i < point.length; i++){
            point[i] = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        }

        lat = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        lng = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));

        alt = 0;

        missions.add(new Mission(seq, command, point[0], point[1], point[2], point[3], lat, lng, alt));
        missionTable.setItems(missions);
    }


    public void handleMissionJump(ActionEvent e){
        int seq = missions.size();
        String command = "Jump";
        double[] point = new double[4];
        double lat, lng, alt;

        for(int i = 0; i < point.length; i++){
            point[i] = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        }

        lat = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));
        lng = Double.parseDouble(String.format("%.2f", random.nextDouble()*100));

        alt = 0;

        missions.add(new Mission(seq, command, point[0], point[1], point[2], point[3], lat, lng, alt));
        missionTable.setItems(missions);
    }

    public void handleMissionDelete(ActionEvent e){
        if( selectedMissionSeq >= 0 ){
            for(int i = selectedMissionSeq +1; i < missions.size(); i++){
                missions.get(i).setSeq(missions.get(i).getSeq()-1);
            }
            missions.remove(selectedMissionSeq);
            missionTable.setItems(missions);
            return;
        }
    }

    public void changeStatus(){
        Platform.runLater(
                ()-> {
                    UAV uav = Network.getUAV();
                    if(uav != null) {
                        if (uav.homeLng != 0.0) {
                            jsproxy.call("setHomePosition", uav.homeLat, uav.homeLng);
                        }

                        if (uav.currLat != 0.0) {
                            jsproxy.call("setCurrentPosition", uav.currLat, uav.currLng, uav.heading);
                        }

                        int mode = uav.modeInt;
                        switch (mode){
                            case COPTER_MODE.COPTER_MODE_LAND: jsproxy.call("setMode", false, false, false, true);
                                break;
                            case COPTER_MODE.COPTER_MODE_RTL: jsproxy.call("setMode", false, false, true, false);
                                break;
                            case COPTER_MODE.COPTER_MODE_AUTO: jsproxy.call("setMode", false, true, false, false);
                                break;
                            case COPTER_MODE.COPTER_MODE_GUIDED: jsproxy.call("setMode", true, false, false, false);
                                break;
                        }
                    }
                }
        );
    }

    public void handleBtnManual(ActionEvent e){
        UAV uav = Network.getUAV();
        if(uav != null) {
            boolean isMoveChecked = checkManualMove.isSelected();
            boolean isAltChecked = checkManualAlt.isSelected();
            double targetAlt = uav.alt;
            if (!txtManualAlt.getText().isEmpty()) {
                targetAlt = Double.parseDouble(txtManualAlt.getText());
            }

            if (isMoveChecked == true && isAltChecked == false) {
                jsproxy.call("manual", uav.alt);

            } else if (isMoveChecked == true && isAltChecked == true) {
                jsproxy.call("manual", targetAlt);
            } else if (isMoveChecked == false && isAltChecked == true) {
                uav.sendSetPositionTargetGlobalInt(uav.currLat, uav.currLng, targetAlt);
            }
        }
    }

    public void handleCheckManualAlt(ActionEvent e){
        if(checkManualAlt.isSelected()){
            UAV uav = Network.getUAV();
            if(uav != null) {
                txtManualAlt.setText(String.valueOf(uav.alt));
            }
        }
    }

    public void handleBtnMissionUpload(ActionEvent e){
        UAV uav = Network.getUAV();

        // Mission items storage
        Map<Integer, msg_mission_item_int> items = new HashMap<>();

        // First mission is always "Home"
        msg_mission_item_int msgHome = new msg_mission_item_int();
        msgHome.seq = 0;
        msgHome.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT;
        msgHome.target_system = 1;
        msgHome.target_component = MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
        msgHome.autocontinue = 1;
        msgHome.x = (int) (uav.homeLat * 10000000);
        msgHome.y = (int) (uav.homeLng * 10000000);
        msgHome.z = 0.0F;
        logger.error("HomeLatLng: " + uav.homeLat+", "+uav.homeLng);

        // Second Mission: Take off
        msg_mission_item_int msgTakeOff = new msg_mission_item_int();
        msgTakeOff.seq = 1;
        msgTakeOff.command = MAV_CMD.MAV_CMD_NAV_TAKEOFF;
        msgTakeOff.target_system = 1;
        msgTakeOff.target_component = MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
        msgTakeOff.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        msgTakeOff.autocontinue = 1;
        msgTakeOff.z = 10.0F;

        // Third Mission : Waypoint
        msg_mission_item_int msgFirst = new msg_mission_item_int();
        msgFirst.seq = 2;
        msgFirst.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT;
        msgFirst.target_system = 1;
        msgFirst.target_component = MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
        msgFirst.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        msgFirst.autocontinue = 1;
        msgFirst.x = 375475886;
        msgFirst.y = 1271192193;
        msgFirst.z = 20.0F;

        // Forth Mission : Waypoint
        msg_mission_item_int msgSecond = new msg_mission_item_int();
        msgSecond.seq = 3;
        msgSecond.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT;
        msgSecond.target_system = 1;
        msgSecond.target_component = MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
        msgSecond.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        msgSecond.autocontinue = 1;
        msgSecond.x = 375470442;
        msgSecond.y = 1271191335;
        msgSecond.z = 20.0F;

        // Final Mission: RTL
        msg_mission_item_int msgRTL = new msg_mission_item_int();
        msgRTL.seq = 4;
        msgRTL.command = MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH;
        msgRTL.target_system = 1;
        msgRTL.target_component = MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
        msgRTL.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        msgRTL.autocontinue = 1;

        // add Missions into HashMap 'items'
        items.put(0, msgHome);
        items.put(1, msgTakeOff);
        items.put(2, msgFirst);
        items.put(3, msgSecond);
        items.put(4, msgRTL);

        Alert alert = AlertDialog.showNoButton("Mission Alert", "Mission Uploading....");
        uav.addMavLinkMessageListener(
                msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK,
                new UAV.MAVLinkMessageListener() {
                    @Override
                    public void receive(MAVLinkMessage message) {
                        msg_mission_ack msg = (msg_mission_ack) message;
                        if(msg.type == 0){
                            closeInfoDialog(alert);
                            closeInfoLabel("Mission Uploaded");
                            UAV.removeMavLinkMessageListener(msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK, this);
                        }else {
                            uav.sendMissionCount(items);
                        }
                    }
                }
        );
        uav.sendMissionCount(items);
    }


    public void handleBtnMissionMake(ActionEvent e){
        UAV uav = Network.getUAV();
        if(uav == null){
            AlertDialog.showOkButton("Alert", "UAV not Connected");
            return;
        } else if(uav.homeLat == 0.0){
            AlertDialog.showOkButton("Alert", "Home Location not Set. Arm Once");
            return;
        }

        jsproxy.call("missionMake");
    }


    public void closeInfoLabel(String info) {
        if(infoLabelThread != null){
            infoLabelThread.interrupt();
        }
        infoLabelThread = new Thread(){
            @Override
            public void run() {
                try {
                    Platform.runLater(() -> lblInfo.setText(info));
                    Thread.sleep(1500);
                    Platform.runLater(() -> {
                        lblInfo.setText("");
                        infoLabelThread = null;
                    });
                } catch (InterruptedException e) { }
            }
        };
        infoLabelThread.setDaemon(true);
        infoLabelThread.start();
    }

    public void closeInfoDialog(Alert alert) {
        if(infoThread != null){
            alert.close();
            infoThread.interrupt();
        }
        infoThread = new Thread(){
            @Override
            public void run() {
                try {
                    Thread.sleep(1500);
                    Platform.runLater(() -> {
                        alert.close();
                        infoThread = null;
                    });
                } catch (InterruptedException e) { }
            }
        };
        infoThread.setDaemon(true);
        infoThread.start();
    }


    public void javascriptLog(String level, String methodName, String message){
        if(level.equals("ERROR")){
            logger.error(methodName + ": " +message);
        }else if(level.equals("INFO")) {
            logger.info(methodName + ": " +message);
        }
    }

    public void javascriptManual(double manualLat, double manualLng, double manualAlt){
        UAV uav = Network.getUAV();
        if(uav != null){
            uav.sendSetPositionTargetGlobalInt(manualLat, manualLng, manualAlt);
        }
    }

}
