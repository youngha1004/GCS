package uav.gcs.main;

import com.MAVLink.enums.COPTER_MODE;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.concurrent.Worker;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.geometry.Rectangle2D;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.*;
import javafx.scene.media.Media;
import javafx.scene.media.MediaPlayer;
import javafx.scene.media.MediaView;
import javafx.scene.web.WebEngine;
import javafx.scene.web.WebView;
import javafx.stage.FileChooser;
import javafx.stage.Modality;
import javafx.stage.Stage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import uav.gcs.Mission.MissionController;
import uav.gcs.hud.Hud;
import uav.gcs.hud.HudController;
import uav.gcs.network.Network;
import uav.gcs.network.NetworkController;
import uav.gcs.network.UAV;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.Random;
import java.util.ResourceBundle;


public class RootController implements Initializable {
    private static Logger logger = LoggerFactory.getLogger(RootController.class);
    public static RootController instance;

    @FXML private BorderPane root;
    @FXML private HBox hudConnectionHBox, hudModeHBox;

    // HUD
    @FXML private StackPane hudStackPane;
    @FXML private TextField txtNewAlt;
    @FXML private Button btnConfig, btnConnect, btnArm;
    @FXML private Button btnTakeoff, btnLand, btnHome;
    @FXML private MediaView mediaViewBackground;

    // State Table & Front Camera
    @FXML private Button btnCamera;
    @FXML private TableView tableState;

    // Split
    @FXML SplitPane splitHudMissionPane;
    ObservableList<SplitPane.Divider> dividers;

    // Mission
    @FXML private StackPane centerPane;

    // State Table
    private ObservableList<State> states = FXCollections.observableArrayList();
    private State stateRoll, statePitch, stateYaw;
    //private int unit_counter;

    // Camera
    private Media mediaBelow;
    private Media mediaBackground;
    private MediaPlayer mediaPlayerBelow;
    private MediaPlayer mediaPlayerBackground;

    // Hud Instance
    private StackPane hud;

    @Override
    public void initialize(URL location, ResourceBundle resources) {

        RootController.instance = this;

        initSplitPaneDividers();

        try {
            initHudStackPane();
            initWebView();
        } catch (IOException e) { logger.error(e.toString()); return; }

        initHudBackgroundMediaView();

        initHudButtonVBox();
        initStateTableView();
        initHudFrontMediaView();

        addListener();

    }

    private void initWebView() throws IOException{
        HBox missionPane = FXMLLoader.load(MissionController.class.getResource("mission.fxml"));
        centerPane.getChildren().add(missionPane);
    }

    private void initSplitPaneDividers(){
        dividers = splitHudMissionPane.getDividers();
        dividers.get(0).setPosition(0.3);
    }

    private void initHudStackPane() throws IOException{
        hud = FXMLLoader.load(getClass().getResource("../hud/hud.fxml"));
        hudStackPane.getChildren().add(hud);

        btnArm.setDisable(false);

        hudStackPane.setPrefSize(HudController.MIN_WIDTH, HudController.MIN_HEIGHT);
        hudStackPane.setMinSize(Region.USE_PREF_SIZE, Region.USE_PREF_SIZE);
    }

    private void initHudBackgroundMediaView(){
        mediaBackground = new Media(RootController.class.getResource("../media/camera.mp4").toExternalForm());
        mediaPlayerBackground = new MediaPlayer(mediaBackground);
        mediaPlayerBackground.setAutoPlay(true);
        mediaPlayerBackground.setOnEndOfMedia( () -> { mediaPlayerBackground.stop(); mediaPlayerBackground.play(); });

        mediaViewBackground.setMediaPlayer(mediaPlayerBackground);
        mediaViewBackground.setFitWidth(hudStackPane.getPrefWidth());
        mediaViewBackground.setFitHeight(mediaViewBackground.getFitWidth() * 6 / 7 );
    }

    private void initHudButtonVBox(){
        hudConnectionHBox.setPrefWidth(hudStackPane.getWidth());
        hudConnectionHBox.setPrefHeight(30);
        hudModeHBox.setPrefWidth(hudStackPane.getWidth());
        hudModeHBox.setPrefHeight(30);
    }

    private void initStateTableView(){
        stateRoll = new State("ROLL", 0.0, 0);
        statePitch = new State("PITCH", 0.0, 0);
        stateYaw = new State("YAW", 0.0, 0);
        stateTableView(120, 120, 200);
    }

    private void initHudFrontMediaView(){
        mediaBelow = new Media(RootController.class.getResource("../media/camera.mp4").toExternalForm());
        mediaPlayerBelow = new MediaPlayer(mediaBelow);
        mediaPlayerBelow.setAutoPlay(true);

        tableState.setVisible(true);
    }

    private void addListener(){
        // divider change
        dividers.get(0).positionProperty().addListener(handleDivider);

        // hud connection
        btnConfig.setOnAction(handleConfiguration);
        btnConnect.setOnAction(handleConnection);
        btnArm.setOnAction(handleArm);

        // hud mode
        btnTakeoff.setOnAction(handleModeTakeOff);
        btnLand.setOnAction(handleModeLand);
        btnHome.setOnAction(handleModeRTL);

        // state table or camera selection
        btnCamera.setOnAction(handleTableImage);
    }

    private ChangeListener<Number> handleDivider =
            (ObservableValue<? extends Number> observable, Number oldValue, Number newValue) -> {
        
        hudStackPane.setMaxWidth(root.getWidth() * newValue.doubleValue());
        hudStackPane.setMaxHeight(hudStackPane.getWidth() * 6 / 7);

        //hudStackPane.setPrefWidth(hudStackPane.getMaxWidth());
        hudStackPane.setPrefHeight(hudStackPane.getMaxHeight());

        hud.setMinSize(hudStackPane.getMaxWidth(), hudStackPane.getMaxHeight());

        mediaViewBackground.setFitWidth(hudStackPane.getWidth());
        mediaViewBackground.setFitHeight(mediaViewBackground.getFitWidth() * 6 / 7);

        tableState.setPrefWidth(hudStackPane.getWidth());

        stateTableView(hudStackPane.getWidth() / 6, hudStackPane.getWidth() / 2, hudStackPane.getWidth() / 4);
    };

    private EventHandler<ActionEvent> handleTableImage =
            (ActionEvent event) -> {
                if(event.getSource() == btnCamera){
                    if (tableState.isVisible()) {
                        tableState.setVisible(false);
                    } else {
                        tableState.setVisible(true);
                    }
                }
    };

    private  EventHandler<ActionEvent> handleConfiguration =
            (ActionEvent event) -> {
                try {
                    Stage dialog = new Stage();
                    dialog.setTitle("Network Configuration");
                    dialog.initModality(Modality.APPLICATION_MODAL);
                    dialog.initOwner(AppMain.instance.primaryStage);
                    AnchorPane anchorPane = null;
                    anchorPane = FXMLLoader.load(NetworkController.class.getResource("Network.fxml"));
                    Scene scene = new Scene(anchorPane);
                    scene.getStylesheets().add(RootController.class.getResource("style_dark_dialog.css").toExternalForm());
                    dialog.setScene(scene);
                    dialog.setResizable(false);
                    dialog.show();
                } catch (IOException e) {
                    logger.info(e.toString());
                }
            };

    private EventHandler<ActionEvent> handleConnection =
            (ActionEvent event) -> {
                if(btnConnect.getText().equals("Connect")){
                    UAV uav = Network.createUAV();
                    uav.addConnectionListener(new UAV.ConnectionListener() {
                        @Override
                        public void connect(UAV uav) {
                            Platform.runLater(() -> {
                                btnConnect.setText("Disconnect");
                                btnArm.setDisable(false);
                            });
                            Thread thread = new Thread() {
                                @Override
                                public void run() {
                                    while (uav.connected){
                                        HudController.instance.changeHud(uav);
                                        RootController.instance.changeStatus(uav);
                                        MissionController.instance.changeStatus();
                                        try { Thread.sleep(200); } catch (InterruptedException e) { logger.info(e.toString()); }
                                    }
                                }
                            };
                            thread.setDaemon(true);
                            thread.start();
                        }

                        @Override
                        public void disconnect(UAV uav) {
                            Platform.runLater(() -> {
                                btnConnect.setText("Connect");
                                btnArm.setDisable(true);
                            });
                            uav.removeConnectionListener(this);
                        }
                    });

                    Thread thread = new Thread() {
                        @Override
                        public void run() {
                            uav.connect();
                        }
                    };
                    thread.setDaemon(true);
                    thread.start();
                } else{
                    Network.destroyUAV();
                }
    };

    public EventHandler<ActionEvent> handleArm =
            (ActionEvent event) -> {
                UAV uav = Network.getUAV();
                if( (uav != null) && uav.connected) {
                    if (btnArm.getText().equals("Arm")) {
                        uav.sendCmdComponentArmDisarm(true);
                    } else {
                        uav.sendCmdComponentArmDisarm(false);
                    }
                }
            };

    private EventHandler<ActionEvent> handleModeTakeOff =
            (ActionEvent event) -> {
                UAV uav = Network.getUAV();
                try {
                    float newAlt = Float.parseFloat(txtNewAlt.getText());
                    if(!uav.armed){
                        throw new Exception("Drone Not Armed");
                    }
                    uav.sendCmdNavTakeoff(newAlt);
                }catch (NumberFormatException e_num){
                    HudController.instance.setWarining("Empty Altitude");
                }catch (NullPointerException e_null){
                    HudController.instance.setWarining("Drone Not Connected");
                }catch (Exception e){
                    HudController.instance.setWarining(e.getMessage());
                }
    };

    private EventHandler<ActionEvent> handleModeLand =
            (ActionEvent event) -> {
                UAV uav = Network.getUAV();
                try {
                    uav.sendSetMode(COPTER_MODE.COPTER_MODE_LAND);
                }catch (NullPointerException e_null){
                    HudController.instance.setWarining("Drone Not Connected");
                }
            };


    private EventHandler<ActionEvent> handleModeRTL =
            (ActionEvent event) -> {
                UAV uav = Network.getUAV();
                try {
                    uav.sendSetMode(COPTER_MODE.COPTER_MODE_RTL);
                }catch (NullPointerException e_null){
                    HudController.instance.setWarining("Drone Not Connected");
                }
            };


    private void changeStatus(UAV uav){
        if( uav.connected ){
            if(uav.armed){
                Platform.runLater(() -> btnArm.setText("Disarm"));
            } else{
                Platform.runLater(() -> btnArm.setText("Arm"));
            }
        }
    }


    private void stateTableView(double width1, double width2, double width3){
        tableState.getColumns().clear();

        TableColumn<State, String> column1 = new TableColumn<>("NAME");
        TableColumn<State, Double> column2 = new TableColumn<>("VALUE");
        TableColumn<State, String> column3 = new TableColumn<>("UNIT");
        column1.setCellValueFactory(new PropertyValueFactory<>("name"));
        column2.setCellValueFactory(new PropertyValueFactory<>("value"));
        column3.setCellValueFactory(new PropertyValueFactory<>("unit"));
        column1.setPrefWidth(width1);
        column2.setPrefWidth(width2);
        column3.setPrefWidth(width3);
        tableState.getColumns().add(column1);
        tableState.getColumns().add(column2);
        tableState.getColumns().add(column3);

        states.clear();
        states.addAll(
                stateRoll,
                statePitch,
                stateYaw
        );
        tableState.setItems(states);
    }
}
