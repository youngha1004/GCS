package uav.gcs.main;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import uav.gcs.hud.HudController;
import uav.gcs.network.UAV;
import uav.gcs.network.UAVTCP;

import java.io.IOException;

public class AppMain extends Application {

    private static Logger logger = LoggerFactory.getLogger(AppMain.class);
    public static AppMain instance;
    public static Stage primaryStage;
    public Scene scene;

    @Override
    public void start(Stage primaryStage) throws IOException {

        instance = this;
        AppMain.primaryStage = primaryStage;
        this.primaryStage.setOpacity(0.0);

        // fxml 에서 scene 얻기
        BorderPane root = FXMLLoader.load(RootController.class.getResource("root.fxml"));

        // scene 생성
        Scene scene = new Scene(root);
        scene.getStylesheets().add(AppMain.class.getResource("root.css").toExternalForm());

        // scene 을 stage 에 올리기
        primaryStage.setScene(scene);

        // stage 설정
        primaryStage.setTitle("UAV Ground Control Station");
        primaryStage.setResizable(false);
        primaryStage.show();

        this.primaryStage.setOpacity(1.0);

    }


    @Override
    public void stop() {
        logger.info("Close and Stop");
    }

    public static void main(String[] args){
        Application.launch(args);
    }
}
