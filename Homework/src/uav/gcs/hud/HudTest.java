package uav.gcs.hud;

import javafx.application.Application;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;

import java.io.IOException;

public class HudTest extends Application {

    @Override
    public void start(Stage primaryStage) {
        try {
            StackPane parent = FXMLLoader.load(HudController.class.getResource("hud.fxml"));
            parent.setStyle("-fx-background-color: black");
            Scene scene = new Scene(parent);

            primaryStage.heightProperty().addListener(
                    (ObservableValue<? extends Number> observable, Number oldValue, Number newValue)
                    -> primaryStage.setHeight(primaryStage.getWidth() * 6 / 7)
            );

            primaryStage.widthProperty().addListener(
                    (ObservableValue<? extends Number> observable, Number oldValue, Number newValue) -> {
                if(newValue.intValue() < HudController.MIN_WIDTH){
                    primaryStage.setWidth(HudController.MIN_WIDTH);
                }
                primaryStage.setHeight(primaryStage.getWidth() * 6 / 7);
            });

            primaryStage.setScene(scene);
            primaryStage.show();

        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void main(String[] args){
        Application.launch(args);
    }
}
