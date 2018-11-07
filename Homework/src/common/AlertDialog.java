package common;

import javafx.scene.control.Alert;
import javafx.scene.control.ButtonBar;
import javafx.scene.control.ButtonType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AlertDialog {
    private static Logger logger = LoggerFactory.getLogger(AlertDialog.class);

    public static Alert showOkButton(String title, String contentText){
        Alert alert = new Alert(Alert.AlertType.NONE, contentText, new ButtonType("close", ButtonBar.ButtonData.OK_DONE));
        alert.setTitle(title);
        alert.show();
        return alert;
    }


    public static Alert showNoButton(String title, String contentText){
        Alert alert = new Alert(Alert.AlertType.NONE, contentText);
        alert.setTitle(title);
        // close dialog when program calls close()
        alert.setResult(ButtonType.OK);
        alert.show();
        return alert;
    }
}
