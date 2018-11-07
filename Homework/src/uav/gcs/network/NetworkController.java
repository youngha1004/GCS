package uav.gcs.network;

import java.net.URL;
import java.util.ResourceBundle;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextField;
import javafx.stage.Stage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class NetworkController implements Initializable {
	private static Logger logger = LoggerFactory.getLogger(NetworkController.class);
	public static NetworkController instance;

	@FXML private TextField txtUdpLocalPort;
	@FXML private TextField txtTcpServerIP;
	@FXML private TextField txtTcpServerPort;
	@FXML private RadioButton radioUdp;
	@FXML private RadioButton radioTcp;
	
	@Override
	public void initialize(URL location, ResourceBundle resources) {
		instance = this;
		getNetwork();
	}
	
	private void getNetwork() {
		if(Network.networkType.equals("UDP")) {
			radioUdp.setSelected(true);
		} else if(Network.networkType.equals("TCP")) {
			radioTcp.setSelected(true);
		}

		txtUdpLocalPort.setText(Network.udpLocalPort);
		txtTcpServerIP.setText(Network.tcpServerIP);
		txtTcpServerPort.setText(Network.tcpServerPort);
	}
	
	private void setNetwork() {
		if(radioUdp.isSelected()) {
			Network.networkType = "UDP";
		} else if(radioTcp.isSelected()) {
			Network.networkType = "TCP";
		}
		Network.udpLocalPort = txtUdpLocalPort.getText();
		Network.tcpServerIP = txtTcpServerIP.getText();
		Network.tcpServerPort = txtTcpServerPort.getText();
		Network.save();
	}

	public void handleBtnApply(ActionEvent event) {
		setNetwork();
		Stage dialog = (Stage)txtUdpLocalPort.getScene().getWindow();
		dialog.close();
	}

	public void handleBtnCancel(ActionEvent event) {
		Stage dialog = (Stage)txtUdpLocalPort.getScene().getWindow();
		dialog.close();
	}
}
