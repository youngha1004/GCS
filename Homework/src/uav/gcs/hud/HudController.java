package uav.gcs.hud;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.geometry.VPos;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.CycleMethod;
import javafx.scene.paint.RadialGradient;
import javafx.scene.paint.Stop;
import javafx.scene.shape.ArcType;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.TextAlignment;
import uav.gcs.network.UAV;

import java.net.URL;
import java.time.LocalTime;
import java.util.ResourceBundle;

public class HudController implements Initializable, Hud {
    public static HudController instance;
    private Thread infoThread;
    private Thread warningThread;

    @FXML private StackPane stackPane;

    // Canvas layers
    @FXML private Canvas layer1;
    @FXML private Canvas layer2;
    @FXML private Canvas layer3;
    @FXML private Canvas layer4;
    @FXML private Canvas layer5;
    @FXML private Canvas layerWarning;

    @FXML private ImageView battery;
    @FXML private Label lblSystemStatus;
    @FXML private Label lblAirSpeed;
    @FXML private Label lblGroundSpeed;
    @FXML private Label lblArmed;
    @FXML private Label lblMode;
    @FXML private Label lblGpsFixed;
    @FXML private Label lblBatteryVoltage;
    @FXML private Label lblBatteryCurrent;
    @FXML private Label lblBatteryLevel;
    @FXML private Label lblTime;
    @FXML private Label lblInfo;
    @FXML private Label lblWarning;


    // constant double fields for minimum width & height
    public static final double MIN_WIDTH =  70 * 7;
    public static final double MIN_HEIGHT = 70 * 6;

    // Canvas' 2D GraphicsContext
    private GraphicsContext context1;
    private GraphicsContext context2;
    private GraphicsContext context3;
    private GraphicsContext context4;
    private GraphicsContext context5;
    private GraphicsContext contextWarning;

    // Drone's roll, pitch and yaw
    private double roll;
    private double pitch;
    private double yaw;
    private double tempRoll;
    private double tempPitch;

    // pitch origin and distance
    private double translateX, translateY;
    private double pitchDistance;

    // roll circle diameter
    private double diameter;

    // roll, pitch, yaw image circle diameter
    private double diameterSmall;

    // roll, pitch, yaw image origin
    private double diameterRollX, diameterRollY;
    private double diameterPitchX, diameterPitchY;
    private double diameterYawX, diameterYawY;

    // roll, pitch, yaw image size
    private double pathScaleRoll, pathScalePitch, pathScaleYaw;
    private String rollPath, pitchPath, yawPath;

    // Drone's altitude
    private double alt;
    private double altWidth, altHeight;
    private double altDistance;

    // warningSignal
    private RadialGradient warning;
    private int count;
    private boolean red;

    // font size for text
    private Font resizeFont;
    private Font resizeDigit;
    private Font resizeDigit_yaw;
    private double resizeWidth;

    // stroke width for lines
    private double lineWidth;

    @Override
    public void initialize(URL location, ResourceBundle resources) {
        HudController.instance = this;

        resizeFont = Font.font("Arial", FontWeight.BOLD, 13.0);
        resizeDigit = Font.font("Arial", FontWeight.BOLD, 15.0);

        stackPane.widthProperty().addListener(
                (ObservableValue<? extends Number> observable, Number oldValue, Number newValue) -> {
                    resizeWidth = newValue.doubleValue();
                    if(resizeWidth > HudController.MIN_WIDTH) {
                        layer1.setWidth(resizeWidth);
                        layer1.setHeight(resizeWidth * 6 / 7 );
                        layer2.setWidth(resizeWidth);
                        layer2.setHeight(resizeWidth * 6 / 7 );
                        layer3.setWidth(resizeWidth);
                        layer3.setHeight(resizeWidth * 6 / 7 );
                        layer4.setWidth(resizeWidth);
                        layer4.setHeight(resizeWidth * 6 / 7 );
                        layer5.setWidth(resizeWidth);
                        layer5.setHeight(resizeWidth * 6 / 7 );
                        layerWarning.setWidth(resizeWidth);
                        layerWarning.setHeight(resizeWidth * 6 / 7 );
                        resizeFont = Font.font("Arial", FontWeight.BOLD, 13.0 + (resizeWidth - HudController.MIN_WIDTH) / 30);
                        resizeDigit_yaw = Font.font("Arial", FontWeight.BOLD, 15.0 + (resizeWidth - HudController.MIN_WIDTH) / 30);
                        resizeDigit = Font.font("Arial", FontWeight.BOLD, 15.0 + (resizeWidth - HudController.MIN_WIDTH) / 20);
                        lineWidth = 1.0 + ((resizeWidth - HudController.MIN_WIDTH) / 100);
                    } else{
                        stackPane.setPrefWidth(HudController.MIN_WIDTH);
                    }
                }
        );

        initCanvasLayer1();
        initCanvasLayer2();
        initCanvasLayer3();
        initCanvasLayer4();
        initCanvasLayer5();
        initLayerWarning();

        lblInfo.setFont(new Font(55));
        lblWarning.setFont(new Font(35));

        ViewLoop viewLoop = new ViewLoop();
        viewLoop.start();


        UAV.addArmStatusListener(
                (boolean armed) -> {
                    if(armed){
                        setInfo("ARMED");
                    }else{
                        setInfo("DISARMED");
                    }
                }
        );
    }

// contents not changed during drawing
    private void initCanvasLayer1(){
        // get Hud's width and height
        layer1.setWidth(stackPane.getPrefWidth());
        layer1.setHeight(stackPane.getPrefHeight());

        context1 = layer1.getGraphicsContext2D();

        diameter = 250 * layer1.getWidth() / HudController.MIN_WIDTH ;
    }

    private void initCanvasLayer2() {
        // get Hud's width and height
        layer2.setWidth(stackPane.getPrefWidth());
        layer2.setHeight(stackPane.getPrefHeight());

        context2 = layer2.getGraphicsContext2D();
        pitchDistance = layer2.getWidth() / 120;
    }

    private void initCanvasLayer3(){
        // get Hud's width and height
        layer3.setWidth(stackPane.getPrefWidth());
        layer3.setHeight(stackPane.getPrefHeight());

        context3 = layer3.getGraphicsContext2D();
    }

    private void initCanvasLayer4(){
        // get Hud's width and height
        layer4.setWidth(stackPane.getPrefWidth());
        layer4.setHeight(stackPane.getPrefHeight());

        context4 = layer4.getGraphicsContext2D();

        rollPath = "M2.87 -6.31L75 5.98L74.99 7.64L38.77 4.29L38.77 7.61L38.88 7.62L40.07 8.04L41.11 8.71L41.98 9.59L42.65 10.63L43.07 11.82L43.22 13.12L43.07 14.41L42.65 15.6L41.98 16.65L41.11 17.52L40.07 18.19L38.88 18.61L37.6 18.76L36.31 18.61L35.12 18.19L34.08 17.52L33.21 16.65L32.54 15.6L32.12 14.41L31.97 13.12L32.12 11.82L32.54 10.63L33.21 9.59L34.08 8.71L35.12 8.04L35.8 7.8L35.8 4.01L10.11 1.64L10.19 1.85L10.47 3.17L10.56 4.54L10.47 5.92L10.19 7.24L9.74 8.49L9.13 9.66L8.38 10.74L7.49 11.71L6.48 12.57L5.36 13.3L4.15 13.89L2.86 14.32L1.49 14.59L1.44 14.6L1.44 17.28L1.58 17.31L1.69 17.38L1.77 17.5L1.8 17.64L1.77 17.79L1.69 17.9L1.58 17.98L1.43 18.01L0.83 18.01L0.83 19.38L0.91 19.32L1.25 19.25L2.25 19.25L2.58 19.32L2.86 19.51L3.05 19.78L3.12 20.12L3.12 23.67L3.05 24L2.86 24.28L2.58 24.47L2.25 24.54L1.25 24.54L0.91 24.47L0.64 24.28L0.45 24L0.38 23.67L0.38 20.14L-0.42 20.14L-0.42 23.6L-0.49 23.94L-0.68 24.22L-0.95 24.4L-1.29 24.47L-2.28 24.47L-2.62 24.4L-2.9 24.22L-3.09 23.94L-3.15 23.6L-3.15 20.06L-3.09 19.72L-2.9 19.44L-2.62 19.26L-2.28 19.19L-1.29 19.19L-0.95 19.26L-0.91 19.29L-0.91 18.01L-1.76 18.01L-1.9 17.98L-2.02 17.9L-2.09 17.79L-2.12 17.64L-2.09 17.5L-2.02 17.38L-1.9 17.31L-1.76 17.28L-1.52 17.28L-1.52 14.56L-2.72 14.32L-4.01 13.89L-5.23 13.3L-6.34 12.57L-7.35 11.71L-8.24 10.74L-8.99 9.66L-9.6 8.49L-10.05 7.24L-10.33 5.92L-10.42 4.54L-10.33 3.17L-10.05 1.85L-9.93 1.52L-35.64 4.06L-35.64 7.8L-34.96 8.04L-33.92 8.71L-33.05 9.59L-32.38 10.63L-31.96 11.82L-31.81 13.12L-31.96 14.41L-32.38 15.6L-33.05 16.65L-33.92 17.52L-34.96 18.19L-36.15 18.61L-37.43 18.76L-38.72 18.61L-39.91 18.19L-40.95 17.52L-41.82 16.65L-42.49 15.6L-42.91 14.41L-43.06 13.12L-42.91 11.82L-42.49 10.63L-41.82 9.59L-40.95 8.71L-39.91 8.04L-38.72 7.62L-38.61 7.61L-38.61 4.35L-75 7.94L-74.84 5.83L-3.21 -6.17L-1.52 -30.59L-0.04 -31.65L1.44 -30.59L2.87 -6.31Z";
        pitchPath = "M75.39 3.49C74.29 4.29 71.02 5.48 66.93 6.48C65 6.94 62.89 7.37 60.74 7.68C54.02 8.67 46.13 8.68 42.61 8.74C39.09 8.8 27.82 8.21 26.77 8.74C25.71 9.27 26.2 9.55 25.54 9.94C24.88 10.34 21.95 10.08 20.25 9.62C18.55 9.16 19.28 9.11 18.55 8.74C17.82 8.38 -17.04 8.62 -21.05 8.5C-25.06 8.38 -28.46 7.53 -31.98 6.92C-35.51 6.32 -67.03 -2.64 -67.69 -3.04C-68.14 -3.31 -68.43 -3.61 -68.57 -3.95L-68.95 -6.58C-69.07 -7.01 -69.01 -7.23 -68.77 -7.23C-68.41 -7.23 -66.36 -7.05 -64.78 -7.17C-63.72 -7.25 -63.53 -7.57 -64.19 -8.14L-72.67 -28.9L-63.93 -28.9C-53.77 -19.85 -46.91 -13.75 -43.35 -10.62C-38 -5.92 -31.14 -6.11 -29.12 -5.91C-28.93 -5.89 -25.59 -5.88 -19.09 -5.87C8.5 -5.77 24.27 -5.82 28.2 -6.03C34.11 -6.35 37.17 -7.25 43.08 -7.57C48.99 -7.88 61.27 -7.58 63.68 -6.78C65.28 -6.25 66.31 -5.45 66.76 -4.39C66.17 -4.44 67.88 -3.77 71.88 -2.37C75.89 -0.97 77.68 0.29 77.27 1.39C77.09 2.01 76.46 2.71 75.39 3.49Z";
        yawPath = "M-4.1 31.52L-4.27 28.34L-4.43 25.24L-4.58 22.22L-4.73 19.3L-4.88 16.46L-5.02 13.7L-5.15 11.04L-5.23 9.49L-5.53 9.49L-50 29.66L-50 19.03L-6.4 -14.5L-6.46 -15.78L-6.53 -17.24L-6.59 -18.61L-6.65 -19.89L-6.7 -21.09L-6.74 -22.21L-6.79 -23.23L-6.82 -24.17L-6.85 -25.03L-6.88 -25.79L-6.9 -26.47L-6.92 -27.07L-6.93 -27.58L-6.94 -28L-6.94 -28.57L-6.95 -29.14L-6.94 -29.72L-6.94 -30.3L-6.93 -30.88L-6.91 -31.46L-6.89 -32.03L-6.86 -32.6L-6.82 -33.17L-6.77 -33.74L-6.71 -34.29L-6.64 -34.84L-6.56 -35.38L-6.47 -35.91L-6.36 -36.43L-6.24 -36.94L-6.11 -37.44L-5.96 -37.92L-5.79 -38.39L-5.61 -38.84L-5.41 -39.27L-5.19 -39.68L-4.95 -40.08L-4.69 -40.45L-4.41 -40.8L-4.11 -41.13L-3.79 -41.43L-3.44 -41.71L-3.07 -41.96L-2.67 -42.19L-2.25 -42.39L-1.8 -42.55L-1.33 -42.69L-0.83 -42.79L-0.29 -42.87L0.27 -42.9L0.83 -42.89L1.36 -42.83L1.85 -42.71L2.32 -42.53L2.76 -42.31L3.16 -42.05L3.55 -41.74L3.9 -41.39L4.23 -41.01L4.54 -40.59L4.82 -40.14L5.08 -39.67L5.31 -39.17L5.53 -38.65L5.73 -38.11L5.9 -37.55L6.06 -36.98L6.21 -36.4L6.33 -35.81L6.44 -35.22L6.54 -34.63L6.63 -34.04L6.7 -33.45L6.76 -32.88L6.81 -32.31L6.85 -31.75L6.88 -31.22L6.91 -30.7L6.93 -30.2L6.94 -29.73L6.95 -29.29L6.95 -28.88L6.96 -28.5L6.96 -28.16L6.96 -27.86L6.96 -27.6L6.95 -27.4L6.95 -27.09L6.94 -26.69L6.92 -26.19L6.9 -25.59L6.88 -24.9L6.85 -24.1L6.82 -23.21L6.78 -22.22L6.74 -21.13L6.7 -19.95L6.65 -18.66L6.6 -17.28L6.54 -15.8L6.48 -14.22L6.48 -14.09L50 19.03L50 28.8L5.54 9.49L5.53 9.62L5.42 12.38L5.31 15.23L5.19 18.18L5.06 21.23L4.93 24.38L4.8 27.63L4.67 30.97L4.52 34.42L4.38 37.96L4.23 41.59L4.45 41.59L16.59 50.49L17.68 57.43L0.11 53.09L-16.59 57.43L-16.59 50.49L-3.58 41.59L-3.76 38.15L-3.93 34.79L-4.1 31.52Z";
    }

    private void initCanvasLayer5(){
        // get Hud's width and height
        layer5.setWidth(stackPane.getPrefWidth());
        layer5.setHeight(stackPane.getPrefHeight());

        context5 = layer5.getGraphicsContext2D();
        altWidth = 65 * layer5.getWidth() / HudController.MIN_WIDTH ;
        altHeight = 170 * layer5.getHeight() / HudController.MIN_WIDTH;
        altDistance = altHeight / 30;
    }

    private void initLayerWarning() {
        contextWarning = layerWarning.getGraphicsContext2D();
        layerWarning.setWidth(stackPane.getPrefWidth());
        layerWarning.setHeight(stackPane.getPrefHeight());
        warning = new RadialGradient(0,0 , 0.5, 0.5, 0.5,
                true, CycleMethod.NO_CYCLE,new Stop(0.8, Color.rgb(0,0,0,0.4)),
                new Stop(0.95, Color.rgb(80,0,0,0.8)), new Stop(1,Color.DARKRED));
    }

// contents changing during drawing
    private void layer1Draw(){
        double width = layer1.getWidth();
        double height = width * 6 / 7;

        diameter = 250 * layer1.getWidth() / HudController.MIN_WIDTH ;

        // clear canvas
        context1.clearRect(0,0, width, height);

        context1.translate(width/2, height/2);

        // draw roll arc & lines
        context1.setStroke(Color.WHITE);
        context1.setLineWidth(lineWidth);

        context1.strokeLine(-width/2, 0, -diameter/2, 0);
        context1.strokeLine(diameter/2, 0, width/2, 0);

        context1.strokeArc(-diameter /2, -diameter /2, diameter, diameter, 135, 90, ArcType.OPEN);
        context1.strokeArc(-diameter /2, -diameter /2, diameter, diameter, 315, 90, ArcType.OPEN);

        context1.setFill(Color.WHITE);
        context1.setFont(resizeDigit);
        context1.rotate(-45);
        for(int i = 0; i < 7; i++){
            context1.strokeLine(-diameter /2 - 5 * diameter / 180, 0, -diameter /2 + 5 * diameter / 180, 0);
            if(i>3) {
                context1.fillText(Integer.toString(-45 + 15 * i), -diameter / 2 - 25 * diameter / 180, 0);
            } else if(i == 3){
                context1.fillText(Integer.toString(0), -diameter / 2 - 15 * diameter / 180, 0);
                context1.fillText(Integer.toString(0), diameter / 2 + 10 * diameter / 180, 0);
            } else{
                context1.fillText(Integer.toString(45 - 15 * i), diameter / 2 + 10 * diameter / 180, 0);
            }
            context1.rotate(15);
        }
        context1.rotate(75);
        for(int i = 0; i < 7; i++){
            context1.strokeLine(-diameter /2 - 5 * diameter / 180, 0, -diameter / 2 + 5 * diameter / 180, 0);
            context1.rotate(15);
        }
        context1.rotate(-240);

        // draw the red arc
        context1.setStroke(Color.rgb(0xff, 0x00, 0x00, 0.8));
        context1.setLineWidth(lineWidth * 4);
        context1.strokeArc(-diameter/2, -diameter/2, diameter, diameter, -tempRoll, tempRoll, ArcType.OPEN);
        context1.strokeArc(-diameter/2, -diameter/2, diameter, diameter, 180-tempRoll, tempRoll, ArcType.OPEN);

        // draw circle center line
        tempRoll = roll;
        context1.rotate(tempRoll);
        context1.setStroke(Color.WHITE);
        context1.setLineWidth(lineWidth);
        context1.strokeLine(-diameter/2, 0, diameter/2, 0);

        // reset rotation
        context1.rotate(-tempRoll);

        // reset translated origin
        context1.translate(-width/2, -height/2);

    }

    private void layer2Draw() {
        double width = layer2.getWidth();
        double height = width * 6 / 7;

        pitchDistance = height / 2 / 60;

        // 뷰 지우기
        context2.clearRect(0, 0, width, height);

        //pitch line
        context2.setStroke(Color.RED);
        context2.setFill(Color.RED);
        context2.setLineWidth(lineWidth * 2);
        context2.strokeLine(width/2 , height / 2 + 10, width/2, height / 2 - 10);
        context2.strokeLine(-10 + width/2 , height / 2, +10 + width/2, height / 2);
        context2.fillText(String.valueOf((int)tempPitch),-80,-(pitchDistance*tempPitch));
        context2.setStroke(Color.WHITE);
        context2.setFill(Color.WHITE);

        // translate origin
        tempPitch = pitch;
        double x = pitchDistance * tempPitch * Math.sin(Math.toRadians(tempRoll));
        translateX = width / 2 - x;
        translateY = height / 2 + pitchDistance * tempPitch - pitchDistance * tempPitch *( 1- Math.cos(Math.toRadians(tempRoll)) );
        context2.translate(translateX, translateY);

        // rotate
        tempRoll = roll;
        context2.rotate(tempRoll);
        context2.setLineWidth(lineWidth);
        context2.setLineDashes(1, lineWidth * 6);
        context2.strokeLine(0, -pitchDistance * 120, 0, pitchDistance * 120);

        // draw center dot line
        context2.setStroke(Color.WHITE);
        context2.setFill(Color.WHITE);
        context2.strokeLine(-width, 0, width, 0);

        context2.setLineDashes(0, 0);
        context2.setFont(resizeDigit);
        context2.setTextAlign(TextAlignment.RIGHT);

        for (int i = 0; i <= (25 + tempPitch); i+=5 ) {
            if(i%2 == 0){
                double y = pitchDistance * i;
                context2.strokeLine(-40 , -y, -5 , -y);
                context2.strokeLine(-40 , -y, -45 , -y + pitchDistance * 2);
                context2.strokeLine(5, -y, 40, -y);
                context2.strokeLine(40, -y, 45, -y + pitchDistance * 2);
                context2.fillText(String.valueOf(i), -30 * width / HudController.MIN_WIDTH, -y - 5);
            }
        }
        for(int i = 10; i <= (25 - tempPitch); i+=5 ){
            if(i%2 == 0){
                double y = pitchDistance * i;
                context2.strokeLine(-40, y, -5, y);
                context2.strokeLine(-40, y, -45, y + pitchDistance * 2);
                context2.strokeLine(5, y, 40, y);
                context2.strokeLine(40, y, 45, y + pitchDistance * 2);
                context2.fillText(String.valueOf(-i), -30 * width / HudController.MIN_WIDTH, y - 5);
            }
        }

        context2.rotate(-tempRoll);
        context2.translate(-translateX, -translateY);
    }

    private void layer3Draw(){
        double width = layer3.getWidth();
        double height = width * 6 / 7;

        double line_width = width * 140 / HudController.MIN_WIDTH;

        context3.clearRect(0,0, width, height);
        context3.translate(width/2,height/35*2);

        //arc 그리기
        context3.setStroke(Color.WHITE);
        context3.setLineWidth(lineWidth);
        context3.strokeArc(-line_width,-line_width/5,line_width*2,line_width*2/5,+150,+240, ArcType.OPEN);

        //yaw 눈금선 그리기
        context3.setFill(Color.WHITE);
        context3.setTextAlign(TextAlignment.CENTER);
        context3.setTextBaseline(VPos.CENTER);
        context3.setFont(resizeDigit_yaw);

        for(int i=0; i<360; i+=15){
            double temp = yaw + i + 360;
            if( ( temp%360 < 60 ) || ( temp%360 > 300) ) {
                temp += 90;
                context3.strokeLine(
                        line_width * Math.cos(Math.toRadians(temp)),
                        line_width/5 * Math.sin(Math.toRadians(temp)),
                        line_width * Math.cos(Math.toRadians(temp)),
                        line_width/5 * Math.sin(Math.toRadians(temp)) - 10 -lineWidth*3);

                if (i == 0 || i == 360) {
                    context3.fillText("N",
                            line_width * Math.cos(Math.toRadians(temp)),
                            line_width/5 * Math.sin(Math.toRadians(temp)) - 20 -lineWidth*5);
                } else {
                    context3.fillText(checkDirection(360 - i),
                            line_width * Math.cos(Math.toRadians(temp)),
                            line_width/5 * Math.sin(Math.toRadians(temp)) - 20 -lineWidth*5);
                }
            }
        }

        // 현재 yaw 값 표시
        context3.setFill(Color.RED);
        context3.setFont(resizeDigit);
        context3.fillText(checkDirection((int)((yaw + 360) % 360)),0, line_width * 7 / 24 + 15);
        context3.setStroke(Color.RED);
        context3.strokeLine(0,line_width/5 + 15 ,0,line_width/5 - 15 - (resizeWidth - HudController.MIN_WIDTH) / 20);

        //원점복귀
        context3.translate(-width/2,-height/35*2);
    }

    private void layer4Draw(){
        double width = layer4.getWidth();
        double height = width * 6 / 7;

        diameterSmall = 85 * width / HudController.MIN_WIDTH;
        pathScaleRoll = 0.5 + (width - HudController.MIN_WIDTH) / 1000 ;
        pathScalePitch = 0.45 + (width - HudController.MIN_WIDTH) / 1000 ;
        pathScaleYaw = 0.55 + (width - HudController.MIN_WIDTH) / 1000 ;

        diameterRollX = 10 + diameterSmall/2;
        diameterRollY = height - diameterSmall/2 - 15;

        diameterPitchX = diameterRollX;
        diameterPitchY = diameterRollY - diameterSmall - 15;

        diameterYawX = diameterRollX + diameterSmall + 15;
        diameterYawY = diameterRollY;

        context4.clearRect(0,0, width, height);
        context4.setStroke(Color.WHITE);
        context4.setFill(Color.WHITE);
        context4.setLineWidth(lineWidth);

        // roll circle
        context4.translate(diameterRollX, diameterRollY);
        context4.strokeOval(-diameterSmall/2, -diameterSmall/2, diameterSmall, diameterSmall);

        for(int i = 0; i < 8; i++){
            context4.rotate(45);
            context4.strokeLine(0, diameterSmall * 9 / 20, 0, diameterSmall * 11 / 20);
        }
        // draw roll plane
        context4.rotate(-roll);
        context4.scale(pathScaleRoll, pathScaleRoll);
        context4.beginPath();
        context4.appendSVGPath(rollPath);
        context4.closePath();
        context4.fill();

        context4.scale(1/pathScaleRoll, 1/pathScaleRoll);
        context4.rotate(+roll);
        context4.translate(-diameterRollX, -diameterRollY);

        // pitch circle
        context4.translate(diameterPitchX, diameterPitchY);
        context4.strokeOval(-diameterSmall/2, -diameterSmall/2, diameterSmall, diameterSmall);

        for(int i = 0; i < 8; i++){
            context4.rotate(45);
            context4.strokeLine(0, diameterSmall * 9 / 20, 0, diameterSmall * 11 / 20);
        }
        // draw pitch plane
        context4.rotate(-pitch);
        context4.scale(pathScalePitch, pathScalePitch);
        context4.beginPath();
        context4.appendSVGPath(pitchPath);
        context4.closePath();
        context4.fill();

        context4.scale(1/pathScalePitch, 1/pathScalePitch);
        context4.rotate(+pitch);
        context4.translate(-diameterPitchX, -diameterPitchY);

        // yaw circle
        context4.translate(diameterYawX, diameterYawY);
        context4.strokeOval(-diameterSmall/2, -diameterSmall/2, diameterSmall, diameterSmall);

        for(int i = 0; i < 8; i++){
            context4.rotate(45);
            context4.strokeLine(0, diameterSmall * 9 / 20, 0, diameterSmall * 11 / 20);
        }
        // draw yaw plane
        context4.rotate(+yaw);
        context4.scale(pathScaleYaw, pathScaleYaw);
        context4.beginPath();
        context4.appendSVGPath(yawPath);
        context4.closePath();
        context4.fill();

        context4.scale(1/pathScaleYaw, 1/pathScaleYaw);
        context4.rotate(-yaw);
        context4.translate(-diameterYawX, -diameterYawY);

    }

    private void layer5Draw(){
        double width = layer5.getWidth();
        double height = width * 6 / 7;

        altWidth = 65 * layer5.getWidth() / HudController.MIN_WIDTH ;
        altHeight = 170 * layer5.getHeight() / HudController.MIN_WIDTH;
        altDistance = altHeight / 30;

        context5.setFont(resizeDigit);

        context5.clearRect(0, 0, width, height);

        context5.translate(width * 6 / 7, height * 4 / 7);

        context5.setStroke(Color.WHITE);
        context5.setFill(Color.WHITE);
        context5.setLineWidth(lineWidth * 2);
        context5.setTextAlign(TextAlignment.RIGHT);

        context5.strokeRect(0, 0, altWidth, altHeight);

        int intAlt = (int) Math.round(alt);
        int altY = -1;
        int altTxt = -1;

        if( (intAlt % 5) != 0 ) {
            if(intAlt > 0) {    // positive alt
                altY = (int) ((intAlt % 5) * altDistance);
                altTxt = intAlt - (intAlt % 5) + 3 * 5;
            } else{             // negative alt
                altY = (int) (altHeight + (intAlt % 5) * altDistance);
                altTxt = intAlt - (intAlt % 5) - 3 * 5;
            }

            if (altTxt % 2 == 0) {      // tenfold
                context5.strokeLine(0, altY, altWidth * 2 / 5, altY);
            } else {                    // fivefold
                context5.strokeLine(0, altY, altWidth * 1 / 5, altY);
            }

            context5.fillText(Integer.toString(altTxt), altWidth * 6 / 7, altY + 5);
        }

        for(int i = 1; i < 6; i++){
            altY = (int) (i * altHeight / 6 + (intAlt % 5) * altDistance);
            altTxt = intAlt - (intAlt % 5) - (i - 3) * 5;

            if (altTxt % 2 == 0) {      // tenfold
                context5.strokeLine(0, altY, altWidth * 2 / 5, altY);
            } else {                    // fivefold
                context5.strokeLine(0, altY, altWidth * 1 / 5, altY);
            }

            context5.fillText(Integer.toString(altTxt), altWidth * 6 / 7, altY + 5);
        }

        context5.setFill(Color.WHITE);
        context5.fillRect(altWidth / 7, altHeight/2 - altDistance * 5 / 2, altWidth * 5 / 6, altDistance * 5);
        context5.setFill(Color.BLACK);
        context5.fillText(Math.round(alt * 10) / 10.0 + " m", altWidth, altHeight/2 + altDistance);

        context5.translate(-width * 6 / 7, -height * 4 / 7);
    }

    private void layer6Draw() {

        lblSystemStatus.setFont(resizeFont);
        lblAirSpeed.setFont(resizeFont);
        lblGroundSpeed.setFont(resizeFont);
        lblArmed.setFont(resizeFont);
        lblMode.setFont(resizeFont);
        lblGpsFixed.setFont(resizeFont);
        lblBatteryVoltage.setFont(resizeFont);
        lblBatteryCurrent.setFont(resizeFont);
        lblTime.setFont(resizeFont);

        lblInfo.setFont(Font.font("Arial", FontWeight.BOLD, 25 + (resizeWidth - HudController.MIN_WIDTH) / 10));
        lblWarning.setFont(Font.font("Arial", FontWeight.BOLD, 20 + (resizeWidth - HudController.MIN_WIDTH) / 20));

        double resizedWidth = 65 + (stackPane.getWidth() - HudController.MIN_WIDTH) / 10;
        double resizedHeight = resizedWidth * 4 / 5;

        Image resizedImage = new Image(HudController.class.getResource("images/battery.png").toExternalForm());
        battery.setImage(resizedImage);
        battery.setFitWidth(resizedWidth);
        battery.setFitHeight(resizedHeight);

        lblBatteryLevel.setFont(resizeFont);
        lblBatteryLevel.setLayoutY(resizedHeight / 2);
    }

    private void layerWarningDraw(){

        contextWarning.setFill(warning);
        contextWarning.clearRect(0,0, stackPane.getWidth(), stackPane.getPrefHeight());
        contextWarning.fillRect(0,0, stackPane.getWidth(), stackPane.getPrefHeight());

        if( (pitch<-30) || (pitch>30) || (roll<-30) || (roll>30)){
            if(red){
                layerWarning.setOpacity(++count / 30);
                if(count > 20){
                    red = false;
                }
            }else{
                layerWarning.setOpacity(0.3 - --count / 30);
                if(count < 0){
                    red = true;
                }
            }
        }else{
            layerWarning.setOpacity(0);
        }
    }

    private String checkDirection(int yaw){
        switch (yaw){
            case 0:
            case 360: return "N";
            case 90: return "E";
            case 180: return "S";
            case 270: return "W";
            default: return String.valueOf(yaw);
        }
    }

    // Animation Timer 상속 class
    private class ViewLoop extends AnimationTimer{
        @Override
        public void handle(long now) {
            layer1Draw();
            layer2Draw();
            layer3Draw();
            layer4Draw();
            layer5Draw();
            layer6Draw();
            layerWarningDraw();

            setTime( LocalTime.now().getHour() + ":" + LocalTime.now().getMinute() + ":" + LocalTime.now().getSecond());
        }
    }

    // override implemented setter&getter methods
    @Override
    public void setRoll(double roll) {
        this.roll = roll;
    }

    @Override
    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    @Override
    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    @Override
    public void setAltitude(double altitude) {
        this.alt = altitude;
    }

    @Override
    public void setSystemStatus(String systemStatus) {
        Platform.runLater(()->this.lblSystemStatus.setText(systemStatus));
    }

    @Override
    public void setArmed(boolean armed) {
        Platform.runLater(()->{
            if(armed) {
                this.lblArmed.setText("ARMED");
            } else {
                this.lblArmed.setText("DISARMED");
            }
        });
    }

    @Override
    public void setBattery(double voltage, double current, double level) {
        Platform.runLater(()-> {
            this.lblBatteryVoltage.setText(voltage+" V");
            this.lblBatteryCurrent.setText(current+" A");
            this.lblBatteryLevel.setText((int)level + "%");
        });
    }

    @Override
    public void setGpsFixed(boolean gpsFixed) {
        Platform.runLater(()->{
            if(gpsFixed) {
                this.lblGpsFixed.setText("GPS Fixed");
            } else {
                this.lblGpsFixed.setText("GPS Not Fixed");
            }
        });
    }

    @Override
    public void setMode(String mode) {
        Platform.runLater(()->this.lblMode.setText(mode));
    }

    @Override
    public void setAirSpeed(double airSpeed) {
        Platform.runLater(()->this.lblAirSpeed.setText("AS: " + airSpeed + " m/s"));
    }

    @Override
    public void setGroundSpeed(double groundSpeed) {
        Platform.runLater(()->this.lblGroundSpeed.setText("GS: " + groundSpeed + "m/s"));
    }

    @Override
    public void setInfo(String info) {
        if(infoThread != null){
            infoThread.interrupt();
        }
        infoThread = new Thread(){
            @Override
            public void run() {
                try {
                    Platform.runLater(()-> HudController.this.lblInfo.setText(info));
                    Thread.sleep(3000);
                    Platform.runLater(()-> HudController.this.lblInfo.setText(""));
                } catch (InterruptedException e) { }
            }
        };
        infoThread.setDaemon(true);
        infoThread.start();
    }

    public void setWarining(String warining){

        if(warningThread != null){
            warningThread.interrupt();
        }
        warningThread = new Thread(){
            @Override
            public void run() {
                try {
                    Platform.runLater(()-> HudController.this.lblWarning.setText(warining));
                    Thread.sleep(2000);
                    Platform.runLater(()-> HudController.this.lblWarning.setText(""));
                } catch (InterruptedException e) { }
            }
        };
        warningThread.setDaemon(true);
        warningThread.start();
    }

    public void setTime(String time){
        Platform.runLater(()-> this.lblTime.setText(time));
    }

    public void changeHud(UAV uav){
        setArmed(uav.armed);
        setSystemStatus(uav.systemStatus);
        setMode(uav.mode);
        setRoll(-uav.roll);
        setPitch(uav.pitch);
        setYaw(uav.yaw);
        setAltitude(uav.alt);
        setBattery(uav.batteryVoltage, uav.batteryCurrent, uav.batteryRemaining);
        setGpsFixed(uav.gpsFixed);
        setAirSpeed(uav.airSpeed);
        setGroundSpeed(uav.groundSpeed);
    }

}