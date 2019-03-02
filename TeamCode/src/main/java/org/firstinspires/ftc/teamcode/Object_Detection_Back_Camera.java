package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Object_Detection_Back_Camera", group = "Vuforia")

public class Object_Detection_Back_Camera extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYlEu/7/////AAABmXB1kirNm0vlrZa4DCCmkis6ZNJkEkHGNYjIfoKWcK+yxnJOhuC4Lw3B63L+Y5vrSoTsr1mEe6bvGcMR8Hg+v1Z1Cih0IrBRHdIfrrg6lfa723ft/unZOKgck3ftCj8gWuiM89d+A4smkenUI5P/HXMKMGKCk4xxv5of9YNSX8r4KFO8lD+bqYgnP+GVXzD/TwQo7Dqer3bf0HVbOqP6j6HREHAZdP6Idg/JwyRG8LSdC6ekTwogxCWsuWiaUhuC8uAQ4r/ZfJykZpXYCxhdcLwMM4OaUXkUAPuUenzxlL8MXkwOhsDfqiQNEfSB00BodWKq28EC6cc+Vsko8r9PreeU6jCYR4d84VK8uBFLGaJx";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        MineralPosition ret = null;
        tfod.activate();
        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 2) {
                    String label1 = "";
                    String label2 = "";
                    int top1 = -1; //top de la tel cand e vertical , deci left pt landscape
                    int top2 = -1;
                    int left1 = 100000; //top de la tel cand e vertical , deci buttom pt landscape
                    int left2 = 100000;
                    for (Recognition recognition : updatedRecognitions) {
                        if (left1 > (int) recognition.getLeft()) {
                            left2 = left1;
                            top2 = top1;
                            label2 = label1;

                            left1 = (int) recognition.getLeft();
                            top1 = (int) recognition.getTop();
                            label1 = recognition.getLabel();
                        } else if (left2 > (int) recognition.getLeft()) {
                            left2 = (int) recognition.getLeft();
                            top2 = (int) recognition.getTop();
                            label2 = recognition.getLabel();
                        }
                    }
                    if (top1 != -1 && top2 != -1) {
                        if (label1.equals(LABEL_GOLD_MINERAL) || label2.equals(LABEL_GOLD_MINERAL)) {
                            if (label1.equals(LABEL_GOLD_MINERAL)) {
                                if (top1 < top2) {
                                    ret = LEFT;
                                } else {
                                    ret = MIDDLE;
                                }
                            } else {
                                if (top2 < top1) {
                                    ret = LEFT;
                                } else {
                                    ret = MIDDLE;
                                }
                            }
                        } else {
                            ret = RIGHT;
                        }
                        break;
                    }
                    telemetry.addData("position : ", ret);
                }
                telemetry.update();
            }
        }
        tfod.shutdown();
        while (opModeIsActive()){
            telemetry.addData("position : ", ret);
            telemetry.update();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
