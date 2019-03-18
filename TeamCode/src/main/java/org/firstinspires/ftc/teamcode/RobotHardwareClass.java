package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

//Class with all the robot hardware and mappings
//This is useful if you want to modify something related to hardware as you will have to modify it
//in just one file (this one)

public abstract class RobotHardwareClass extends LinearOpMode {
    //motoare roti
    protected DcMotor MotorFL = null;
    protected DcMotor MotorFR = null;
    protected DcMotor MotorBL = null;
    protected DcMotor MotorBR = null;

    //motoare mecanisme
    protected DcMotor MotorGlisieraL = null;
    protected DcMotor MotorGlisieraR = null;

    protected DcMotor MotorExtindere = null;
    protected DcMotor MotorRotirePerii = null;

    //servo
    protected Servo ServoSortareL = null;
    protected Servo ServoSortareR = null;

    protected Servo ServoBlocareL = null;
    protected Servo ServoBlocareR = null;

    protected Servo ServoPhone = null;
    protected Servo ServoTeamMarker = null;

    //senzori
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;
    BNO055IMU imuGyro;

    //vuforia stuff
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected static final String VUFORIA_KEY = "AYlEu/7/////AAABmXB1kirNm0vlrZa4DCCmkis6ZNJkEkHGNYjIfoKWcK+yxnJOhuC4Lw3B63L+Y5vrSoTsr1mEe6bvGcMR8Hg+v1Z1Cih0IrBRHdIfrrg6lfa723ft/unZOKgck3ftCj8gWuiM89d+A4smkenUI5P/HXMKMGKCk4xxv5of9YNSX8r4KFO8lD+bqYgnP+GVXzD/TwQo7Dqer3bf0HVbOqP6j6HREHAZdP6Idg/JwyRG8LSdC6ekTwogxCWsuWiaUhuC8uAQ4r/ZfJykZpXYCxhdcLwMM4OaUXkUAPuUenzxlL8MXkwOhsDfqiQNEfSB00BodWKq28EC6cc+Vsko8r9PreeU6jCYR4d84VK8uBFLGaJx";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;
    protected boolean FrontCamera = true; //false -> back ; true -> front


    //************
    //INITIALISE
    //************

    protected void initialise(boolean bIsDriver){
        //TODO : HARDWARE MAPPING

        //motoare roti
        MotorFL = hardwareMap.dcMotor.get("MotorFL");
        MotorFR = hardwareMap.dcMotor.get("MotorFR");
        MotorBL = hardwareMap.dcMotor.get("MotorBL");
        MotorBR = hardwareMap.dcMotor.get("MotorBR");

        //motoare mecanisme
        MotorGlisieraL = hardwareMap.dcMotor.get("MotorGlisieraL");
        MotorGlisieraR = hardwareMap.dcMotor.get("MotorGlisieraR");

        MotorExtindere = hardwareMap.dcMotor.get("MotorExtindere");
        MotorRotirePerii = hardwareMap.dcMotor.get("MotorRotirePerii");

        //servo
        ServoSortareL = hardwareMap.servo.get("ServoSortareL");
        ServoSortareR = hardwareMap.servo.get("ServoSortareR");

        ServoBlocareL = hardwareMap.servo.get("ServoBlocareL");
        ServoBlocareR = hardwareMap.servo.get("ServoBlocareR");

        ServoPhone = hardwareMap.servo.get("ServoPhone");
        ServoTeamMarker = hardwareMap.servo.get("ServoTeamMarker");

        //senzori
        RangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        RangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");

        //RangeL.setI2cAddress(RangeR.getI2cAddress());

        //TODO : INITIALIZARE PUTERE

        //motoare roti
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);

        //motoare mecanisme
        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);

        MotorExtindere.setPower(0);
        MotorRotirePerii.setPower(0);

        //INITIALIZARE POZITIE SERVO
        ServoSortareL.setPosition(0.5);
        ServoSortareR.setPosition(0.5);

        ServoBlocareL.setPosition(0.5);
        ServoBlocareR.setPosition(0.5);

        ServoPhone.setPosition(0.5);
        ServoTeamMarker.setPosition(0.5);

        //TODO : SETARE DIRECTII

        //motoare roti
        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        //motoare mecanisme
        MotorGlisieraL.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraR.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorExtindere.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorRotirePerii.setDirection(DcMotorSimple.Direction.FORWARD);

        //servo
        ServoSortareL.setDirection(Servo.Direction.FORWARD);
        ServoSortareR.setDirection(Servo.Direction.FORWARD);

        ServoBlocareL.setDirection(Servo.Direction.FORWARD);
        ServoBlocareR.setDirection(Servo.Direction.FORWARD);

        ServoPhone.setDirection(Servo.Direction.FORWARD);
        ServoTeamMarker.setDirection(Servo.Direction.FORWARD);

        //TODO : RESET ENCODER

        //motoare roti
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //motoare mecanisme
        MotorGlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorExtindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //TODO : SETARE ENCODER

        //motoare roti
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motoare mecanisme
        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorExtindere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO : SETARE BREAK

        //motoare roti
        MotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motoare mecanisme
        MotorGlisieraL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorExtindere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO : VUFORIA

        if(!bIsDriver) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            if (FrontCamera) {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            } else {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            }
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }


        //TODO : GYRO

        imuGyro = hardwareMap.get(BNO055IMU.class, "imu");
    }
}
