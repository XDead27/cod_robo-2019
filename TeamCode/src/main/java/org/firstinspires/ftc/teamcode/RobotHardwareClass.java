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

    //servo
    protected CRServo ContinuousServo = null;
    protected Servo FixedServo = null;
    protected Servo PhoneServo = null;
    protected Servo TeamMarkerServo = null;

    //senzori
    protected ModernRoboticsI2cColorSensor color = null;
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;
    protected ModernRoboticsI2cGyro gyro = null;
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
        //hardware mapping
        MotorFL = hardwareMap.dcMotor.get("MotorFL");
        MotorFR = hardwareMap.dcMotor.get("MotorFR");
        MotorBL = hardwareMap.dcMotor.get("MotorBL");
        MotorBR = hardwareMap.dcMotor.get("MotorBR");
        MotorGlisieraL = hardwareMap.dcMotor.get("MotorGlisieraL");
        MotorGlisieraR = hardwareMap.dcMotor.get("MotorGlisieraR");
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");
        PhoneServo = hardwareMap.servo.get("PhoneServo");
        TeamMarkerServo = hardwareMap.servo.get("TeamMarkerServo");

        //initializare putere
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);

        ContinuousServo.setPower(0);

        //initializare pozitie
        FixedServo.setPosition(0);
        PhoneServo.setPosition(0.5);
        TeamMarkerServo.setPosition(0.5);

        //setare directii
        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraL.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraR.setDirection(DcMotorSimple.Direction.REVERSE);

        ContinuousServo.setDirection(CRServo.Direction.FORWARD);
        FixedServo.setDirection(Servo.Direction.FORWARD);
        PhoneServo.setDirection(Servo.Direction.FORWARD);
        TeamMarkerServo.setDirection(Servo.Direction.FORWARD);


        //reset encoder
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //setare encoder
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setare cand power == 0
        MotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //vuforia
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


        //GYRO
        //
        //

        BNO055IMU.Parameters REVGyroParameters = new BNO055IMU.Parameters();

        REVGyroParameters.mode = BNO055IMU.SensorMode.IMU;
        REVGyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        REVGyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        REVGyroParameters.loggingEnabled = false;

        imuGyro = hardwareMap.get(BNO055IMU.class, "imu");

        if(!bIsDriver) {
            imuGyro.initialize(REVGyroParameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imuGyro.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imuGyro.getCalibrationStatus().toString());
            telemetry.update();
        }
    }
}
