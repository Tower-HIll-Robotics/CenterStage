package org.firstinspires.ftc.teamcode.autonomous;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="MainAutonomous", group="")
//@Disabled
public class mainAutonomous extends LinearOpMode {
    // Declare OpMode members.
    private DcMotorEx FRM = null; // V2
    private DcMotorEx BRM = null; // V4
    private DcMotorEx FLM = null; // V1
    private DcMotorEx BLM = null; // V3
    private DcMotorEx wheelMotor = null;
    private DcMotorEx rampMotor = null;
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;

    // main timer
    private ElapsedTime runtime = new ElapsedTime();

    // time to press start after initialization
    double starting_time = 0;

    //power variables

    // placement variables
    boolean modeSelected = false;
    // 0 means not selected, 1 is blue, -1 is red
    double blue = 0;
    // values "left", and "right"
    String side = "none";

    double FRPower, BRPower, FLPower, BLPower;
    double speed = 0.5;
    double turningPower = 0.4; // 0.22
    double initialTurningVelocity = 200; // 150
    double turningVelocity;
    double errorMargin = 0.5; // degrees
    double autoPower = 0.15;
    double theoreticalAngle;

    // object detection cases (Left perspective looking at field from starting point)
    String spikePosition = "left";
    private TfodProcessor tfod;
    double max_index = -1;
    // private static final String TFOD_MODEL_ASSET = "";

    // Gamepad previousGamePad1 = new Gamepad();
    // Gamepad currentGamePad1 = new Gamepad();

    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "NewRedBallModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/NewRedBallModell.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "redball",
    };
    private VisionPortal visionPortal;
    double maxConfidence;

    @Override
    public void runOpMode() {

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();

        // left dpad is left
        // right dpad is right
        // up dpad is blue
        // down dpad is red
        while (!modeSelected) {
            previousGamePad1.copy(currentGamePad1);
            currentGamePad1.copy(gamepad1);
            telemetry.addData("Side", side);
            telemetry.addData("Color", blue);
            telemetry.addData("Status", "Select side and color using dpad");
            telemetry.update();

            if (currentGamePad1.dpad_up && blue == 0) {
                blue = 1;
            }
            else if (currentGamePad1.dpad_down && blue == 0){
                blue = -1;
            }

            if (currentGamePad1.dpad_left && side == "none") {
                side = "left";
            }
            else if (currentGamePad1.dpad_right && side == "none") {
                side = "right";
            }

            if (blue != 0 && side != "none") {
                modeSelected = true;
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initTfod();
        telemetry.addData("Status", "Tfod Initialized");
        telemetry.update();

        while (!opModeIsActive()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        waitForStart();
        visionPortal.close();

        // initialization time
        starting_time = runtime.time();

        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");
        wheelMotor = hardwareMap.get(DcMotorEx.class, "wheelMotor");
        rampMotor = hardwareMap.get(DcMotorEx.class, "rampMotor");


        // Setting parameters for imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        theoreticalAngle = getAngle();

        FLM.setDirection(DcMotorEx.Direction.REVERSE);
        FRM.setDirection(DcMotorEx.Direction.REVERSE);

        // leftIntake.setDirection(CRServo.Direction.REVERSE);

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rampMotor.setPower(-0.5);
        sleep(1000);
        rampMotor.setPower(0);

        if (blue == -1) {
            if (spikePosition == "right") {
                RedRightSpike();
            }
            else if (spikePosition == "center") {
                RedCenterSpike();
            }
            else {
                RedLeftSpike();
            }

            if (side == "left") {
                runStraight(60);
            }
        }
        else {
            if (spikePosition == "right") {
                BlueRightSpike();
            }
            else if (spikePosition == "center") {
                BlueCenterSpike();
            }
            else {
                BlueLeftSpike();
            }

            if (side == "right") {
                runStraight(110);
            }
        }
        // run a path based on sleeve recognition
    }

    // 132 cm is 1 foot
    // counter-clockwise is positive
    // x ticks - 60.96 cm

    private void dropIntakePixel() {
        wheelMotor.setPower(1);
        sleep(3000);
        wheelMotor.setPower(0);
    }
    private void RedRightSpike() {
        runStraight(65);
        sleep(1000);

        turn(-90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        turn(180);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        runStraight(60);
        strafe(20, true);
    }

    private void RedCenterSpike() {
        runStraight(58);
        sleep(1000);
        runStraight(7);
        sleep(1000);

        turn(90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        runStraight(65);
    }

    private void RedLeftSpike() {
        runStraight(65);
        sleep(1000);

        turn(90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        // Need to fix this part
        runStraight(-10);
        sleep(1000);
        runStraight(270);
    }

    private void BlueRightSpike() {
        runStraight(65);
        sleep(1000);

        turn(-90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);
        dropIntakePixel();

        turn(180);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        runStraight(60);
        dropIntakePixel();
    }

    private void BlueCenterSpike() {
        runStraight(58);
        sleep(1000);
        dropIntakePixel();
        runStraight(7);
        sleep(1000);

        turn(90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);

        runStraight(65);
        dropIntakePixel();
    }

    private void BlueLeftSpike() {
        runStraight(65);
        sleep(1000);

        turn(90);
        sleep(1000);
        turn(0);
        sleep(1000);
        turn(0);
        sleep(1000);
        dropIntakePixel();

        // Need to fix this part
        runStraight(-10);
        sleep(1000);
        runStraight(70);
        dropIntakePixel();
    }




    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * 4.94);
        // return (int) (DistanceCM * 21.32 * 1);
        // for original programming bot motors,
        // - 300 ticks is 2 feet/60.96 for new robot - 4.94 ticks is 1 cm
        // - 1300 ticks is 2 feet/60.96 cm old robot
    }

    public void runStraight(double centimeters) {
        int ticks = CMtoTicks(centimeters);

        FRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        /*if (centimeters >= 0) {
            FRPower = autoPower;
            FLPower = autoPower;
            BRPower = autoPower;
            BLPower = autoPower;
        }
        else {
            FRPower = -autoPower;
            FLPower = -autoPower;
            BRPower = -autoPower;
            BLPower = -autoPower;
        }*/

        FRPower = autoPower;
        FLPower = autoPower;
        BRPower = autoPower;
        BLPower = autoPower;

        if (FRPower<0) {
            FRM.setTargetPosition(-ticks + FRM.getCurrentPosition());
            FLM.setTargetPosition(-ticks + FLM.getCurrentPosition());
            BRM.setTargetPosition(-ticks + BRM.getCurrentPosition());
            BLM.setTargetPosition(-ticks + BLM.getCurrentPosition());
        } else {
            FRM.setTargetPosition(ticks + FRM.getCurrentPosition());
            FLM.setTargetPosition(ticks + FLM.getCurrentPosition());
            BRM.setTargetPosition(ticks + BRM.getCurrentPosition());
            BLM.setTargetPosition(ticks + BLM.getCurrentPosition());
        }

        FLM.setPower(FLPower);
        FRM.setPower(FRPower);
        BLM.setPower(BLPower);
        BRM.setPower(BRPower);

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("FRM", FRPower);
        telemetry.addData("FLM",FLPower);
        telemetry.addData("BRM", BRPower);
        telemetry.addData("BLM", BLPower);
        telemetry.update();

        FRM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BLM.setPower(0);
    }

    public void strafe (double centimeters, boolean left){
        int ticks = CMtoTicks(centimeters);

        FRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if(left){
            FRPower = autoPower;
            FLPower = -autoPower;
            BRPower = -autoPower;
            BLPower = autoPower;

            FRM.setTargetPosition(ticks + FRM.getCurrentPosition());
            FLM.setTargetPosition(-ticks + FLM.getCurrentPosition());
            BRM.setTargetPosition(-ticks + BRM.getCurrentPosition());
            BLM.setTargetPosition(ticks + BLM.getCurrentPosition());
        }
        else{
            FRPower = -autoPower;
            FLPower = autoPower;
            BRPower = autoPower;
            BLPower = -autoPower;

            FRM.setTargetPosition(-ticks + FRM.getCurrentPosition());
            FLM.setTargetPosition(ticks + FLM.getCurrentPosition());
            BRM.setTargetPosition(ticks + BRM.getCurrentPosition());
            BLM.setTargetPosition(-ticks + BLM.getCurrentPosition());
        }
        FLM.setPower(FLPower);
        FRM.setPower(FRPower);
        BLM.setPower(BLPower);
        BRM.setPower(BRPower);

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("FRM", FRPower);
        telemetry.addData("FLM",FLPower);
        telemetry.addData("BRM", BRPower);
        telemetry.addData("BLM", BLPower);
        telemetry.update();

        FRM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BLM.setPower(0);
    }

    private double getAngle() {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
           We have to process the angle because the imu works in euler angles so the Z axis is
           returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
           180 degrees. We detect this transition and track the total cumulative angle of rotation. */

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void turn(double degrees){
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentAngle = getAngle();
        double targetAngle = theoreticalAngle + degrees;
        double motorPower = turningPower;
        double linearError = 0;

        while (currentAngle<(targetAngle-errorMargin) || currentAngle>(targetAngle+errorMargin))
        {

            if (Math.abs(targetAngle - currentAngle) < 5) {
                turningVelocity = 0.8 * initialTurningVelocity;
            }
            else {
                turningVelocity = initialTurningVelocity;
            }

            if (currentAngle<targetAngle-errorMargin) {
                FLM.setPower(motorPower * -1);
                BLM.setPower(motorPower * -1);
                FRM.setPower(-motorPower * -1);
                BRM.setPower(-motorPower * -1);

                FLM.setVelocity(-turningVelocity);
                BLM.setVelocity(-turningVelocity);
                FRM.setVelocity(turningVelocity);
                BRM.setVelocity(turningVelocity);
            }
            if (currentAngle>targetAngle+errorMargin) {
                FLM.setPower(-motorPower * -1);
                BLM.setPower(-motorPower * -1);
                FRM.setPower(motorPower * -1);
                BRM.setPower(motorPower * -1);

                FLM.setVelocity(turningVelocity);
                BLM.setVelocity(turningVelocity);
                FRM.setVelocity(-turningVelocity);
                BRM.setVelocity(-turningVelocity);
            }

            telemetry.addData("TARGET ANGLE", targetAngle);
            telemetry.addData("CURRENT ANGLE", getAngle());
            telemetry.update();

            currentAngle = getAngle();
        }

        theoreticalAngle += degrees;

        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        maxConfidence = 0;
        max_index = -1;
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            // first is the x
            // 484

            if ((recognition.getConfidence() > maxConfidence) && (recognition.getWidth() < 120) && (recognition.getHeight() < 120)) {
                telemetry.addData("Confident Detection", maxConfidence);
                maxConfidence = recognition.getConfidence();
                max_index = currentRecognitions.indexOf(recognition);
            }

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        if (max_index != -1) {
            Recognition max_recognition = currentRecognitions.get((int) max_index);
            double max_x = (max_recognition.getLeft() + max_recognition.getRight()) / 2;
            double max_y = (max_recognition.getTop() + max_recognition.getBottom()) / 2;

            if (max_x < 350 && max_x > 204) {
                spikePosition = "center";
            } else if (max_x < 530 && max_x > 421) {
                spikePosition = "right";
            }
        }
        // Add bounds
        // Center Spike Left: 204
        // Center Spike Right: 350

        // Right Spike Left: 421
        // Right Spike Right: 530
    }   // end method telemetryTfod()

    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

                // Add model asset here
                .setModelAssetName(TFOD_MODEL_ASSET)
                // May need default

                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true) // default is true
                // .setIsModelQuantized(true) // default is true
                .setModelInputSize(300) // default is 300
                .setModelAspectRatio(16.0 / 9.0) // default is 16.0 / 9.0
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f); // default is .75

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

}