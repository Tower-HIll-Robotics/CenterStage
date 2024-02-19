package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="MainDrive", group="--")
// @Disabled
public class mainDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;

    private DcMotor wheelMotor = null;
    private DcMotor rampMotor = null;
    private DcMotor actuatorMotor = null;
    private DcMotor slide = null;
    // private DcMotor slide = null;
    // private CRServo boxRotator = null;
    private Servo boxOpener = null;
    private Servo airplaneLauncher = null;
    private CRServo boxRotator = null;
    double scaleMultiplier = 1;

    double FRPower, BRPower, FLPower, BLPower;

    double wheelMotorPower = 1;

    //set constants
    double directionMultiplier = 0.5;
    double intakePower = 1;
    double outtakePower = 1;

    //adjust this to driver preference, 1 is perfectly linear (no curve adjustement from default), larger values make the dip more extreme,
    //dont go below 1 unless its for fun
    double driver_scaling_constant = 2;

    // default value
    double slide_encoder_value = 0;

    // Setting up Slug Mode Parameters
    boolean slugMode = false;
    double slugMultiplier = 0.2;

    // set up imu for gyro
    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // get motors, linear slide, and wheel intakes from hardware map
        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");
        boxRotator = hardwareMap.get(CRServo.class,"boxRotator");
        boxOpener = hardwareMap.get(Servo.class, "boxOpener");
        wheelMotor = hardwareMap.get(DcMotorEx.class, "wheelMotor");
        // rampMotor = hardwareMap.get(DcMotorEx.class, "rampMotor");
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        slide = hardwareMap.get(DcMotorEx.class, "liftMotor");
        // actuatorMotor = hardwareMap.get(DcMotorEx.class, "actuatorMotor");

        //GamePads to save previous state of gamepad for button toggling
        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad2 = new Gamepad();
        Gamepad currentGamePad2 = new Gamepad();

        //setting motor parameters
        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // actuatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // rampMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // For prgoramming robot, reverse front right
        // FRM.setDirection(DcMotorEx.Direction.REVERSE);
        //
        FLM.setDirection(DcMotorEx.Direction.REVERSE);
        // BLM.setDirection(DcMotorEx.Direction.REVERSE);
        // BRM.setDirection(DcMotorEx.Direction.REVERSE);
        // FRM.setDirection(DcMotorEx.Direction.REVERSE);

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

        //direction for headless mode
        double direction = Math.PI/2;
        double speed;


        //scaling constant default for joystick's tuned scaling curve, dont change from 1 unless you know what you're doing but really it doesnt make sense to
        double scaling_constant = 1.0;

        boolean headlessMode = true;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamePad1.copy(currentGamePad1);
            currentGamePad1.copy(gamepad1);
            previousGamePad2.copy(currentGamePad2);
            currentGamePad2.copy(gamepad2);

            /*try{
                //setting previous state of gamepad1 and current position for later use toggling slug mode
                previousGamePad1.copy(currentGamePad1);
                currentGamePad1.copy(gamepad1);
                previousGamePad2.copy(currentGamePad2);
                currentGamePad2.copy(gamepad2);
            }
                catch(RobotCoreException e){
            //}*/

            //get joystick magnitudes, we are calculating a constant to fine tune rob to movement exponentially to the magnitude of the joystick
            double right_joy_magnitude = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            double left_joy_magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));

            //apply magnitude curve for fine tuning movement
            scaling_constant = Math.pow(left_joy_magnitude, driver_scaling_constant);

            if (scaling_constant == 0.0)
            {
                scaling_constant = 1.0;
            }

            if(currentGamePad1.right_bumper && !previousGamePad1.right_bumper){
                headlessMode = !headlessMode;
            }

            if (headlessMode) {
                telemetry.addData("Right trigger", currentGamePad2.right_trigger);
                telemetry.addData("Left trigger", currentGamePad2.left_trigger);

                // start button will reset the robot heading
                if (currentGamePad1.start && !previousGamePad1.start) {
                    parameters = new BNO055IMU.Parameters();
                    parameters.mode = BNO055IMU.SensorMode.IMU;
                    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.loggingEnabled = false;
                    imu.initialize(parameters);
                }

                if (currentGamePad2.right_trigger > 0) {
                    slide.setPower(1);
                }
                else if (currentGamePad2.left_trigger > 0) {
                    slide.setPower(-1);
                }
                else {
                    slide.setPower(0);
                }

                if (currentGamePad2.dpad_down) {
                    wheelMotor.setPower(0.8 * wheelMotorPower);
                }
                else if (currentGamePad2.dpad_up) {
                    wheelMotor.setPower(-0.8 * wheelMotorPower);
                }
                else {
                    wheelMotor.setPower(0);
                }

                if (currentGamePad2.right_bumper && !currentGamePad2.left_bumper) {
                    boxRotator.setPower(0.5);
                }
                else if (currentGamePad2.left_bumper && !currentGamePad2.right_bumper ) {
                    boxRotator.setPower(-0.5);
                }
                else {
                    boxRotator.setPower(0);
                }

                if (currentGamePad2.a) {
                    boxOpener.setPosition(0);
                }
                else if (currentGamePad2.y) {
                    boxOpener.setPosition(0.5);
                }
                else if (currentGamePad2.x) {
                    boxOpener.setPosition(1);
                }
                else if (currentGamePad2.b) {
                    boxOpener.setPosition(-0.5);
                }


                /*if (currentGamePad1.dpad_down) {
                    actuatorMotor.setPower(0.5);
                }
                else if (currentGamePad1.dpad_up) {
                    actuatorMotor.setPower(-0.5);
                }
                else {
                    actuatorMotor.setPower(0);
                }*/

                /*if (currentGamePad2.right_trigger > 0) {
                    rampMotor.setPower(1);
                }
                else if (currentGamePad2.left_trigger > 0) {
                    rampMotor.setPower(-1);
                }
                else {
                    rampMotor.setPower(0);
                }*/

                /*if (currentGamePad1.dpad_up) {
                    airplaneLauncher.setPower(0.5);
                }
                else if (currentGamePad1.dpad_down) {
                    airplaneLauncher.setPower(-0.5);
                }
                else {
                    airplaneLauncher.setPower(0);
                }*/

                if (currentGamePad1.dpad_left) {
                    airplaneLauncher.setPosition(0.5);
                }
                else if (currentGamePad1.dpad_right) {
                    airplaneLauncher.setPosition(0.85);
                }



                // button a to toggle slug mode

                if (currentGamePad1.left_bumper && !previousGamePad1.left_bumper) {
                    slugMode = !slugMode;
                }

                // setting direction, atan2 gives back coordinates in radians, on the range of -pi to pi, (adj, opp)
                direction = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (getAngle() * (Math.PI / 180));

                //a high value of speed should ensure the robot moves at maximum speed at all times because any numbers that
                //are too high will be fixed once divided by the denominator later.
                speed = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));

                //rotation is added to the left side motors of the robot to allow for curved driving
                FLPower = (speed * (Math.sin(direction + Math.PI / 4.0)) - directionMultiplier * gamepad1.right_stick_x); // reverse + and -
                FRPower = (speed * (Math.sin(direction - Math.PI / 4.0)) + directionMultiplier * gamepad1.right_stick_x);
                BLPower = (speed * (Math.sin(direction - Math.PI / 4.0)) - directionMultiplier * gamepad1.right_stick_x);
                BRPower = (speed * (Math.sin(direction + Math.PI / 4.0)) + directionMultiplier * gamepad1.right_stick_x);

                if (slugMode) {
                    FRPower = FRPower * slugMultiplier;
                    BRPower = BRPower * slugMultiplier;
                    FLPower = FLPower * slugMultiplier;
                    BLPower = BLPower * slugMultiplier;
                }

                while(gamepad1.a){
                    BRM.setPower(0.1);
                }
                while(gamepad1.b){
                    FRM.setPower(0.1);
                }
                while(gamepad1.y){
                    FLM.setPower(0.1);
                }
                while(gamepad1.x){
                    BLM.setPower(0.1);
                }

                FRPower = FRPower * scaleMultiplier;
                BRPower = BRPower * scaleMultiplier;
                FLPower = FLPower * scaleMultiplier;
                BLPower = BLPower * scaleMultiplier;

                FRPower = FRPower * scaling_constant;
                BRPower = BRPower * scaling_constant;
                FLPower = FLPower * scaling_constant;
                BLPower = BLPower * scaling_constant;

                FRM.setPower(FRPower);
                BRM.setPower(BRPower);
                FLM.setPower(FLPower);
                BLM.setPower(BLPower);
            }
            else{
                // button a to toggle slug mode
                if (currentGamePad1.left_bumper && !previousGamePad1.left_bumper) {
                    slugMode = !slugMode;
                }

                /*if (currentGamePad1.right_trigger > 0) {
                    slide.setPower(currentGamePad1.right_trigger);
                }
                else if (currentGamePad1.left_trigger > 0) {
                    slide.setPower(-currentGamePad1.left_trigger);
                }
                else {
                    slide.setPower(0);
                }*/

                /*if (currentGamePad2.dpad_down) {
                    wheelMotor.setPower(wheelMotorPower);
                }
                else if (currentGamePad2.dpad_up) {
                    wheelMotor.setPower(-wheelMotorPower);
                }
                else {
                    wheelMotor.setPower(0);
                }*/

                /*if (currentGamePad2.right_trigger > 0) {
                    rampMotor.setPower(0.5);
                }
                else if (currentGamePad2.right_trigger < 0) {
                    rampMotor.setPower(-0.5);
                }
                else {
                    rampMotor.setPower(0);
                }*/

                /*if (currentGamePad1.right_bumper) {
                    boxRotator.setPower(0.5);
                }
                else if (currentGamePad1.left_bumper) {
                    boxRotator.setPower(-0.5);
                }
                else {
                    boxRotator.setPower(0);
                }*/

                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                //rotation (rot) will be added to the left side motors of the robot to allow for curved driving
                double rot = directionMultiplier*gamepad1.right_stick_x;

                FRPower = y - x - rot;
                BRPower = y + x - rot;
                FLPower = y + x + rot;
                BLPower = y - x + rot;

                if (slugMode) {
                    FRPower = FRPower * slugMultiplier;
                    BRPower = BRPower * slugMultiplier;
                    FLPower = FLPower * slugMultiplier;
                    BLPower = BLPower * slugMultiplier;
                }

                FRPower = FRPower * scaleMultiplier;
                BRPower = BRPower * scaleMultiplier;
                FLPower = FLPower * scaleMultiplier;
                BLPower = BLPower * scaleMultiplier;

                FRPower = FRPower * scaling_constant;
                BRPower = BRPower * scaling_constant;
                FLPower = FLPower * scaling_constant;
                BLPower = BLPower * scaling_constant;

                FRM.setPower(FRPower);
                BRM.setPower(BRPower);
                FLM.setPower(FLPower);
                BLM.setPower(BLPower);
            }


            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.addData("Heading", getAngle());
            telemetry.addData("scaling_constant", scaling_constant);
            telemetry.addData("Direction of Driving Based on Robot", direction*180/Math.PI);
            telemetry.addData("Desired direction", Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            telemetry.update();

        }
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

        // return globalAngle;
        return globalAngle;
    }
}