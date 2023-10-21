package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="MainDrive", group="Teleop")
// @Disabled
public class mainDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;

    double FRPower, BRPower, FLPower, BLPower;

    //set constants
    double directionMultiplier = 0.5;
    double intakePower = 1;
    double outtakePower = 1;

    // default value
    double slide_encoder_value = 0;

    // Setting up Slug Mode Parameters
    boolean slugMode = false;
    double slugMultiplier = 0.33;

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

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


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
        double direction = 0;
        double speed;

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

            if(currentGamePad1.right_bumper && !previousGamePad1.right_bumper){
                headlessMode = !headlessMode;
            }

            if(headlessMode) {
                telemetry.addData("Right trigger", currentGamePad2.right_trigger);
                telemetry.addData("Left trigger", currentGamePad2.left_trigger);

                //start button will reset the robot heading
                if (currentGamePad1.start && !previousGamePad1.start) {
                    parameters = new BNO055IMU.Parameters();
                    parameters.mode = BNO055IMU.SensorMode.IMU;
                    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.loggingEnabled = false;
                    imu.initialize(parameters);
                }

                // button a to toggle slug mode
                if (currentGamePad1.left_bumper && !previousGamePad1.left_bumper) {
                    slugMode = !slugMode;
                }

                // setting direction, atan2 gives back coordinates in radians, on the range of -pi to pi, (adj, opp)
                direction = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (getAngle() * (Math.PI / 180));

                //a high value of speed should ensure the robot moves at maximum speed at all times because any numbers that
                //are too high will be fixed once divided by the denominator later.
                speed = 10;

                //rotation is added to the left side motors of the robot to allow for curved driving
                FLPower = speed * (Math.sin(direction + Math.PI / 4.0) + directionMultiplier * gamepad1.right_stick_x);
                FRPower = speed * (Math.sin(direction - Math.PI / 4.0) - directionMultiplier * gamepad1.right_stick_x);
                BLPower = speed * (Math.sin(direction - Math.PI / 4.0) + directionMultiplier * gamepad1.right_stick_x);
                BRPower = speed * (Math.sin(direction + Math.PI / 4.0) - directionMultiplier * gamepad1.right_stick_x);

                if (slugMode) {
                    FRPower = FRPower * slugMultiplier;
                    BRPower = BRPower * slugMultiplier;
                    FLPower = FLPower * slugMultiplier;
                    BLPower = BLPower * slugMultiplier;
                }

                //Denominator is the maximum power given to a motor
                double denominator = Math.max(Math.abs(FRPower), Math.max(Math.abs(BRPower), Math.max(Math.abs(FLPower), Math.abs(BLPower))));
                //dividing by denominator will ensure the highest power given to a motor is 1, so none of the proportions are thrown off
                FLPower /= denominator;
                FRPower /= denominator;
                BLPower /= denominator;
                BRPower /= denominator;

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

                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
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

                //Denominator is the maximum power given to a motor
                double denominator = Math.max(Math.abs(FRPower), Math.max(Math.abs(BRPower), Math.max(Math.abs(FLPower), Math.abs(BLPower))));
                //dividing by denominator will ensure the highest power given to a motor is 1, so none of the proportions are thrown off
                FLPower /= denominator;
                FRPower /= denominator;
                BLPower /= denominator;
                BRPower /= denominator;

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