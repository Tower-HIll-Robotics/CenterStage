/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode.autonomous;


import android.util.Size;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
@Autonomous(name = "Drive To AprilTag", group = "Autonomous")
// @Disabled
public class driveToAprilTagMecanum extends LinearOpMode {


    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;


    //The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;


    //Our Array used to hold x, y, and yaw april tag values
    static public double[] values = new double[3];
    static public int id = 2;


    //BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    @Override
    public void runOpMode() {
        //Map motors
        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");


        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // For prgoramming robot, reverse front right
        // FRM.setDirection(DcMotorEx.Direction.REVERSE);
        FLM.setDirection(DcMotorEx.Direction.REVERSE);


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


        runToAprilTag();
    }


    private void runToAprilTag() {
        initAprilTag();
        updateValues();
        int marginOfError = 1;
        double xPower = 0.4;
        double yPower = 0.4;


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        if (opModeIsActive()) {
            updateValues();
            // values[0] is the left and right alignment with april tag (x)
            // values[1] is the distance to april tag (y)
            // values[2] is the yaw of the robot to the april tag
            double desiredX = 0;
            double desiredY = 20;
            double distanceX;
            double distanceY;
            values[1]= 30;




            while (values[0] >= desiredX + marginOfError || values[0] <= desiredX - marginOfError || values[1] >= desiredY + marginOfError || values[1] <= desiredY - marginOfError && opModeIsActive()) {
                updateValues();
                distanceY = Math.abs(desiredY - values[1]);
                if (values[0] > desiredX){
                    FLM.setPower(xPower);
                    BLM.setPower(-xPower);
                    FRM.setPower(-xPower);
                    BRM.setPower(xPower);
                }
                else if (values[0] < desiredX){
                    FLM.setPower(-xPower);
                    BLM.setPower(xPower);
                    FRM.setPower(xPower);
                    BRM.setPower(-xPower);
                }
                else{
                    FLM.setPower(0);
                    BLM.setPower(0);
                    FRM.setPower(0);
                    BRM.setPower(0);
                }


                if (values[1] > desiredY){
                    if (distanceY < 5){
                        FLM.setPower(0.5 * yPower);
                        BLM.setPower(0.5 * yPower);
                        FRM.setPower(0.5 * yPower);
                        BRM.setPower(0.5 * yPower);
                    }
                    FLM.setPower(yPower);
                    BLM.setPower(yPower);
                    FRM.setPower(yPower);
                    BRM.setPower(yPower);
                }
                else if (values[1] < desiredY){
                    if (distanceY < 5){
                        FLM.setPower(0.5 * -yPower);
                        BLM.setPower(0.5 * -yPower);
                        FRM.setPower(0.5 * -yPower);
                        BRM.setPower(0.5 * -yPower);
                    }
                    FLM.setPower(-yPower);
                    BLM.setPower(-yPower);
                    FRM.setPower(-yPower);
                    BRM.setPower(-yPower);
                }
                else{
                    FLM.setPower(0);
                    BLM.setPower(0);
                    FRM.setPower(0);
                    BRM.setPower(0);
                }




                telemetry.addData("x", values[0]);
                telemetry.addData("y", values[1]);
                telemetry.addData("yaw", values[2]);
                telemetry.update();


            }
        }
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)


                // ... these parameters are fx, fy, cx, cy.


                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));


        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);


        // Set and enable the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */


    private void updateValues() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double x = 0;
        double y = 30;
        double yaw = 0;


        // Set the x, y, and yaw for the specified ID
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                //might be able to optimize this by not going through every detection
                if (detection.id == id) {
                    x = detection.ftcPose.x;
                    y = detection.ftcPose.y;
                    yaw = detection.ftcPose.yaw;
                    break;
                }
            }
        }   // end for() loop


        values[0] = x;
        values[1] = y;
        values[2] = yaw;
    }   // end method updateValues()


    /*determines the robot's heading based on it's initial start position:
       straight ahead is 0 degrees, left is positive, right is negative
    */
    private double getAngle() {
       /* We have to process the angle because the imu works in euler angles so the Z axis is
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


    public void turn(double degrees) {
        double currentAngle = getAngle();
        double targetAngle = currentAngle + degrees;
        double errorMargin = 1;
        while (currentAngle < (targetAngle - errorMargin) || currentAngle > (targetAngle + errorMargin)) {
            double motorPower = 0.2;
            //TO DO: check turn direction
            if (currentAngle < targetAngle - errorMargin) {
                FLM.setPower(motorPower);
                BLM.setPower(motorPower);
                FRM.setPower(-motorPower);
                BRM.setPower(-motorPower);
            }
            else if (currentAngle>targetAngle+errorMargin) {
                FLM.setPower(-motorPower);
                BLM.setPower(-motorPower);
                FRM.setPower(motorPower);
                BRM.setPower(motorPower);
            }
            telemetry.addData("TARGET ANGLE", targetAngle);
            telemetry.addData("CURRENT ANGLE", getAngle());
            telemetry.update();

            //update currentAngle
            currentAngle = getAngle();
        }
    }
}   // end class