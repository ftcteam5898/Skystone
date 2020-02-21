package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "SPINBOI", group = "Autonomous")

public class SPINBOI extends LinearOpMode {

    //Motors (Based on Quadrant)
    private DcMotor DT3, DT2, DT4, DT1, LF, IN1, IN2;

    //Servos (Left - 1, Right - 2)
    private CRServo MD1, MD2, OT1, OT2;

    private Servo ctl, cbl, ctr, cbr;

    private BNO055IMU imu2;
    private Orientation angles2;

    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector = new SkystoneDetector();
    private String position;

    @Override
    public void runOpMode() {

        setupHardware();
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
        double headingO = getHeading();
        telemetry.addData("Heading:", getHeading());
        telemetry.update();
        spin(67, 0.2, 0.5, 0.7, 0.3, 0.333, 0.333);
    }

    public void setupHardware() {

        //Initialize hardware
        DT3 = hardwareMap.get(DcMotor.class, "lb");
        DT2 = hardwareMap.get(DcMotor.class, "lf");
        DT4 = hardwareMap.get(DcMotor.class, "rb");
        DT1 = hardwareMap.get(DcMotor.class, "rf");
        LF = hardwareMap.get(DcMotor.class, "lift");
        IN1 = hardwareMap.get(DcMotor.class, "li");
        IN2 = hardwareMap.get(DcMotor.class, "ri");
        MD1 = hardwareMap.get(CRServo.class, "lm");
        MD2 = hardwareMap.get(CRServo.class, "rm");
        OT1 = hardwareMap.get(CRServo.class, "lo");
        OT2 = hardwareMap.get(CRServo.class, "ro");
        ctl = hardwareMap.get(Servo.class, "ctl");
        ctr = hardwareMap.get(Servo.class, "ctr");
        cbl = hardwareMap.get(Servo.class, "cbl");
        cbr = hardwareMap.get(Servo.class, "cbr");

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU2";
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        imu2.initialize(parameters2);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopRobot() {

        //Stop all motors
        DT1.setPower(0);
        DT4.setPower(0);
        DT2.setPower(0);
        DT3.setPower(0);

    }

    public void setRunMode(DcMotor.RunMode runMode) {

        //Change all motor run modes to runMode variable
        DT3.setMode(runMode);
        DT2.setMode(runMode);
        DT4.setMode(runMode);
        DT1.setMode(runMode);

    }

    public void setStopMode(DcMotor.ZeroPowerBehavior zpb) {

        //Change all motors to a ZeroPowerBehavior
        DT3.setZeroPowerBehavior(zpb);
        DT2.setZeroPowerBehavior(zpb);
        DT4.setZeroPowerBehavior(zpb);
        DT1.setZeroPowerBehavior(zpb);

    }

    public double getPower(double amtDone, double edge1, double edge2, double max, double min, double p1, double p2) {

        //Determine power based on an adjustable curve metric inspired by the Normal Distribution
        if (amtDone >= edge1 && amtDone <= 1 - edge2) {

            //We've accelerated and are in the middle of our motion, so we're at max power.
            return max;

        } else {

            if (amtDone > 1 - edge2) {

                //Last edge... what's our power?
                amtDone = 1 - amtDone;
                double amtNormDone = amtDone / edge2;
                return min + evaluateNormal(1, p2, amtNormDone, max - min);

            } else {

                //How much of the way through are we, and what power should we be on?
                double amtNormDone = amtDone / edge1;
                return min + evaluateNormal(1, p1, amtNormDone, max - min);

            }

        }

    }

    public double evaluateNormal(double mu, double sigma, double x, double max) {

        //Return an adjusted Normal Distribution value such that the maximum possible value is "max"
        double exponent = -Math.pow(x - mu, 2) / (2 * Math.pow(sigma, 2));
        return max * Math.pow(Math.E, exponent);

    }

    //NEED TO FIX LATER
    public void correct(double ho) {
        double diff = ho - getHeading();
        //spin((int) (-diff * 19));
    }

    public double getHeading() {
        angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double v2 = formatAngle(angles2.angleUnit, angles2.firstAngle);
        return (AngleUnit.DEGREES.normalize(v2)); //[-180, 180]
    }

    public double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double formatDegrees(double degrees){
        return AngleUnit.DEGREES.normalize(degrees);
    }

    public void spin(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
        DT1.setTargetPosition(DT1.getCurrentPosition() + amt);
        DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
        DT2.setTargetPosition(DT2.getCurrentPosition() + amt);
        DT3.setTargetPosition(DT3.getCurrentPosition() + amt);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ((DT1.isBusy() && DT4.isBusy() && DT2.isBusy() && DT3.isBusy()) && opModeIsActive()) {

            //Turn on all the motors
            double prop = (Math.abs(amt) - Math.abs(DT1.getTargetPosition() - DT1.getCurrentPosition())) / (double) amt;
            telemetry.addData("P", prop);
            telemetry.update();
            double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
            DT1.setPower(adjPower);
            DT4.setPower(adjPower);
            DT2.setPower(adjPower);
            DT3.setPower(adjPower);

        }
        stopRobot();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
