package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

//Changed || to && in busy loop, should improve or we'll need to write our own methods

@Autonomous(name = "RedBothSkystonesPlusPark", group = "Autonomous")
public class RedBothSkystonesPlusPark extends LinearOpMode {

    private DcMotor lb, lf, rb, rf, liftl, liftr, intakel, intaker;

    private CRServo foundation1, foundation2;

    private ColorSensor cs1, cs2;

    private BNO055IMU imu2;
    private Orientation angles1, angles2;

    private ArrayList<String> blackbox;

    @Override
    public void runOpMode() {

        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        liftl = hardwareMap.get(DcMotor.class, "liftl");
        liftr = hardwareMap.get(DcMotor.class, "liftr");
        intakel = hardwareMap.get(DcMotor.class, "intakel");
        intaker = hardwareMap.get(DcMotor.class, "intaker");
        foundation1 = hardwareMap.get(CRServo.class, "foundation1");
        foundation2 = hardwareMap.get(CRServo.class, "foundation2");
        cs1 = hardwareMap.get(ColorSensor.class, "cs1");
        cs2 = hardwareMap.get(ColorSensor.class, "cs2");

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU2";
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        imu2.initialize(parameters2);

        blackbox = new ArrayList<>();

        double headingO = getHeading();

        int amt = 5400;

        waitForStart();

        //align(42.5);

        long to = -1;
        boolean dds = false;

        left(2060, 0.5);
        correct(headingO);
        if ((cs1.red() * cs1.green()) / (Math.pow(cs1.blue(), 2)) <= 3) {
            left(300);
            forward(150);
            foundation1.setPower(-1);
            to = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
            forward(100);
            dds = true;
        } else {
            forward(-500);
            correct(headingO);
            if ((cs1.red() * cs1.green()) / (Math.pow(cs1.blue(), 2)) <= 3) {
                left(300);
                forward(150);
                foundation1.setPower(-1);
                to = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
                forward(200);
                amt += 500;
            } else {
                forward(-500);
                left(300);
                forward(150);
                correct(headingO);
                foundation1.setPower(-1);
                to = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
                forward(200);
                amt += 500;
            }
        }
        left(-900);
        correct(headingO);
        if (dds) forward(100);
        forward(amt - 2300); //
        correct(headingO);
        foundation1.setPower(1);
        to = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - to < 1200) {}
        foundation1.setPower(0);
        correct(headingO);
        forward(-amt + 500); //
        correct(headingO);
        left(1000, 0.5);
        correct(headingO);
        foundation1.setPower(-1);
        to = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
        forward(200);
        left(-1050);
        correct(headingO);
        forward(100);
        correct(headingO);
        forward(amt - 500); //
        foundation1.setPower(1);
        to = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
        foundation1.setPower(0);
        correct(headingO);
        forward(-1800); //

    }

    public void correct(double ho) {

        double diff = ho - getHeading();
        spin((int) (-diff * 19));
        /*while (Math.abs(getHeading() - ho) > 0.25) {
            rb.setPower((getHeading() - ho) / 40);
            rf.setPower((getHeading() - ho) / 40);
            lb.setPower((getHeading() - ho) / 40);
            lf.setPower((getHeading() - ho) / 40);
        }*/

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

    public void stopRobot() {

        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

    }

    public void setRunMode(DcMotor.RunMode runMode) {

        lb.setMode(runMode);
        lf.setMode(runMode);
        rb.setMode(runMode);
        rf.setMode(runMode);

    }

    public void forward(int amt) {

        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        double pow = 0.85;
        long to = System.currentTimeMillis();
        while ((rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) && opModeIsActive()) {
            double mult = (System.currentTimeMillis() - to) / 250;
            if (mult > 1) mult = 1;
            rb.setPower(mult * pow);
            rf.setPower(mult * pow);
            lb.setPower(mult * pow);
            lf.setPower(mult * pow);
        }
        stopRobot();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void left(int amt) {

        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        double pow = 0.85;
        long to = System.currentTimeMillis();
        while ((rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) && opModeIsActive()) {
            double mult = (System.currentTimeMillis() - to) / 250;
            if (mult > 1) mult = 1;
            rb.setPower(mult * pow);
            rf.setPower(mult * pow);
            lb.setPower(mult * pow);
            lf.setPower(mult * pow);
        }
        stopRobot();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void left(int amt, double pow) {

        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        long to = System.currentTimeMillis();
        while ((rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) && opModeIsActive()) {
            double mult = (System.currentTimeMillis() - to) / 250;
            if (mult > 1) mult = 1;
            rb.setPower(mult * pow);
            rf.setPower(mult * pow);
            lb.setPower(mult * pow);
            lf.setPower(mult * pow);
        }
        stopRobot();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(int amt) {

        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setPower(0.85);
        rb.setPower(0.85);
        lf.setPower(0.85);
        lb.setPower(0.85);
        while ((rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) && opModeIsActive()) {}
        stopRobot();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
