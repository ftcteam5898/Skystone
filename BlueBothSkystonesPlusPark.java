package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BlueBothSkystonesPlusPark", group = "Autonomous")
public class BlueBothSkystonesPlusPark extends LinearOpMode {

    private DcMotor backLeft, frontLeft, backRight, frontRight;
    private CRServo stoneArm;
    private ColorSensor colorSensor;
    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        setupHardware();

        //Take our original heading reading
        double originalHeading = getHeading();

        //A variable constant to adjust distance traveled past the bridge
        int amt = 5400;

        //A special case for a later position #1 correction
        boolean isPositionOne = false;

        waitForStart();

        //Move to the blocks and correct
        left(2060, 0.5);
        correct(originalHeading);

        if (isSkystone()) {

            //Ram the stones and adjust
            left(300);
            forward(150);

            //Drop servo arm
            stoneArm.setPower(1);
            pause(1250);

            //Slight adjustment
            forward(100);

            //Establish that a later correction is needed
            isPositionOne = true;

        } else {

            //Move to next (sky)stone
            forward(-500);
            correct(originalHeading);

            if (isSkystone()) {

                //Ram the stones and adjust
                left(300);
                forward(150);

                //Drop servo arm
                stoneArm.setPower(1);
                pause(1250);

                //Slight adjustment
                forward(200);

                //Change "amt" constant so it's the same amount for all three positions
                amt += 500;

            } else {

                //Move to the last stone (which we now know is the skystone)
                forward(-500);


                //Ram the stones and adjust
                left(300);
                forward(150);
                correct(originalHeading);


                //Drop servo arm
                stoneArm.setPower(1);
                pause(1250);

                //Slight adjustment
                forward(200);

                //Change "amt" constant so it's the same amount for all three positions
                amt += 500;

            }
        }

        //Move away from stones
        left(-900);
        correct(originalHeading);

        //Make small correction for only position #1
        if (isPositionOne){

            //Just make a slight movement forward
            forward(100);

        }

        //Drive completely under the bridge
        forward(amt - 2300);
        correct(originalHeading);

        //Release skystone
        stoneArm.setPower(-1);
        pause(1200);
        stoneArm.setPower(0);
        correct(originalHeading);

        //Move back under bridge to next skystone
        forward(-amt + 300);
        correct(originalHeading);

        //Ram stones
        left(900, 0.5);
        correct(originalHeading);

        //Grab second skystone
        stoneArm.setPower(1);
        pause(1250);

        //Slight adjustment and slide away from stones
        forward(200);
        left(-1150);
        correct(originalHeading);

        //Go back under the bridge
        forward(amt);

        //Release second skystone
        stoneArm.setPower(-1);
        pause(1250);
        stoneArm.setPower(0);

        //Park under bridge
        correct(originalHeading);
        forward(-2100);
    }

    public void pause(long time) {

        //Stalls thread for specified time in milliseconds
        long to = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - to < time) {}

    }

    public boolean isSkystone() {

        //Check if the normalized value is in Skystone range
        return (colorSensor.red() * colorSensor.green()) / Math.pow(colorSensor.blue(), 2) <= 2;

    }

    public void setupHardware() {

        //Initialize hardware
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        stoneArm = hardwareMap.get(CRServo.class, "foundation2");
        colorSensor = hardwareMap.get(ColorSensor.class, "cs2");

        //Set up IMU
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU2";
        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters2);

    }

    public void correct(double ho) {

        //Calculate difference between current and original heading, then correct based on a pre-measured constant
        double diff = ho - getHeading();
        spin((int) (-diff * 19));

    }

    public double getHeading() {

        //Returns a heading value in [-180, 180]
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(formatAngle(angles.angleUnit, angles.firstAngle));

    }

    public double formatAngle(AngleUnit angleUnit, double angle) {

        //Reformat angle for getHeading method
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));

    }

    public double formatDegrees(double degrees){

        //Return normalized degrees value
        return AngleUnit.DEGREES.normalize(degrees);
    }

    public void stopRobot() {

        //Stop all motors
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

    }

    public void setRunMode(DcMotor.RunMode runMode) {

        //Change all motor run modes to runMode variable
        backLeft.setMode(runMode);
        frontLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontRight.setMode(runMode);

    }

    public void forward(int amt) {

        //Reset all target positions... +/- varies by side
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + amt);
        backRight.setTargetPosition(backRight.getCurrentPosition() + amt);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - amt);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - amt);

        //Set run mode
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Turn on all the motors
        frontRight.setPower(0.85);
        backRight.setPower(0.85);
        frontLeft.setPower(0.85);
        backLeft.setPower(0.85);

        //Stall until we're done moving or the OpMode is no longer active
        while ((frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) && opModeIsActive()) {}

        //Turn off!
        stopRobot();

        //Change run mode
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void left(int amt) {

        //Run at default 0.85 power
        left(amt, 0.85);

    }

    public void left(int amt, double pow) {

        //Reset all target positions... +/- varies by side
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - amt);
        backRight.setTargetPosition(backRight.getCurrentPosition() + amt);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - amt);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + amt);

        //Set run mode
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Turn on all the motors
        frontRight.setPower(pow);
        backRight.setPower(pow);
        frontLeft.setPower(pow);
        backLeft.setPower(pow);

        //Stall until we're done moving or the OpMode is no longer active
        while ((frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) && opModeIsActive()) {}

        //Turn off!
        stopRobot();

        //Change run mode
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(int amt) {

        //Reset all target positions... +/- varies by side
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + amt);
        backRight.setTargetPosition(backRight.getCurrentPosition() + amt);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + amt);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + amt);

        //Set run mode
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Turn on all the motors
        frontRight.setPower(0.85);
        backRight.setPower(0.85);
        frontLeft.setPower(0.85);
        backLeft.setPower(0.85);

        //Stall until we're done moving or the OpMode is no longer active
        while ((frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) && opModeIsActive()) {}

        //Turn off!
        stopRobot();

        //Change run mode
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
