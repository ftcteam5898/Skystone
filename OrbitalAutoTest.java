package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "OrbitalAutoTest", group = "Autonomous")
public class OrbitalAutoTest extends LinearOpMode {

    private DcMotor lb, lf, rb, rf;

    @Override
    public void runOpMode() {

        setupHardware();
        waitForStart();

        //All right, let's see what we can do!
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
        forward(1000, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);

        while (opModeIsActive()) {}
    }

    public void setupHardware() {

        //Initialize hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");

    }

    public void stopRobot() {

        //Stop all motors
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

    }

    public void setRunMode(DcMotor.RunMode runMode) {

        //Change all motor run modes to runMode variable
        lb.setMode(runMode);
        lf.setMode(runMode);
        rb.setMode(runMode);
        rf.setMode(runMode);

    }

    public void setStopMode(DcMotor.ZeroPowerBehavior zpb) {

        //Change all motors to a ZeroPowerBehavior
        lb.setZeroPowerBehavior(zpb);
        lf.setZeroPowerBehavior(zpb);
        rb.setZeroPowerBehavior(zpb);
        rf.setZeroPowerBehavior(zpb);

    }

    public void forward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {

        //Reset all target positions... +/- varies by side
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);

        //Set run mode
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Stall until we're done moving or the OpMode is no longer active
        while ((rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) && opModeIsActive()) {

            //Turn on all the motors
            double prop = (amt - (rf.getTargetPosition() - rf.getCurrentPosition())) / (double) amt;
            telemetry.addData("P", prop);
            telemetry.update();
            double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
            rf.setPower(adjPower);
            rb.setPower(adjPower);
            lf.setPower(adjPower);
            lb.setPower(adjPower);

        }

        //Turn off!
        stopRobot();

        //Change run mode
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

}
