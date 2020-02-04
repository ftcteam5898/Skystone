package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "WallFollowTest", group = "Autonomous")
public class WallFollowTest extends LinearOpMode {

    private DcMotor lb, lf, rb, rf;

    private DistanceSensor r1, r2;

    @Override
    public void runOpMode() {

        setupHardware();
        waitForStart();

        while (opModeIsActive()) {

            double bDiff = (15 - trialBack(3)) / 25;
            double fDiff = (15 - trialFront(3)) / 25;
            if (Math.abs(bDiff - fDiff) < 0.01) {
                lb.setPower(-0.25);
                lf.setPower(-0.25);
                rb.setPower(0.25);
                rf.setPower(0.25);
            } else if (fDiff > bDiff) {
                lb.setPower(-0.3);
                lf.setPower(-0.3);
                rb.setPower(0.25);
                rf.setPower(0.25);
            } else {
                lb.setPower(-0.25);
                lf.setPower(-0.25);
                rb.setPower(0.3);
                rf.setPower(0.3);
            }

        }

    }

    public double trialBack(int trials) {

        double total = 0;
        for (int i = 0; i < trials; i++) {

            total += r1.getDistance(DistanceUnit.INCH);

        }
        return total / trials;

    }

    public double trialFront(int trials) {

        double total = 0;
        for (int i = 0; i < trials; i++) {

            total += r2.getDistance(DistanceUnit.INCH);

        }
        return total / trials;

    }

    public void setupHardware() {

        //Initialize hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        r1 = hardwareMap.get(DistanceSensor.class, "r1");
        r2 = hardwareMap.get(DistanceSensor.class, "r2");

    }

    public void stopRobot() {

        //Stop all motors
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

    }

}
