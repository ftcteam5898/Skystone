package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OrbitalTeleOp", group = "TeleOp")
public class OrbitalTeleOp extends LinearOpMode {

    private DcMotor lb, lf, rb, rf, li, ri;

    @Override
    public void runOpMode() {

        setupHardware();
        waitForStart();

        //Enter main TeleOp loop
        while (opModeIsActive()) {

            //Calculate mecanum powers with turning
            double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            //Set motor powers
            lf.setPower(v1);
            rf.setPower(-v2);
            lb.setPower(v3);
            rb.setPower(-v4);

            //Intake
            if (gamepad1.a) {

                //Let's intake
                li.setPower(-0.5);
                ri.setPower(0.5);

            } else if (gamepad1.b) {

                //Outake
                li.setPower(0.5);
                ri.setPower(-0.5);

            } else {

                //Stop motion
                li.setPower(0);
                ri.setPower(0);

            }

        }

    }

    public void setupHardware() {

        //Initialize motors and other hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        li = hardwareMap.get(DcMotor.class, "li");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        ri = hardwareMap.get(DcMotor.class, "ri");

    }

}
