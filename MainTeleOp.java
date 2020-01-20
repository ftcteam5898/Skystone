package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    private DcMotor lb, lf, rb, rf, liftl, liftr, intakel, intaker;

    private CRServo grab1, pl, pr, foundation1, foundation2, fl, fr;

    private boolean locked = false;

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
        grab1 = hardwareMap.get(CRServo.class, "grab1");
        pl = hardwareMap.get(CRServo.class, "pl");
        pr = hardwareMap.get(CRServo.class, "pr");
        foundation1 = hardwareMap.get(CRServo.class, "foundation1");
        foundation2 = hardwareMap.get(CRServo.class, "foundation2");
        fl = hardwareMap.get(CRServo.class, "fl");
        fr = hardwareMap.get(CRServo.class, "fr");

        waitForStart();

        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            lf.setPower(v1);
            rf.setPower(-v2);
            lb.setPower(v3);
            rb.setPower(-v4);
            liftl.setPower(0.4 * gamepad2.right_stick_y);
            liftr.setPower(-0.4 * gamepad2.right_stick_y);
            if (gamepad2.dpad_left) {

                locked = false;
                grab1.setPower(1);

            } else if (gamepad2.dpad_right) {

                locked = false;
                grab1.setPower(-1);

            } else if (!locked) {
                grab1.setPower(0);
            }
            if (gamepad2.dpad_up) {

                locked = true;

            } else if (gamepad2.dpad_down) {

                locked = false;

            }

            if (gamepad1.a || gamepad1.dpad_down) {

                intaker.setPower(1);
                intakel.setPower(-1);
                pl.setPower(1);
                pr.setPower(-1);

            } else if (gamepad1.b || gamepad1.dpad_right) {

                intaker.setPower(-1);
                intakel.setPower(1);
                pl.setPower(-1);
                pr.setPower(1);

            } else {

                intaker.setPower(0);
                intakel.setPower(0);
                pl.setPower(0);
                pr.setPower(0);

            }
            if (gamepad2.x) {

                foundation1.setPower(1);

            } else if (gamepad2.y) {

                foundation2.setPower(-1);

            } else {

                foundation1.setPower(0);
                foundation2.setPower(0);

            }
            if (gamepad2.b) {

                fl.setPower(1);
                fr.setPower(-1);

            } else if (gamepad2.a) {

                fl.setPower(-1);
                fr.setPower(1);

            } else {

                fl.setPower(0);
                fr.setPower(0);

            }

        }

    }

}
