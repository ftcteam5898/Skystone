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

@Autonomous(name = "SkystoneBlue", group = "Autonomous")
public class SkystoneBlue extends LinearOpMode {
	private DcMotor DT3, DT2, DT4, DT1, LF1, LF2, IN1, IN2;
	//DT2---O----O----DT1
	// |   IN1  IN2    |
	// |               |
	// |               |
	// |  LF1    LF2   |
	//DT3-------------DT4

	private CRServo SS1, SS2;
	private ColorSensor CS1, CS2; //CS1 = V3.0, CS2 = V2.0
	// -----O----O------
	// |               |
	// |               |
	//CS1             CS2
	//SS1             SS2
	// -----------------


	private BNO055IMU imu2;
	private Orientation angles1, angles2;
	private ArrayList<String> blackbox;

	@Override
	public void runOpMode() {
		DT3 = hardwareMap.get(DcMotor.class, "lb");
		DT2 = hardwareMap.get(DcMotor.class, "lf");
		DT4 = hardwareMap.get(DcMotor.class, "rb");
		DT1 = hardwareMap.get(DcMotor.class, "rf");
		LF1 = hardwareMap.get(DcMotor.class, "liftl");
		LF2 = hardwareMap.get(DcMotor.class, "liftr");
		IN1 = hardwareMap.get(DcMotor.class, "intakel");
		IN2 = hardwareMap.get(DcMotor.class, "intaker");
		SS1 = hardwareMap.get(CRServo.class, "foundation2");
		CS1 = hardwareMap.get(ColorSensor.class, "cs1");
		CS2 = hardwareMap.get(ColorSensor.class, "cs2");

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
		int[] pos = {0, 0}; //y = 5400
		waitForStart();
		long to = -1;

	//Go To Stones
		left(2060, 0.7);
		pos[0] = -2060
		correct(headingO);
	//Scan
		for (int i = 0; i < 3; i++){
			if ((CS2.red() * CS2.green()) / (Math.pow(CS2.blue(), 2)) <= 2 || i == 2) {
				forwardLeft(212);
				pos[0] -= 150;
				pos[1] += 150;
				left(88);
				pos[0] -= 88;
				SS1.setPower(1);
				//to = System.currentTimeMillis();
				//while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
				forward(100);
				pos[1] += 100;
				correct(headingO);
				break;
			}
			else {
				forward(-500);
				pos[1] -= 500;
				correct(headingO);
			}
		}
	//First Skystone Run (2298, First Stone Y)
		firstStone = pos[1];
		forwardRight(1626);
		pos[0] += 1150;
		pos[1] += 1150;
		correct(headingO);
		forward(1150 - pos[1]);
		pos[1] = 2300;
		SS1.setPower(-1);
		//to = System.currentTimeMillis();
		//while (opModeIsActive() && System.currentTimeMillis() - to < 1200) {}
		SS1.setPower(0);
		correct(headingO);
	//Return for Second Skystone (1148, 2300)
		forward(-2300 + (firstStone - 850));
		pos[1] = firstStone - 850;
		correct(headingO);
		forwardRight(-1626);
		pos[0] -= 1150;
		pos[1] -= 1150;
		SS1.setPower(1);
		//to = System.currentTimeMillis();
		//while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
		correct(headingO);
	//Second Skystone Run (2298, First Stone y - 2000)
		forwardRight(1626);
		pos[0] += 1150
		pos[1] += 1150
		correct(headingO);
		forward(850 - firstStone + 2300);
		pos[1] = 2300
		SS1.setPower(-1);
		//to = System.currentTimeMillis();
		//while (opModeIsActive() && System.currentTimeMillis() - to < 1250) {}
		SS1.setPower(0);
		correct(headingO);
	//Park
		forward(-1000);
		pos[1] -= 1000;
	}

//HEADING
	public void correct(double ho) {
		double diff = ho - getHeading();
		spin((int) (-diff * 19));
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
		DT1.setPower(0);
		DT4.setPower(0);
		DT2.setPower(0);
		DT3.setPower(0);
	}

	public void setRunMode(DcMotor.RunMode runMode) {
		DT3.setMode(runMode);
		DT2.setMode(runMode);
		DT4.setMode(runMode);
		DT1.setMode(runMode);
	}

//MOVEMENT
	public void forward(int amt) {
		DT1.setTargetPosition(DT1.getCurrentPosition() + amt);
		DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
		DT2.setTargetPosition(DT2.getCurrentPosition() - amt);
		DT3.setTargetPosition(DT3.getCurrentPosition() - amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT1.setPower(0.85);
		DT4.setPower(0.85);
		DT2.setPower(0.85);
		DT3.setPower(0.85);
		while ((DT1.isBusy() && DT4.isBusy() && DT2.isBusy() && DT#.isBusy()) && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void left(int amt) {
		DT1.setTargetPosition(DT1.getCurrentPosition() - amt);
		DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
		DT2.setTargetPosition(DT2.getCurrentPosition() - amt);
		DT3.setTargetPosition(DT3.getCurrentPosition() + amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT1.setPower(0.85);
		DT4.setPower(0.85);
		DT2.setPower(0.85);
		DT3.setPower(0.85);
		while ((DT1.isBusy() && DT4.isBusy() && DT2.isBusy() && DT3.isBusy()) && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void left(int amt, double pow) {
		DT1.setTargetPosition(DT1.getCurrentPosition() - amt);
		DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
		DT2.setTargetPosition(DT2.getCurrentPosition() - amt);
		DT3.setTargetPosition(DT3.getCurrentPosition() + amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT1.setPower(pow);
		DT4.setPower(pow);
		DT2.setPower(pow);
		DT3.setPower(pow);
		while ((DT1.isBusy() && DT4.isBusy() && DT2.isBusy() && DT3.isBusy()) && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void forwardLeft(int amt) {
		DT1.setTargetPosition(DT1.getCurrentPosition() + amt);
		DT3.setTargetPosition(DT3.getCurrentPosition() - amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT1.setPower(1.0);
		DT3.setPower(1.0);
		while ((DT1.isBusy() && DT3.isBusy()) && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void forwardRight(int amt) {
		DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
		DT2.setTargetPosition(DT2.getCurrentPosition() - amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT4.setPower(1.0);
		DT2.setPower(1.0);
		while ((DT4.isBusy() && DT2.isBusy() && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void spin(int amt) {
		DT1.setTargetPosition(DT1.getCurrentPosition() + amt);
		DT4.setTargetPosition(DT4.getCurrentPosition() + amt);
		DT2.setTargetPosition(DT2.getCurrentPosition() + amt);
		DT3.setTargetPosition(DT3.getCurrentPosition() + amt);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		DT1.setPower(0.85);
		DT4.setPower(0.85);
		DT2.setPower(0.85);
		DT3.setPower(0.85);
		while ((DT1.isBusy() && DT4.isBusy() && DT2.isBusy() && DT3.isBusy()) && opModeIsActive()) {}
		stopRobot();
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
}
