package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector = new SkystoneDetector();
    private String position;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }

        // code while robot is running
        if (position.equals("LEFT")) {
            // left code
        }
    }
}