package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.OPTIONS;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import android.content.Context;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp(name = "Test - Prop Detector", group = "Test")
public final class PropDetectorTest extends CommandOpMode {
    private PropDetector propDetector;
    private OpenCvCamera camera;
    private GamepadEx gamepad;

    @Override public void initialize() {
       propDetector = new PropDetector(PropColor.RED);

       gamepad = new GamepadEx(gamepad1);

       initializeCamera();

       configureBindings();

       schedule(
              new RunCommand(this::displayControls),
              new InstantCommand(telemetry::update)
       );
    }

    private void initializeCamera() {
        Context appContext = hardwareMap.appContext;

        int cameraMonitorViewId = appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
                );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, UPRIGHT);
                camera.setPipeline(propDetector);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed To Open Camera With Error Code", errorCode);
                telemetry.update();
            }
        });
    }

    private void configureBindings() {
        new GamepadButton(gamepad, OPTIONS)
                .whenPressed(propDetector::swapColor);
    }

    private void displayControls() {
        telemetry.addLine("To Swap Prop Detection Color Press Options On Gamepad 1");
    }
}