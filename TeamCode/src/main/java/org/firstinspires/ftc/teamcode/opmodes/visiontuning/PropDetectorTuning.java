package org.firstinspires.ftc.teamcode.opmodes.visiontuning;

import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;
import static org.openftc.easyopencv.OpenCvCameraRotation.*;

import android.content.Context;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.vision.*;
import org.openftc.easyopencv.*;

@TeleOp(name = "Tuning - Prop Detector", group = "Tuning")
public final class PropDetectorTuning extends OpModeTemplate {
    private OpenCvCamera camera;
    private PropDetector propDetector;

    @Override public void initialize() {
        propDetector = new PropDetector(PropColor.RED);

        initializeGamepads();
        initializeCamera();

        configureBindings();

        schedule(
                new DisplayTuningInstructionsCommand(telemetry),
                new UpdateTelemetryCommand(telemetry)
        );
    }

    @Override protected void configureBindings() {
        new GamepadButton(operatorGamepad, OPTIONS)
                .whenPressed(propDetector::swapColor);

        new GamepadButton(operatorGamepad, DPAD_UP)
                .whenPressed(propDetector::incrementErodePasses);

        new GamepadButton(operatorGamepad, DPAD_DOWN)
                .whenPressed(propDetector::decrementErodePasses);
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

    private static class DisplayTuningInstructionsCommand extends CommandBase {
        private final Telemetry telemetry;

        private boolean isFinished;

        public DisplayTuningInstructionsCommand(@NonNull Telemetry telemetry) {
            this.telemetry = telemetry;
            isFinished = false;
        }

        @Override public void execute() {
            telemetry.addLine("To Swap Detection Color, Press Options On Gamepad One");
            telemetry.addLine("To Increment The Erode Passes Press Dpad Up On Gamepad One");
            telemetry.addLine("To Decrement The Erode Passes Press Dpad Down On Gamepad One");
            isFinished = true;
        }

        @Override public boolean isFinished() {
            return this.isFinished;
        }
    }

    private static class UpdateTelemetryCommand extends CommandBase {
       private final Telemetry telemetry;

       private boolean isFinished;

       public UpdateTelemetryCommand(@NonNull Telemetry telemetry) {
           this.telemetry = telemetry;
           isFinished = false;
       }

       @Override public void execute() {
           telemetry.update();

           isFinished = true;
       }

       @Override public boolean isFinished() {
           return this.isFinished;
       }
    }
}
