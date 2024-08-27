package org.firstinspires.ftc.teamcode.opmodes.visiontuning;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.AprilTagConstants.*;
import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.*;

import static java.util.concurrent.TimeUnit.*;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;
import java.util.List;

@TeleOp(name = "Tuning - April Tag", group = "Tuning")
public final class AprilTagTuning extends OpModeTemplate {
    private ArrayList<String> allAprilTagDetections;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private String aprilTagLogFileName;

    private ElapsedTime elapsedTime;

    private final int DESIRED_TAG_ID = 2;

    @Override public void initialize() {
        new ConstantsLoader().load();

        initializeGamepads();
        initAprilTagDetection();
        configureBindings();

        elapsedTime = new ElapsedTime();

        schedule(
                new RunCommand(this::detectAprilTags)
        );
    }

    @Override protected void configureBindings() {
        new GamepadButton(operatorGamepad, OPTIONS)
                .whenPressed(this::saveAprilTagData);
    }

    private void initAprilTagDetection() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(FX, FY, CX, CY)
                .build();

        aprilTagProcessor.setDecimation(DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 360))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        setCameraProperties();

        allAprilTagDetections = new ArrayList<String>();

        aprilTagLogFileName = createAprilTagLogFileName();
    }

    private void setCameraProperties() {
        while (visionPortal.getCameraState() != STREAMING) {
            telemetry.addLine("Waiting For Camera To Start");
            telemetry.update();
        }

        telemetry.addLine("Camera Ready");
        telemetry.update();

        setExposure();
        setGain();
        setWhiteBalanceTemperature();
    }

    private void setExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);

        long minExposure = exposureControl.getMinExposure(MILLISECONDS);
        long maxExposure = exposureControl.getMaxExposure(MILLISECONDS);

        if (EXPOSURE_MS < minExposure + 1) {
            exposureControl.setExposure(minExposure + 1, MILLISECONDS);
        } else if (EXPOSURE_MS > maxExposure) {
            exposureControl.setExposure(maxExposure, MILLISECONDS);
        } else {
            exposureControl.setExposure(EXPOSURE_MS, MILLISECONDS);
        }
    }

    private void setGain() {
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        int minGain = gainControl.getMinGain();
        int maxGain = gainControl.getMaxGain();

        if (GAIN > maxGain) {
            gainControl.setGain(maxGain);
        } else if (GAIN < minGain) {
            gainControl.setGain(minGain);
        } else {
            gainControl.setGain(GAIN);
        }
    }

    private void setWhiteBalanceTemperature() {
        WhiteBalanceControl whiteBalanceControl
                = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);

        int minWhiteBalanceTemperature = whiteBalanceControl.getMinWhiteBalanceTemperature();
        int maxWhiteBalanceTemperature = whiteBalanceControl.getMaxWhiteBalanceTemperature();

        if (WHITE_BALANCE_TEMPERATURE > maxWhiteBalanceTemperature) {
            whiteBalanceControl.setWhiteBalanceTemperature(maxWhiteBalanceTemperature);
        } else if (WHITE_BALANCE_TEMPERATURE < minWhiteBalanceTemperature) {
            whiteBalanceControl.setWhiteBalanceTemperature(minWhiteBalanceTemperature);
        } else {
            whiteBalanceControl.setWhiteBalanceTemperature(WHITE_BALANCE_TEMPERATURE);
        }
    }

    @NonNull private String createAprilTagLogFileName() {
        String currentDate =
                new SimpleDateFormat("yyyy_MM_dd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HH_mm_ss", Locale.CANADA).format(new Date());

        return "AprilTagLog" + currentDate + "_" + currentTime + ".txt";
    }

    private void detectAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        int numberOfTags = currentDetections.size();

        displaySaveInstructions();

        if (numberOfTags == 0) {
            telemetry.addLine("Camera Waiting ; No Tags Detected");
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;

            if (detection.id != DESIRED_TAG_ID) continue;

            double yaw     = detection.ftcPose.yaw;
            double pitch   = detection.ftcPose.pitch;
            double roll    = detection.ftcPose.roll;
            double range   = detection.ftcPose.range;
            double bearing = detection.ftcPose.bearing;
            double x       = detection.ftcPose.x;
            double y       = detection.ftcPose.y;
            double z       = detection.ftcPose.z;

            double time = elapsedTime.seconds();

            String line = time    + ","
                        + yaw     + ","
                        + pitch   + ","
                        + roll    + ","
                        + range   + ","
                        + bearing + ","
                        + x       + ","
                        + y       + ","
                        + z       + ",";

            allAprilTagDetections.add(line);

            telemetry.addData("Exposure", EXPOSURE_MS);
            telemetry.addData("Gain", GAIN);
            telemetry.addData("White Balance Temperature", WHITE_BALANCE_TEMPERATURE);

            telemetry.addData("Yaw ", "%.3f", yaw);
            telemetry.addData("Pitch ", "%.3f", pitch);
            telemetry.addData("Roll ", "%.3f", roll);
            telemetry.addData("Range ", "%.3f", range);
            telemetry.addData("Bearing ", "%.3f", bearing);
            telemetry.addData("X ", "%.3f", x);
            telemetry.addData("Y ", "%.3f", y);
            telemetry.addData("Z ", "%.3f", z);
        }

        telemetry.update();
    }

    private void saveAprilTagData() {
        telemetry.clearAll();
        telemetry.addLine("Saving April Tag Data");
        telemetry.update();

        @SuppressLint("sdCardPath")
        String sdCardPath = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";

        File aprilTagLogFile = new File(sdCardPath + aprilTagLogFileName);

        try {
            FileWriter fileWriter         = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            // ----- Write Header ----- //

            bufferedWriter.write("Exposure=" + EXPOSURE_MS);
            bufferedWriter.write("Gain=" + GAIN);
            bufferedWriter.write("WhiteBalance=" + WHITE_BALANCE_TEMPERATURE);
            bufferedWriter.write("Decimation=" + DECIMATION);

            bufferedWriter.newLine();

            bufferedWriter.write("time,yaw,pitch,roll,range,bearing,localX,y,z");

            bufferedWriter.newLine();

            for (String detection: allAprilTagDetections) {
                bufferedWriter.write(detection);
                bufferedWriter.newLine();
            }

            bufferedWriter.flush();
            bufferedWriter.close();
        } catch (IOException ioException) {
            telemetry.addData("Failed to write to file", ioException.getMessage());
            telemetry.update();

            sleep(2000);
        }

        allAprilTagDetections.clear();

        sleep(2000);
        telemetry.clearAll();
    }

    private void displaySaveInstructions() {
        telemetry.addLine("To save april tag information, press options (the one on the left) on");
        telemetry.addLine("gamepad 1");
    }
}