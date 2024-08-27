package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;

@TeleOp(name = "Test - Spark Fun Odometry", group = "Test")
public final class SparkFunOdometryTest extends OpModeTemplate {
    @Override public void initialize() {
       new ConstantsLoader(telemetry).load();

       initializeHardware();
       configureBindings();
       configureTriggers();

       driveSubsystem.setFloat();
       armSubsystem.disable();

       schedule(
               new RunCommand(this::displayOdometryInformation),
               new RunCommand(telemetry::update)
       );
    }

    @Override protected void configureBindings() {
        new GamepadButton(driverGamepad, OPTIONS)
                .or(new GamepadButton(operatorGamepad, OPTIONS))
                .whenActive(odometrySubsystem::calibrateIMU);

        new GamepadButton(driverGamepad, SHARE)
                .or(new GamepadButton(operatorGamepad, SHARE))
                .whenActive(odometrySubsystem::reset);
    }

    private void displayOdometryInformation() {
        SparkFunOTOS.Pose2D position = odometrySubsystem.position();

        telemetry.addData("Connected", odometrySubsystem.isConnected());
        telemetry.addData("X Position", position.x);
        telemetry.addData("Y Position", position.y);
        telemetry.addData("Heading", position.h);
    }
}
