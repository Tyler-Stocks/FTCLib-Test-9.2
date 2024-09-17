package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.Constants.IntakeConstants.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;
import static org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide.*;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;
import org.firstinspires.ftc.teamcode.playstationcontroller.Triggers.*;
import org.firstinspires.ftc.teamcode.playstationcontroller.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.triggers.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.hanger.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.triggers.*;
import org.firstinspires.ftc.teamcode.subsystems.launcher.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.commands.*;

// The group is set to "a" so it always goes to the top of the list
@TeleOp(name = "Use This One", group = "a")
public final class TeleOpMain extends OpModeTemplate {

    @Override public void initialize() {
       new ConstantsLoader().load();

       initializeHardware();

       forceStopTelemetry();
       configureBindings();
       configureTriggers();

        schedule(
                new DriveRobotCentricCommand(
                       driveSubsystem,
                       driverGamepad::getLeftY,
                       driverGamepad::getLeftX,
                       driverGamepad::getRightX
                )
        );
    }

    @Override protected void configureBindings() {
        // -----------------------------------------------------------------------------------------
        // Endgame Triggers
        // -----------------------------------------------------------------------------------------

        new GamepadButton(operatorGamepad, SHARE)
                .whenPressed(new ToggleOpModeCommand(this))
                .whenPressed(new RumbleCommand(gamepad1, gamepad2));

        new GamepadButton(operatorGamepad, LEFT_BUMPER).and(new EndgameTrigger(this))
                .whenActive(new ReleaseLauncherCommand(launcherSubsystem));

        new GamepadButton(operatorGamepad, RIGHT_BUMPER).and(new EndgameTrigger(this))
                .whenActive(new ReleaseHangerCommand(hangerSubsystem));

        // -----------------------------------------------------------------------------------------
        // Outtake Triggers
        // -----------------------------------------------------------------------------------------

        new GamepadButton(operatorGamepad, LEFT_BUMPER)
                .toggleWhenActive(
                        new OpenOuttakeCommand(armSubsystem, LEFT, OPEN_POSITION),
                        new OpenOuttakeCommand(armSubsystem, LEFT, CLOSED_POSITION)
                );

        new GamepadButton(operatorGamepad, RIGHT_BUMPER)
                .toggleWhenActive(
                        new OpenOuttakeCommand(armSubsystem, RIGHT, OPEN_POSITION),
                        new OpenOuttakeCommand(armSubsystem, LEFT, CLOSED_POSITION)
                );

        // -----------------------------------------------------------------------------------------
        // Intake Triggers
        // -----------------------------------------------------------------------------------------

        new LeftGamepadTrigger(INTAKE_TRIGGER_THRESHOLD, operatorGamepad)
                .toggleWhenPressed(
                        new IntakeCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        new RightGamepadTrigger(OUTTAKE_TRIGGER_THRESHOLD, operatorGamepad)
                .toggleWhenPressed(
                        new IntakeCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        // -----------------------------------------------------------------------------------------
        // Arm Bindings
        // -----------------------------------------------------------------------------------------

        new GamepadButton(operatorGamepad, DPAD_DOWN)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem, 0, 0));

        new GamepadButton(operatorGamepad, CROSS)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem,
                        BOTTOM_WORM_POSITION, BOTTOM_ELEVATOR_POSITION));

        new GamepadButton(operatorGamepad, SQUARE)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem,
                        LOW_WORM_POSITION, LOW_ELEVATOR_POSITION));

        new GamepadButton(operatorGamepad, CIRCLE)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem,
                        MEDIUM_WORM_POSITION, MEDIUM_ELEVATOR_POSITION));

        new GamepadButton(operatorGamepad, TRIANGLE)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem,
                        HIGH_WORM_POSITION, HIGH_ELEVATOR_POSITION));

        new GamepadButton(operatorGamepad, OPTIONS)
                .whenPressed(new SetArmTargetPositionCommand(armSubsystem,
                        TOP_WORM_POSITION, TOP_ELEVATOR_POSITION));

        // -----------------------------------------------------------------------------------------
        // Mosaic Fixer Bindings
        // -----------------------------------------------------------------------------------------

        new GamepadButton(driverGamepad, LEFT_BUMPER)
                .toggleWhenPressed(
                        new DisableMosaicFixerCommand(mosaicFixerSubsystem, LEFT),
                        new EnableMosaicFixerCommand(mosaicFixerSubsystem, RIGHT)
                );

        new GamepadButton(driverGamepad, DPAD_DOWN)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, RETRACTED, LEFT));

        new GamepadButton(driverGamepad, DPAD_LEFT)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, LOW, LEFT));

        new GamepadButton(driverGamepad, DPAD_RIGHT)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, MEDIUM, LEFT));

        new GamepadButton(driverGamepad, DPAD_UP)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, HIGH, LEFT));

        new GamepadButton(driverGamepad, RIGHT_BUMPER)
                .toggleWhenPressed(
                        new DisableMosaicFixerCommand(mosaicFixerSubsystem, RIGHT),
                        new EnableMosaicFixerCommand(mosaicFixerSubsystem, RIGHT)
                );

        new GamepadButton(driverGamepad, CROSS)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, RETRACTED, RIGHT));

        new GamepadButton(driverGamepad, SQUARE)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, LOW, RIGHT));

        new GamepadButton(driverGamepad, CIRCLE)
                .whenPressed(new MoveMosaicFixerCommand(mosaicFixerSubsystem, MEDIUM, RIGHT));
    }

    @Override protected void configureTriggers() {
        // Safety:
        // We don't need to worry about the outtake activating if the runs when the arm is
        // not at home because we disable the intake when the arm is not at home.
        new IntakeIsActiveTrigger(intakeSubsystem)
                .toggleWhenActive(
                        new OpenOuttakeCommand(armSubsystem, BOTH, OPEN_POSITION),
                        new OpenOuttakeCommand(armSubsystem, BOTH, CLOSED_POSITION)
                );

        new ArmIsAtHomeTrigger(armSubsystem)
                .toggleWhenActive(
                        new InstantCommand(intakeSubsystem::enableIntaking),
                        new InstantCommand(intakeSubsystem::disableIntaking)
                );
    }
}
