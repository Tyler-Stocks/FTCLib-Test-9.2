package org.firstinspires.ftc.teamcode.subsystems.intake.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

/**
 * Command to spin the intake in the intake direction. Aside from spinning the intake, the command
 * also triggers the opening of the outtake
 */
public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Constructs a new IntakeCommand
     * @param intakeSubsystem The intake subsystem to run
     */
    public IntakeCommand(
            @NonNull IntakeSubsystem intakeSubsystem
    ) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override public void execute() {
       intakeSubsystem.intake();
    }
}
