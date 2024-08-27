package org.firstinspires.ftc.teamcode.subsystems.intake.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    private boolean isFinished;

    /**
     * Constructs a new stop intake command
     * @param intakeSubsystem The intake subsystem to call the command on
     */
    public StopIntakeCommand(@NonNull IntakeSubsystem intakeSubsystem) {
       this.intakeSubsystem  = intakeSubsystem;

       isFinished = false;

       addRequirements(intakeSubsystem);
    }

    @Override public void execute() {
        intakeSubsystem.stop();
        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }
}
