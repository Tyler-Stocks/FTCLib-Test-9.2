package org.firstinspires.ftc.teamcode.subsystems.intake.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {
   private final IntakeSubsystem intakeSubsystem;

    /**
     * Constructs a new outtake command
     * @param intakeSubsystem The intake subsystem to rely on
     */
   public OuttakeCommand(@NonNull IntakeSubsystem intakeSubsystem) {
       this.intakeSubsystem = intakeSubsystem;

       addRequirements(intakeSubsystem);
   }

   @Override public void execute() {
       intakeSubsystem.outtake();
   }
}
