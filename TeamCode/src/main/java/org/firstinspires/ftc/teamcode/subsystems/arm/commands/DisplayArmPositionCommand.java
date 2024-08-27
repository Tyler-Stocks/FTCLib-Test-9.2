package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class DisplayArmPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final Telemetry telemetry;

    /**
     * Command to display only the position information
     * @param armSubsystem The ArmSubsystem to read the positions of
     * @param telemetry The telemetry to displays the information on
     */
    public DisplayArmPositionCommand(
            @NonNull ArmSubsystem armSubsystem,
            @NonNull Telemetry telemetry) {
       this.armSubsystem = armSubsystem;
       this.telemetry    = telemetry;
    }

    @Override public void execute() {
       telemetry.addLine("Note: If you are measuring the encoder positions of the arm, look at");
       telemetry.addLine("the top 2 values");

       telemetry.addData(
               "Worm Position",
               armSubsystem.wormPosition());
       telemetry.addData(
               "Elevator Position",
               armSubsystem.elevatorPosition());
       telemetry.addData(
               "Worm Motor Target Position",
               armSubsystem.wormTargetPosition());
       telemetry.addData(
               "Elevator Motor Target Position",
               armSubsystem.elevatorTargetPosition());
       telemetry.addData(
               "Worm Motor Target Position (Local)",
               armSubsystem.localWormTargetPosition());
       telemetry.addData(
              "Elevator Motor Target Position (Local)",
              armSubsystem.localElevatorTargetPosition());
       telemetry.update();
    }
}
