package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class SetArmTargetPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final int wormTargetPosition, elevatorTargetPosition;

    /**
     * Sets the target position for the arm. If the arm is homing, then the target position will be
     * stored and approached once homing is complete.
     * @param armSubsystem           The arm subsystem to set the target position of
     * @param wormTargetPosition     The target position of the worm
     * @param elevatorTargetPosition The target position of the elevator
     */
    public SetArmTargetPositionCommand(
            @NonNull ArmSubsystem armSubsystem,
            int wormTargetPosition,
            int elevatorTargetPosition)
    {
       this.armSubsystem           = armSubsystem;
       this.wormTargetPosition     = wormTargetPosition;
       this.elevatorTargetPosition = elevatorTargetPosition;
    }

    @Override public void execute() {
        armSubsystem.setTargetPos(wormTargetPosition, elevatorTargetPosition);
    }

    @Override public boolean isFinished() {
        return true;
    }
}
