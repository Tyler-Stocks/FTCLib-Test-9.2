package org.firstinspires.ftc.teamcode.subsystems.drive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveRobotCentricCommand extends CommandBase {
    private final DriveSubsystem drivebase;
    private final DoubleSupplier driveSupplier,
                                 strafeSupplier,
                                 turnSupplier;

    /**
     * Command To Drive The Robot Relative To Itself. Doesn't finish unless interrupted
     * @param drivebase      The drivebase subsystem to move
     * @param driveSupplier  The supplier for the drive value
     * @param strafeSupplier The supplier for the strafe value
     * @param turnSupplier   The supplier for the turn value
     */
    public DriveRobotCentricCommand(
            @NonNull DriveSubsystem drivebase,
            @NonNull DoubleSupplier driveSupplier,
            @NonNull DoubleSupplier strafeSupplier,
            @NonNull DoubleSupplier turnSupplier
    ) {
        this.drivebase      = drivebase;
        this.driveSupplier  = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier   = turnSupplier;

        addRequirements(drivebase);
    }

    @Override public void execute() {
        this.drivebase.driveRobotCentric(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble());
    }
}
