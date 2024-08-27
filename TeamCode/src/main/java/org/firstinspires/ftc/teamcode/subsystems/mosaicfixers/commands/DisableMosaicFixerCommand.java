package org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public final class DisableMosaicFixerCommand extends CommandBase {
    private final MosaicFixerSubsystem mosaicFixerSubsystem;
    private final RobotSide mosaicFixerSide;

    private boolean isFinished;

    /**
     * Constructs a new DisableMosaicFixerCommand
     * @param mosaicFixerSubsystem The mosaic fixer subsystem to disable the arm of
     * @param mosaicFixerSide Which arm to disable see
     */
    public DisableMosaicFixerCommand(
            @NonNull MosaicFixerSubsystem mosaicFixerSubsystem,
            @NonNull RobotSide mosaicFixerSide
    ) {
        this.mosaicFixerSubsystem = mosaicFixerSubsystem;
        this.mosaicFixerSide      = mosaicFixerSide;

        isFinished = false;

        addRequirements(mosaicFixerSubsystem);
    }

    @Override public void execute() {
        switch (mosaicFixerSide) {
            case LEFT:
                mosaicFixerSubsystem.disableLeftMosaicFixer();
                break;
            case RIGHT:
                mosaicFixerSubsystem.disableRightMosaicFixer();
                break;
            case BOTH:
                mosaicFixerSubsystem.disableLeftMosaicFixer();
                mosaicFixerSubsystem.disableRightMosaicFixer();
                break;
        }

        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }
}
