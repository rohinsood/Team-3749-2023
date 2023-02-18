package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveElbowPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double elbow_setpoint;

    public MoveElbowPID(Arm arm, double elbow_setpoint) {
        this.arm = arm;
        this.elbow_setpoint = elbow_setpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setElbowAngle(elbow_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.getElbowAtSetpoint();
    }
}
