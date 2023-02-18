package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveArmPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double shoulder_setpoint;
    private final double elbow_setpoint;

    public MoveArmPID(Arm arm, double shoulderSetpoint, double elevatorSetpoint) {
        this.arm = arm;
        this.shoulder_setpoint = shoulderSetpoint;
        this.elbow_setpoint = elevatorSetpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setShoulderAngle(shoulder_setpoint);
        arm.setElbowAngle(elbow_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.getShoulderAtSetpoint() && arm.getElbowAtSetpoint();
    }
}
