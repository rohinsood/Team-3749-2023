package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveShoulderPID extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Arm arm;
    private final double shoulder_setpoint;

    public MoveShoulderPID(Arm arm, double shoulder_setpoint) {
        this.arm = arm;
        this.shoulder_setpoint = shoulder_setpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setShoulderAngle(shoulder_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.getShoulderAtSetpoint();
    }
}
