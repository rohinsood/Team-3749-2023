package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.Arm.ElbowSetpoints;
import frc.robot.utils.Constants.Arm.ShoulderSetpoints;

public class ArmTeleopCommand extends CommandBase {

    private final Arm arm;
    private final Xbox xbox;
    private double[] desired_setpoint;
    private double[] current_setpoint;
    private boolean reached_sting = false;

    private boolean node_to_node = (current_setpoint == ArmSetpoints.CONE_TOP.angles
            || current_setpoint == ArmSetpoints.CONE_MID.angles)
            && (desired_setpoint == ArmSetpoints.CONE_TOP.angles
                    || desired_setpoint == ArmSetpoints.CONE_MID.angles);
    private boolean to_double_sub = current_setpoint == ArmSetpoints.DOUBLE_SUBSTATION.angles
            || desired_setpoint == ArmSetpoints.DOUBLE_SUBSTATION.angles;
    private boolean top_intake_to_stowed = (current_setpoint == ArmSetpoints.TOP_INTAKE.angles
            || current_setpoint == ArmSetpoints.STOWED.angles)
            && (desired_setpoint == ArmSetpoints.TOP_INTAKE.angles
                    || desired_setpoint == ArmSetpoints.STOWED.angles);

    public ArmTeleopCommand(Arm arm, Xbox xbox) {
        this.arm = arm;
        this.xbox = xbox;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        desired_setpoint = ArmSetpoints.STOWED.angles;
    }

    @Override
    public void execute() {
        if (xbox.a().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.DOUBLE_SUBSTATION.angles;
        } else if (xbox.b().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.STOWED.angles;
        } else if (xbox.x().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.CONE_MID.angles;
        } else if (xbox.y().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.CONE_TOP.angles;
        } else if (xbox.rightBumper().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.TOP_INTAKE.angles;
        }

        // if moving form node to node, to the double substation, or from the top_intake
        // to stowed position AND if the arm is not already at its stung position, the
        // arm will not move to its sting position
        if (!node_to_node || !to_double_sub || !top_intake_to_stowed && !reached_sting) {
            arm.setArmAngle(ShoulderSetpoints.STING.angle, ElbowSetpoints.STING.angle);
            reached_sting = arm.getShoulderAtSetpoint() && arm.getElbowAtSetpoint();
            return;
        }

        arm.setArmAngle(desired_setpoint[0], desired_setpoint[1]);

        current_setpoint = desired_setpoint;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
