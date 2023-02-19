package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.Arm.ElbowSetpoints;
import frc.robot.utils.Constants.Arm.ShoulderSetpoints;

public class ArmTeleopCommand extends CommandBase {

    private final Arm arm;
    private final Xbox xbox;
    private ArmSetpoints desired_setpoint = ArmSetpoints.STOWED;
    private ArmSetpoints current_setpoint = desired_setpoint;
    private boolean reached_sting = false;

    private boolean node_to_node = (current_setpoint == ArmSetpoints.CONE_TOP
            || current_setpoint == ArmSetpoints.CONE_MID)
            && (desired_setpoint == ArmSetpoints.CONE_TOP
                    || desired_setpoint == ArmSetpoints.CONE_MID);
    private boolean to_double_sub = current_setpoint == ArmSetpoints.DOUBLE_SUBSTATION
            || desired_setpoint == ArmSetpoints.DOUBLE_SUBSTATION;
    private boolean top_intake_to_stowed = (current_setpoint == ArmSetpoints.TOP_INTAKE
            || current_setpoint == ArmSetpoints.STOWED)
            && (desired_setpoint == ArmSetpoints.TOP_INTAKE
                    || desired_setpoint == ArmSetpoints.STOWED);

    public ArmTeleopCommand(Arm arm, Xbox xbox) {
        this.arm = arm;
        this.xbox = xbox;
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        desired_setpoint = ArmSetpoints.STOWED;
    }
    
    @Override
    public void execute() {
        SmartDashboard.putString("current", current_setpoint.name());
        SmartDashboard.putString("desired", desired_setpoint.name());
        SmartDashboard.putBoolean("reached string", reached_sting);

        if (xbox.a().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.DOUBLE_SUBSTATION;
        } else if (xbox.b().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.STOWED;
        } else if (xbox.x().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.CONE_MID;
        } else if (xbox.y().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.CONE_TOP;
        } else if (xbox.rightBumper().getAsBoolean()) {
            desired_setpoint = ArmSetpoints.TOP_INTAKE;
        }

        // if moving form node to node, to the double substation, or from the top_intake
        // to stowed position AND if the arm is not already at its stung position, the
        // arm will not move to its sting position
        if ((!node_to_node || !to_double_sub || !top_intake_to_stowed) && !reached_sting) {
            arm.setArmAngle(ShoulderSetpoints.STING.angle, ElbowSetpoints.STING.angle);
            reached_sting = arm.getShoulderAtSetpoint() && arm.getElbowAtSetpoint();
            current_setpoint = reached_sting ? ArmSetpoints.STING : current_setpoint;
            return;
        }

        arm.setArmAngle(desired_setpoint.angles[0], desired_setpoint.angles[1]);
        
        current_setpoint = desired_setpoint;
        
        reached_sting = false;
    }

    @Override
    public void end(boolean interrupted) {
        desired_setpoint = ArmSetpoints.STOWED;
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}