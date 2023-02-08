
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/***
 * @author Anusha Khobare
 * @author Aashray Reddy
 * @author Ryan R McWeeny
 * @author Hanlun Li
 * @author Zaddy Harkirat
 * 
 *     Claw.java creates objects, dependencies, and motor controller groups to allow us to set the speed of each motor for intake and outtake    
 */

public class Claw extends SubsystemBase {
    // Creates a PIDController with gains kP, kI, and kD
    private final PIDController claw_PID = new PIDController(Constants.Claw.claw_kP, Constants.Claw.claw_kI, Constants.Claw.claw_kD);

    // left and right side of the claw (the motor)
    private final CANSparkMax right_motor = new CANSparkMax(Constants.Claw.right_side, MotorType.kBrushless);
    private final CANSparkMax left_motor = new CANSparkMax(Constants.Claw.left_side, MotorType.kBrushless);

    // motor controller group for both sides
    private final MotorControllerGroup clawMotors = new MotorControllerGroup(left_motor, right_motor);

    // relative encoder
    private final RelativeEncoder right_encoder = right_motor.getEncoder();
    private final RelativeEncoder left_encoder = left_motor.getEncoder();
    // Initializes the base subsystem
    public Claw() {
        right_motor.setInverted(true); // invert the motor to not break it

        right_motor.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active
        left_motor.setIdleMode(IdleMode.kBrake); // set neo to be braked when not active

    }
    
    /**
     * averages the Encoder velocities from both left and right encoders.(gives error if method is not set to return double)
     * @return
     */
    public double AVG_encoder_values(){
        final double encoder_AVG = (left_encoder.getVelocity() + right_encoder.getVelocity())/2;
        return(encoder_AVG);
    }
        
    /**
     * set speed for motor
     * @param setpoint_velocity
     * This parmeter is to tell the PID calculator what the ideal speed is, and how to get there.
     * @return 
     */
    public void setSpeed(double setpoint_velocity) {
        clawMotors.set(claw_PID.calculate(AVG_encoder_values(), Constants.Claw.setpoint_velocity));
    }   

    // Runs every 20 ms
    @Override
    public void periodic() {
    }

}
