package frc.robot.utils;

/***
 * Stores constant variables within subclasses for different subsystems.
 * Such constant values can include motor IDs, motor speed, PID
 * constants, etc...
 */
public final class Constants {

    public static final class Base {
        public static final int neo_id = 3749;
        public static final int falcon_id = 6328;
        public static final SmartData<Double> speed = new SmartData<Double>("Base Speed", 2.54);
    }
    public static final class Lights {
        public static final int led_length = 20; //change this value
        public static final int led_port = 0; //change this value
        public static final int[] light1 = {0, 255, 0};
        public static final int[] light2 = {255, 255, 255};
    }

    public static final class Arm {
        public static final int neo_motor_telescope_port = 13; // Change this value later
        public static final int neo_motor_elevator_port = 15; // Change this value later

        public static final int neo_motor_lower_port = 0; // Change this value later
        public static final int neo_motor_upper_port = 1; // Change this value later

        public static final int neo_motor_telescope_speed = 1; // Change this value later
        public static final int neo_motor_elevator_speed = 1; // Change this value later

        public static final int neo_motor_lower_speed = 1; // Change this value later
        public static final int neo_motor_upper_speed = 1; // Change this value later

        public static final int neo_motor_lower_stop = 0; // Change this value later
        public static final int neo_motor_upper_stop = 0; // Change this value later

        public static final int number_of_motors = 2;

        public static final int kp = 40;
        public static final int ki = 0;
        public static final int kd = 0;

        public static final int max_velocity = 2;
        public static final int max_acceleration = 5;
    }

    public static final class Controller {
        public static final int joystick_port = 0;
        private static final int kMotorPort = 0; //fix
    }
}
