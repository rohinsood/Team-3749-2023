package frc.robot.utils;

/***
 * @author Anusha Khobare
 * @author Ryan R McWeeny
 * 
 *     ColorSensor.java is color sensor code to differniate between the yellow cone and purple cube gamepieces.
 *     Note that this ultility is currently unused in claw code. But is ready for possible future use (use the gamePiece Function).
 *     Rearranged + Adapted code from RevRobotics Color Sensor Template Code
 */

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants;

public class ColorSensor {
    // defines color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    public Claw claw; //object

    public ColorSensor() {
        // adds colors from constants to array with all colors
        m_colorMatcher.addColorMatch(Constants.cone_color);
        m_colorMatcher.addColorMatch(Constants.cube_color);
    }

    // Constants.Claw.Object is a String (see Constants.java)
    // States object obtained (possible objects: "Cone", "Cube", None)
    // is in the claw (returned in "gamePiece" method)
    public static String gamePiece(){
        // .getColor gets RGB values at the current time (what the sensor sees)
        Color detectedColor = m_colorSensor.getColor();
        
        //.matchClosestColor() caluclates the closest color from the listed colors in Color Matcher array
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        Smartdashboard.
        if (match.color == Constants.cone_color) {
            //If cone_color detected, it is cone
            Constants.Claw.Object = "Cone";
            return Constants.Claw.Object;
        }
        else if (match.color == Constants.cube_color) {
            //If cube_color detected, it is  cube
            Constants.Claw.Object = "Cube";
            return Constants.Claw.Object;
        }
        else {
            //If color detected falls out of line of margin from .matchClosestColor(), then nothing is detected
            Constants.Claw.Object = "None";
            return Constants.Claw.Object;
        }
    }

    public static Boolean autostop() {
        if (gamePiece() == "None") {
            return false;
        } else {
            return true;
            claw.setSpeed(1.2);
        }
    }
}
