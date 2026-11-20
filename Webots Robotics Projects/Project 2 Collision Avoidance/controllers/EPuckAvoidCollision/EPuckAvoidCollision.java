import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

public class EPuckAvoidCollision {

  public static void main(String[] args) {

    int TIME_STEP = 64;
    int MAX_SPEED = 6;

    // create the Robot instance
    Robot robot = new Robot();

    // initialize devices
    DistanceSensor[] ps = new DistanceSensor[8];
    String[] psNames = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };

    for (int i = 0; i < 8; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(TIME_STEP);
    }

    // initialize motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");

    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);

    // keeps the robot turning for several steps
    int turnCounter = 0;

    // feedback loop
    while (robot.step(TIME_STEP) != -1) {

      // read sensor outputs
      double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};

      for (int i = 0; i < 8; i++) {
        psValues[i] = ps[i].getValue();
      }

      // detect obstacles
      boolean right_obstacle =
        psValues[0] > 80.0 ||
        psValues[1] > 80.0 ||
        psValues[2] > 80.0;

      boolean left_obstacle =
        psValues[5] > 80.0 ||
        psValues[6] > 80.0 ||
        psValues[7] > 80.0;

      // normal forward movement
      double leftSpeed = 0.5 * MAX_SPEED;
      double rightSpeed = 0.5 * MAX_SPEED;

      // continue an escape turn
      if (turnCounter > 0) {

        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = -0.5 * MAX_SPEED;

        turnCounter--;
      }

      // obstacle detected on both sides
      else if (left_obstacle && right_obstacle) {

        // begin a longer turn
        turnCounter = 15;

        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = -0.5 * MAX_SPEED;
      }

      // obstacle on left
      else if (left_obstacle) {

        // turn right
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = -0.5 * MAX_SPEED;
      }

      // obstacle on right
      else if (right_obstacle) {

        // turn left
        leftSpeed = -0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
      }

      // send speeds to motors
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}