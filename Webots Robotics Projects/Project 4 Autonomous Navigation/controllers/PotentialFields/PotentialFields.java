
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;

public class PotentialFields {

  
  static final double GOAL_X = -0.375;
  static final double GOAL_Y = 0.375;

  
  static final double GOAL_TOLERANCE = 0.03;

  static final double MAX_SPEED = 6.28;
  static final double FORWARD_SPEED = 3.0;
  static final double TURN_GAIN = 2.5;

  static final double ATTRACTION_GAIN = 1.0;
  static final double REPULSION_GAIN = 2.5;

  static final double SENSOR_THRESHOLD = 70.0;
  static final double SENSOR_MAX = 1000.0;

  static final double[] SENSOR_ANGLES = {
    -0.30, -0.80, -1.57, -2.60,
     2.60,  1.57,  0.80,  0.30
  };

  public static void main(String[] args) {

    Robot robot = new Robot();

    int TIME_STEP =
        (int) Math.round(robot.getBasicTimeStep());

    Motor leftMotor =
        robot.getMotor("left wheel motor");

    Motor rightMotor =
        robot.getMotor("right wheel motor");

    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);

    // Initialize proximity sensors
    DistanceSensor[] ps = new DistanceSensor[8];

    for (int i = 0; i < 8; i++) {
      ps[i] = robot.getDistanceSensor("ps" + i);
      ps[i].enable(TIME_STEP);
    }

    // Initialize GPS
    GPS gps = robot.getGPS("gps");

    // Initialize Compass
    Compass compass = robot.getCompass("compass");

    // Check that positioning sensors exist
    if (gps == null || compass == null) {
      System.err.println(
          "ERROR: GPS or Compass not found!");
      System.err.println(
          "Check the e-puck turretSlot.");
      return;
    }

    gps.enable(TIME_STEP);
    compass.enable(TIME_STEP);

    System.out.println(
        "Potential Fields navigation started.");

    boolean goalReached = false;

 
    while (robot.step(TIME_STEP) != -1) {



      double[] position = gps.getValues();

      double x = position[0];
      double y = position[1];

      double dx = GOAL_X - x;
      double dy = GOAL_Y - y;

      double distance = Math.hypot(dx, dy);

 

      if (!goalReached &&
          distance <= GOAL_TOLERANCE) {

        goalReached = true;

        System.out.println("GOAL REACHED!");
        System.out.println(
            "Final position: " + x + ", " + y);
      }

  
      if (goalReached) {
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);
        continue;
      }



      double attractiveX =
          ATTRACTION_GAIN * dx / distance;

      double attractiveY =
          ATTRACTION_GAIN * dy / distance;

  

      double[] north = compass.getValues();

      double northAngle =
          Math.atan2(north[1], north[0]);

      double heading =
          Math.PI / 2.0 - northAngle;

    

      double localAttractiveX =
          attractiveX * Math.cos(heading) +
          attractiveY * Math.sin(heading);

      double localAttractiveY =
         -attractiveX * Math.sin(heading) +
          attractiveY * Math.cos(heading);

 

      double repulsiveX = 0.0;
      double repulsiveY = 0.0;

      double frontObstacle = 0.0;

      for (int i = 0; i < 8; i++) {

        double reading = ps[i].getValue();

    
        double strength =
            (reading - SENSOR_THRESHOLD) /
            (SENSOR_MAX - SENSOR_THRESHOLD);

        strength = Math.max(
            0.0, Math.min(1.0, strength));

        double angle = SENSOR_ANGLES[i];

        repulsiveX -=
            REPULSION_GAIN * strength *
            Math.cos(angle);

        repulsiveY -=
            REPULSION_GAIN * strength *
            Math.sin(angle);

      
        if (i == 0 || i == 7) {
          frontObstacle =
              Math.max(frontObstacle, strength);
        }
      }


      double totalX =
          localAttractiveX + repulsiveX;

      double totalY =
          localAttractiveY + repulsiveY;

 
      double error =
          Math.atan2(totalY, totalX);

   
      double forwardSpeed =
          FORWARD_SPEED *
          Math.max(0.0, Math.cos(error));

    
      forwardSpeed *=
          (1.0 - 0.85 * frontObstacle);

     
      if (distance < 0.10) {
        forwardSpeed *= Math.max(
            0.25, distance / 0.10);
      }

      double turnSpeed =
          TURN_GAIN * error;

      double leftSpeed =
          forwardSpeed - turnSpeed;

      double rightSpeed =
          forwardSpeed + turnSpeed;

   
      leftSpeed = Math.max(
          -MAX_SPEED,
          Math.min(MAX_SPEED, leftSpeed));

      rightSpeed = Math.max(
          -MAX_SPEED,
          Math.min(MAX_SPEED, rightSpeed));

   
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }

 
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
  }
}
