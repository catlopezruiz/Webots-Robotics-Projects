
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.GPS;

public class MazeRunner {

    static final int TIME_STEP = 64;
    static final double SPEED = 4.0;

    static final double FRONT_LIMIT = 83;
    static final double RIGHT_TARGET = 83;
    static final double RIGHT_TOLERANCE = 15;

    // Maze 2 exit coordinates
    static final double EXIT_X = 0.413;
    static final double EXIT_Y = 0.332;
    static final double EXIT_RADIUS = 0.07;

    public static void main(String[] args) {

        Robot robot = new Robot();

        Motor leftMotor =
            robot.getMotor("left wheel motor");
        Motor rightMotor =
            robot.getMotor("right wheel motor");

        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);

        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);

        DistanceSensor[] ps = new DistanceSensor[8];

        for (int i = 0; i < 8; i++) {
            ps[i] = robot.getDistanceSensor("ps" + i);
            ps[i].enable(TIME_STEP);
        }

        // GPS used only to detect the exit
        GPS gps = robot.getGPS("gps");
        gps.enable(TIME_STEP);

        boolean turningLeft = false;
        boolean completed = false;

        while (robot.step(TIME_STEP) != -1) {

            if (completed) {
                leftMotor.setVelocity(0);
                rightMotor.setVelocity(0);
                continue;
            }

            // Check current position
            double[] position = gps.getValues();

            double x = position[0];
            double y = position[1];

            double distanceToExit = Math.sqrt(
                Math.pow(x - EXIT_X, 2) +
                Math.pow(y - EXIT_Y, 2)
            );

            if (distanceToExit < EXIT_RADIUS) {

                leftMotor.setVelocity(0);
                rightMotor.setVelocity(0);

                completed = true;

                System.out.println(
                    "MAZE 2 COMPLETED! Exit reached!"
                );

                continue;
            }

            // Original distance sensor readings
            double frontRight = ps[0].getValue();
            double frontLeft = ps[7].getValue();
            double rightFront = ps[1].getValue();
            double rightSide = ps[2].getValue();

            boolean frontBlocked =
                frontRight > FRONT_LIMIT ||
                frontLeft > FRONT_LIMIT;

            double leftSpeed;
            double rightSpeed;

            // Original wall-following logic
            if (turningLeft) {
                if (frontRight < FRONT_LIMIT &&
                    frontLeft < FRONT_LIMIT) {
                    turningLeft = false;
                }
            }

            if (frontBlocked && !turningLeft) {
                turningLeft = true;
            }

            if (turningLeft) {

                leftSpeed = -SPEED * 0.5;
                rightSpeed = SPEED * 0.5;

            } else if (
                rightSide < RIGHT_TARGET - RIGHT_TOLERANCE &&
                rightFront < RIGHT_TARGET
            ) {

                leftSpeed = SPEED;
                rightSpeed = SPEED * 0.25;

            } else if (
                rightSide > RIGHT_TARGET + RIGHT_TOLERANCE
            ) {

                leftSpeed = SPEED * 0.65;
                rightSpeed = SPEED;

            } else {

                leftSpeed = SPEED;
                rightSpeed = SPEED;
            }

            leftMotor.setVelocity(leftSpeed);
            rightMotor.setVelocity(rightSpeed);
        }
    }
}


