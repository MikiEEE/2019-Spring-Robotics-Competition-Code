package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.*;
import java.lang.Math;

@Autonomous(name = "hwk4",group = "Concept")

public class Hwk4 extends LinearOpMode {

      //Things Mikey Added
    private static OpMode opmode = null;
    private static final String VUFORIA_KEY = "AbHxg6r/////AAABmYsaQYY0G0YRsP0ypVAoPQ83l6te+3ui/u1twNBe1fZh7NCaijkQSBqEuAiMUIRoRdF2ZreXhc9BejC7wg0vOGQ6khEZArG0C9hcqyyZPsoTkOPIDoNEOfbNbZqgefIwN1VDNxIfgMR0mA9aLxVxohD3OGzuzEqWSSXAZwetHmB2wPyDnE3meo44/fj1QUnfyaJdEb4wsVvjnLeUKM/XMi5Dm2c4R7qV0NLmBQJ8emDQO+udmVajfG6vtN7ZFtTPba2k7XlY5ngK6UQQ6ih493mocvWQh26mrsAF/Hp6JFj2edH3Xr2rblmvDAhUeUMyZyJUg67y3qouO02jC/RG7docCp96PnH4TNqMuB+pvKxV";

     private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (float)(12*2.5) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    public DriveGyro drive;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";




    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

          OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation((24 * mmPerInch), -(16 * mmPerInch), mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(-(24 * mmPerInch), -(26 *  mmPerInch), mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            backSpace.setLocation(backSpaceLocationOnField);

         OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(-(22 * mmPerInch), (14 * mmPerInch), mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation((21 * mmPerInch), (26 *  mmPerInch), mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            frontCraters.setLocation(frontCratersLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = (int)(6.5 * mmPerInch);   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 153;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 12;     // eg: Camera is ON the robot's center line


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                    CAMERA_CHOICE == FRONT ? 90 : -90, -90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /*MY CODE STARTS HERE*/
        //Set Up for AStar
        DistanceMath dMath = new DistanceMath();
        AStar path = new AStar();
        drive = new DriveGyro(hardwareMap, this);
        Hashtable<String, Integer> hash;
        LinkedList<Double> angles = new LinkedList<Double>();
        OpenGLMatrix location = null;
        VectorF translation = null;

        LinkedList<Cell> plan;
        Cell start, finish;
        boolean success = false;
        double[] ideal = null;
        double angle;
        int[] coordinates = null;
        int endingX;
        String name = null;

            //Arrays for Picture locations
        // 1,3 1,4 2,3 2,4
        final double [][] rover = {{-10.9,-16.9},{-9.5,-27.5},{4.9,-17},{4.9,-27.0}};//2:4 is good
        final double [][] space = {{-4.3,-16.1},{-3.2,-23.5},{6.1,-16.3},{4.4,-26.3}};

        //1,0, 1,1 2,0 2,1
        final double [][] craters = {{-4.2,23.1},{-7.5,16.8},{4.2,23.0},{3.3,15.5}};
        final double [][] foot = {{-2.9,24.9},{-5.2,17.0},{6.2,24.3},{5.2,18.0}};


        HashMap<String,double[][]> ideals= new HashMap<String,double[][]>(){
            {
                put("Blue-Rover",rover);
                put("Front-Craters",craters);
                put("Red-Footprint",foot);
                put("Back-Space",space);
            }
        };

        do {
            try {
                location = getMatrix(allTrackables); //Get the location vector for the robot
                translation = location.getTranslation();
                name = getPicName(allTrackables); // Get the name of the picture
                coordinates = findCoordinate(translation.get(0)/mmPerInch,translation.get(1)/mmPerInch,name); //find Astar Grid Coords
                ideal = getIdeal(coordinates[0],coordinates[1],name,ideals); //find the ideal Vuforia Grid Coords
                success = true;
            }
            catch(Exception e){continue;}
        }while(!success);


        //First Centering procedure
        centerPicture(allTrackables, drive,1,.2);  // Make car normal to picture
        correctXY(allTrackables,drive,.20, ideal); // Make car in ideal location given by getIdeal()


        //Find Cube
        endingX = coordinates[0]; // Set ending coordinate to the starting coordinate
        endingX += getXOffset(endingX); // Add or subtract from the ending coordinate to the get the X of the cube
        drive.turnMe(coordinates[0] == 1 ? 90:-90,0.1,1);


        //Second Centering procedure
        centerPicture(allTrackables, drive,1,.2);  // Make car normal to picture
        correctXY(allTrackables,drive,.20, ideal); // Make car in ideal location given by getIdeal()


       // Begin AStar
        plan =  path.getQueue(4, 5, coordinates[0], coordinates[1], endingX, 0, new int[][]{{2,2},{1,2}}); // Get the AStar Path
        start = plan.pollLast(); // Get the startig point

        // Convert the points into a list of angles
        while(!plan.isEmpty()) {
            finish = plan.pollLast();
            angles.add(getAngle(start,finish));
            start = finish;
        }
        goPath(angles,.2,.15); // Go the Astar path

        drive.turnMe(endingX == 1 ? 90:-90,0.1,.1); // Turn to picture closest to the endingX

        name = getPicName(allTrackables); // Get name of the picture
        ideal = getIdeal(endingX,0,name,ideals); // get ideal vuforia coordinates

        //Third Centering procedure
        centerPicture(allTrackables, drive,2,.2);   //Go normal to the picture
        correctXY(allTrackables,drive,.2, ideal);//Correct in XY space
    }



    /*
    Routes and moves the robot along path
    @param tolerance - drive.turn() tolerance double
    */
    public void goPath(LinkedList<Double> angles, double speed, double tolerance) {
        double angle;
        for(;opModeIsActive() && !angles.isEmpty();) {
            angle = angles.poll();
            if(Math.abs(angle) < 1 && Math.abs(angle) > -1) {
              drive.driveMe(speed,12,0);
              continue;
            }
            else if((Math.abs(angle) - 90) < 1) {
                drive.turnMe(angle,speed,tolerance);
                drive.driveMe(speed,12,0);
            }
            drive.turnMe(-angle,speed,tolerance);
        }
        return;
    }

    /*
        Uses vuforia to correct itself on the grid.
    */
    public void correctXY(List<VuforiaTrackable> trackables, DriveGyro drive, double speed, double[] targets){

        boolean success = false;
        OpenGLMatrix location;

        while(!success) {
            try {
                location = getMatrix(trackables);
                VectorF translation = location.getTranslation();
                Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);

                int heading = (int)rotation.thirdAngle;
                double Xcord = translation.get(0)/ mmPerInch;
                double Ycord = translation.get(1)/ mmPerInch;
                double turn;
                double correction;

                if(heading > -1) {
                    turn = 90;
                    correction = targets[0] - Xcord;
                }
                else {
                    turn = -90;
                    correction = Xcord - targets[0];
                }

                drive.driveMe(speed,correction,0);
                drive.turnMe(turn,speed/2, .2);
                correction = targets[1] - Ycord;
                drive.driveMe(speed,correction,0);
                success = true;
            }
            catch(Exception e) {
                telemetry.addData("ERROR",e);
                telemetry.update();
            }
        }
    }

    /*
    returns the name of the picture being viewed.
    */
    public String getPicName(List<VuforiaTrackable> trackables){

        boolean targetVisible = false;
        String name = new String("");

        while(!targetVisible){
            for (VuforiaTrackable trackable : trackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    name = trackable.getName();
                }
            }
        }
        return name;
    }

    /*
    finds the trackable items and returns the openglmatrix of the car's location with
    reference to the picture.
    */
    public OpenGLMatrix getMatrix( List<VuforiaTrackable> trackables) {
        OpenGLMatrix lastLocation = null;
        boolean targetVisible = false;
        while(!targetVisible && lastLocation == null){
            for (VuforiaTrackable trackable : trackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }
            }
        }
        return lastLocation;
    }


    /*
    Makes the car normal to the picture
    @param tolerance for heading different from drive tolerance
    */
    public void centerPicture(List<VuforiaTrackable> trackables, DriveGyro drive, int tolerance,double speed) {
        OpenGLMatrix location = null;
        Orientation rotation = null;
        VectorF translation = null;
        boolean success = false;
        do{
            try{
                location = getMatrix(trackables);
                rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
                translation = location.getTranslation();
                success = true;
            }catch(Exception e) {continue;}
        }while(!success);

        while(!isNormal(rotation, tolerance, speed, drive,speed)){
            try{
                location = getMatrix(trackables);
                rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
            }
            catch(Exception e) {
                continue;
            }
        }
        return;
    }

    /*
    Used by centerPicture(). Determines if car is normal or not.
    */
    public boolean isNormal(Orientation rotation, int tolerance, double speed, DriveGyro drive) {
        boolean isCentered = false;
        int heading = (int)rotation.thirdAngle;
        int error = Math.abs(heading) - 90;
        int normal = heading > 0 ? 90 : -90;

        if (Math.abs(error) > tolerance) {
            if(heading > normal){
                drive.turnRight(speed);
            }
            else {
                drive.turnLeft(speed);
            }
        }
        else {
            isCentered = true;
            drive.stop();
        }
        return isCentered;
    }

      public static OpMode getOpModeInstance() {
            return opmode;
    }

    public void setOpmode() {
            opmode = this;
    }


    public static double getAngle(Cell startC, Cell finishC) {
      Hashtable<String, Integer> start = startC.getCoordinates(); //coordinates of each
      Hashtable<String, Integer> finish = finishC.getCoordinates();

      if(finish.get("Y") - start.get("Y") == -1) {
        return 0.0;
      }
      else if(finish.get("Y") - start.get("Y") == 1) {
        return 0.0;
      }
      else if(finish.get("X") - start.get("X") == -1) {
        return 90.0;
      }
      else if(finish.get("X") - start.get("X") == 1) {
        return -90.0;
      }
      else {
        return 0.0;
      }
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }

    public double getDistance(double dif) {
        double second = Math.pow(dif,2) * .0007;
        double first  = -.377 * dif;
        double coe    = 60.628;
        double total = second + first + coe;
        return Math.abs(total);
    }

    public int getXOffset(int currentX) {
        initTfod();
        int left = 0, count = 0, offset = 0;
        double angle = 0, turnAngle;
        boolean found = false;
        turnAngle = currentX == 1? -40: 40;
        while(!found) {
           left =  relativeCube()[1];
           if(left == -1) {
               switch(count % 2){
                case 0:
                angle = -turnAngle;
                offset = currentX == 1 ? 1: -1;
                drive.turnMe(turnAngle,.1,.1);
                break;
                case 1:
                angle = 0;
                offset = 0;
                drive.turnMe(-turnAngle,.1,.1);
                break;
               }
               count += 1;
               count = count > 1000 ? 0: count;
           }
           else {
               drive.turnMe(angle,.2,.2);
               found = true;
               break;
           }
        }
        tfod.shutdown();
        return offset;
    }

/*
Gets cube's pixel representation on the screen if not seen returns {-1,-1,-1,-1}
*/
    public int[] relativeCube(){
        tfod.activate();
        boolean Going = true;
        int counter = 20000;
        int T = -1;
        int L = -1;
        int B = -1;
        int R = -1;
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        do{
            if(updatedRecognitions != null){
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                       T = (int) recognition.getTop();
                       L = (int) recognition.getLeft();
                       B = (int) recognition.getBottom();
                       R = (int) recognition.getRight();
                       Going = false;
                    }
                }
            }
            updatedRecognitions = tfod.getUpdatedRecognitions();
            counter--;
        }while(Going && counter > -1);
        return new int[]{T,L,B,R};
    }

/*
  Converts the Vuforia cordinates to integers on the Astar map
*/
    public static int[] findCoordinate(double x, double y,String name) {

        int xCoordinate = x > 0 ? 2:1;
        int yCoordinate;

        if(y < -19) {
            yCoordinate = 4;
        }
        else if(y > -19  && y < -6) {
            yCoordinate = 3;
        }
        else if(y > 6 && y < 19) {
            yCoordinate = 1;
        }
        else {
            yCoordinate = 0;
        }

        int[] coordinates = {xCoordinate,yCoordinate};
        return coordinates;
    }

/*
Converts the AStar grid coordinates to vuforia coordinates
*/
    public double[] getIdeal(int x, int y,String name,HashMap<String,double[][]> ideals) {

        //Bottom Row
        if(x == 1 && y == 3)return ideals.get(name)[0];
        if(x == 1 && y == 4)return ideals.get(name)[1];
        if(x == 2 && y == 3)return ideals.get(name)[2];
        if(x == 2 && y == 4)return ideals.get(name)[3];

        //Top Row
        if(x == 1 && y == 0)return ideals.get(name)[0];
        if(x == 1 && y == 1)return ideals.get(name)[1];
        if(x == 2 && y == 0)return ideals.get(name)[2];
        if(x == 2 && y == 1)return ideals.get(name)[3];

        return new double[] {-1,-1};
    }
}
