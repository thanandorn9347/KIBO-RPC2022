package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.Point3;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    static Point pos_navcam = new Point(0.1177, -0.0422, -0.0826);
    static Point pos_laserpointer = new Point(0.1302, 0.0572, -0.1111);

    @Override
    protected void runPlan1() {
        // the mission starts

        api.startMission();
        double[][] camera_param = api.getNavCamIntrinsics();
        double[] cameraMatrix = camera_param[0];
        double[] distCoeffs = camera_param[1];
        System.out.println("cameraMatrix: " + cameraMatrix[0] + " " + cameraMatrix[1] + " " + cameraMatrix[2] + " " + cameraMatrix[3] + " " + cameraMatrix[4] + " " + cameraMatrix[5] + " " + cameraMatrix[6] + " " + cameraMatrix[7] + " " + cameraMatrix[8] + " ");
        System.out.println("distCoeffs: " + distCoeffs[0] + " " + distCoeffs[1] + " " + distCoeffs[2] + " " + distCoeffs[3] + " " + distCoeffs[4] + " ");
        // move to a point
        moveToWrapper(10.68 + 0.0285, -7.675 - 0.0994, 4.32, 0, 0.707, 0, 0.707);
//        moveToWrapper(10.6031 + 0.0285, -7.71007 - 0.0994, 3.76093, 0, 0.707, 0, 0.707);
//        moveToWrapper(10.6031 + 0.1111, -7.71007 + 0.0572 , 4.32, 0, 0.707, 0, 0.707);

        api.reportPoint1Arrival();
        api.laserControl(true);
        api.takeTarget1Snapshot();
//        Bitmap temp = api.getBitmapNavCam();
//        api.saveBitmapImage(temp, "temp.jpg");
        api.laserControl(false);

//        moveToWrapper(11.2, -9.45, 4.6872966, 0, 0, -0.707, 0.707);
        moveToWrapper(11.2, -9.45, 4.6872966, -0.153, -0.153, -0.690, 0.690);

        moveToWrapper(11.2625 - pos_navcam.getY(), -9.92284, 5.3625 - pos_navcam.getZ(), 0, 0, -0.7071068, 0.7071068);

        Mat image = api.getMatNavCam();
        Point point2_pos = new Point(11.2625-   pos_navcam.getY(), -9.92284, 5.3625 - pos_navcam.getZ());
        Quaternion point2_quaternion = new Quaternion(0, 0, -0.7071068f, 0.7071068f);
        double[] fixed_tar_pos = LaserPoint.calcTargetQuaternion_reverse(point2_pos);
        Point3 _target_tvec_cam_frame = PoseEstimation.target2_poseEst(image, cameraMatrix, distCoeffs, fixed_tar_pos);
        Point target_tvec_cam_frame = new Point(_target_tvec_cam_frame.x, _target_tvec_cam_frame.y, _target_tvec_cam_frame.z);
        QuaternionCal laser_quaternion_cal = new QuaternionCal(0, 0, -0.7071068f, 0.7071068f);
        Point target_pos_rel = LaserPoint.calcTargetPositionRelative(target_tvec_cam_frame);

        moveToWrapper(point2_pos.getX() + target_pos_rel.getX() - pos_laserpointer.getY(),
                -10, point2_pos.getZ() + target_pos_rel.getZ() - pos_laserpointer.getZ(),
                laser_quaternion_cal.getX(), laser_quaternion_cal.getY(),
                laser_quaternion_cal.getZ(), laser_quaternion_cal.getW());

        api.laserControl(true);
        api.takeTarget2Snapshot();
        api.laserControl(false);
//        api.saveMatImage(image, "point2");
//        api.saveMatImage(PoseEstimation.cropped_img, "point2_cropped");

        moveToWrapper(10.944, -9.43019, 4.96538, 0, 0, -0.707, 0.707);
        moveToWrapper(11.27460, -7.89178, 4.96538, 0, 0, -0.707, 0.707);

        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {
        final int LOOP_MAX = 20;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result;
        int loopCounter = 0;
        do {
            result = api.moveTo(point, quaternion, false);
            if (result == null) {
               System.out.println("moveToWrapper  "+ "result==null");
                break;
            } else if (result.getMessage().contains("Keep out zone violation")) {
                System.out.println("moveToWrapper  "+ "result==KOZ");
                break;
            } else if (result.getMessage().contains("Keep in zone violation")) {
                System.out.println("moveToWrapper  "+ "result==KIZ");
                break;
            }

            ++loopCounter;

        } while (!result.hasSucceeded() && loopCounter < LOOP_MAX);
    }


}

