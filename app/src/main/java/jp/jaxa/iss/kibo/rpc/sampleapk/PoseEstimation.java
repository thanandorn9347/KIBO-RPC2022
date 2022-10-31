package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Board;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

class PoseEstimation {
    private static ArBoard t_board1 = new ArBoard();
    private static ArBoard t_board2 = new ArBoard();
    private static Mat camMatrix;
    private static Mat dstMatrix;
    private static MatOfDouble distortion;
    private static List<Mat> objP2;
    private static MatOfInt board2ID;


    private static Mat read_img;
    static Mat cropped_img;
    private static int cropped_x_pix;
    private static int cropped_y_pix;
    private static Point3 target_tvec_cam;

    private static void setCamCalibration(double[] _cameraMatrix, double[] _distCoeffs) {
        CamParams camparams = new CamParams(_cameraMatrix, _distCoeffs);
        camMatrix = camparams.getCamMatrix();
        dstMatrix = camparams.getDistortionMat();
        double[] distortionArray = camparams.getDistortionArray();
        distortion = new MatOfDouble();
        distortion.fromArray(distortionArray);

    }

    private static void setBoard() {

        t_board1.set_target_board1();
        t_board2.set_target_board2();
        List<Mat> objP1 = t_board1.getObjP1();
        MatOfInt board1ID = t_board1.getBoard1ID();

        objP2 = t_board2.getObjP2();
        board2ID = t_board2.getBoard2ID();

    }

    private static void find_paper(List<Point> src_pts) {

        List<Integer> list_x = new ArrayList<Integer>();
        List<Integer> list_y = new ArrayList<Integer>();

        for (int i = 0; i < 4; i++) {
            list_x.add((int) src_pts.get(i).x);
            list_y.add((int) src_pts.get(i).y);
        }
        Collections.sort(list_x);
        Collections.sort(list_y);


//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3

        MatOfPoint2f _pts = new MatOfPoint2f();
        _pts.fromList(src_pts);
        cropped_ROI(read_img, _pts);
    }

    private static void cropped_ROI(Mat img, MatOfPoint2f _pts) {
        Rect target_rect = Imgproc.boundingRect(_pts);
        cropped_img = new Mat();
        cropped_img = img.submat(target_rect);
        cropped_x_pix = target_rect.x;
        cropped_y_pix = target_rect.y;
    }

    private static MatOfPoint3f _getSingleMarkerObjectPoints(float markerLength, Mat ids, double[] fixed_target_pos) {
        // set coordinate system in the middle of the marker, with Z pointing out
        List<Point3> points3d = new ArrayList<>();
        double trans_x = fixed_target_pos[0];
        double trans_y = fixed_target_pos[1];
        double trans_z = fixed_target_pos[2];
        if (markerLength > 0) {
            for (int i = 0; i < ids.height(); i++) {
                int id = (int) ids.get(i, 0)[0];

                Point3 aruco_offset_to_target;
                if (id == 13) {
                    aruco_offset_to_target = new Point3(-0.1125 + trans_x, 0.0415 + trans_y, trans_z);

                } else if (id == 14) {
                    aruco_offset_to_target = new Point3(0.1125 + trans_x, 0.0415 + trans_y, trans_z);

                } else if (id == 11) {
                    aruco_offset_to_target = new Point3(0.1125 + trans_x, -0.0415 + trans_y, trans_z);

                } else if (id == 12) {
                    aruco_offset_to_target = new Point3(-0.1125 + trans_x, -0.0415 + trans_y, trans_z);

                } else {
                    System.out.println("_getSingleMarkerObjectPoints" + "error_get single markers");
                    break;
                }
                points3d.add(new Point3((-markerLength / 2.f) + aruco_offset_to_target.x, (-markerLength / 2.f) + aruco_offset_to_target.y, aruco_offset_to_target.z));
                points3d.add(new Point3((markerLength / 2.f) + aruco_offset_to_target.x, (-markerLength / 2.f) + aruco_offset_to_target.y, aruco_offset_to_target.z));
                points3d.add(new Point3((markerLength / 2.f) + aruco_offset_to_target.x, (markerLength / 2.f) + aruco_offset_to_target.y, aruco_offset_to_target.z));
                points3d.add(new Point3((-markerLength / 2.f) + aruco_offset_to_target.x, (markerLength / 2.f) + aruco_offset_to_target.y, aruco_offset_to_target.z));
            }


        } else {
            System.out.println("Point3:" + "null");
            return null;
        }
        MatOfPoint3f mat_points3d = new MatOfPoint3f();
        mat_points3d.fromList(points3d);
        return mat_points3d;
    }

    static Point3 target2_poseEst(Mat _read_img, double[] _cameraMatrix, double[] _distCoeffs, double[] target_tvec_cam_frame) {
        setCamCalibration(_cameraMatrix, _distCoeffs);
        setBoard();
        read_img = _read_img;
        ArrayList<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        int dictID = Aruco.DICT_5X5_250;
        Dictionary dict = Aruco.getPredefinedDictionary(dictID);
        List<Mat> rejectedImg = new ArrayList<>();
        DetectorParameters parameters = DetectorParameters.create();
        ////////////////////////////////////////////////////
        Board t2_board = Board.create(objP2, dict, board2ID);
        ////////////////////////////////////////////////////
        Aruco.detectMarkers(read_img, dict, corners, ids, parameters, rejectedImg, camMatrix, dstMatrix);
        if (ids.empty()) {
            return new Point3(target_tvec_cam_frame[0],target_tvec_cam_frame[1],target_tvec_cam_frame[2]);
        }

        MatOfPoint3f real_objPoints = _getSingleMarkerObjectPoints(0.05f, ids, target_tvec_cam_frame);
        int counter = 0;
        int number_points = corners.size() * corners.get(0).cols();
        double[] pix_x_x = new double[number_points];
        double[] pix_x_y = new double[number_points];
        double[] pix_y_x = new double[number_points];
        double[] pix_y_y = new double[number_points];
        for (int i = 0; i < corners.size(); i++) {

            MatOfPoint2f un_target_pos_pix = new MatOfPoint2f();
            for (int k = 0; k < corners.get(0).cols(); k++) {

                double p_x = corners.get(i).get(0, k)[0];
                double p_y = corners.get(i).get(0, k)[1];
                MatOfPoint2f pix = new MatOfPoint2f(new Point(p_x, p_y));
                Calib3d.undistortPoints(pix, un_target_pos_pix, camMatrix, dstMatrix);
                double z_c = target_tvec_cam_frame[2];
                MatOfPoint3f point_cam = new MatOfPoint3f(new Point3(un_target_pos_pix.get(0, 0)[0] * z_c, un_target_pos_pix.get(0, 0)[1] * z_c, z_c));
                double err_x = point_cam.get(0, 0)[0] - real_objPoints.get(counter, 0)[0];
                double err_y = point_cam.get(0, 0)[1] - real_objPoints.get(counter, 0)[1];
                pix_x_x[counter] = p_x;
                pix_x_y[counter] = err_x;
                pix_y_x[counter] = p_y;
                pix_y_y[counter] = err_y;

                counter += 1;
            }

        }
        LinearRegression lr_pix_x = new LinearRegression(pix_x_x, pix_x_y);
        LinearRegression lr_pix_y = new LinearRegression(pix_y_x, pix_y_y);

        Mat t2_rvec = new Mat();
        Mat t2_tvec = new Mat();
        Aruco.estimatePoseBoard(corners, ids, t2_board, camMatrix, dstMatrix, t2_rvec, t2_tvec);
        t_board2.find_ROI3D(t2_rvec, t2_tvec);

        List<MatOfPoint3f> offset = t_board2.get_offset();
        MatOfPoint2f _targetImagePlane = new MatOfPoint2f();
        Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
        Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);
        double[] _r = new double[]{0.0f, 0.0f, 0.0f};
        double[] _t = new double[]{0.0f, 0.0f, 0.0f};
        _rvec.put(0, 0, _r);
        _tvec.put(0, 0, _t);
        List<Point> ROI_points = new ArrayList<>();
        for (int i = 0; i < 4; i++) {

            Calib3d.projectPoints(offset.get(i), _rvec, _tvec, camMatrix, distortion, _targetImagePlane);
            int _cpx = (int) _targetImagePlane.get(0, 0)[0];
            int _cpy = (int) _targetImagePlane.get(0, 0)[1];
            Point _center = new Point(_cpx, _cpy);
            ROI_points.add(_center);
        }
        find_paper(ROI_points);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat binaryImg = new Mat();
        Imgproc.threshold(cropped_img, binaryImg, 100, 200, Imgproc.THRESH_BINARY_INV);
        Imgproc.findContours(binaryImg, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        for (int i = 0; i < contours.size(); i++) {

            if (hierarchy.get(0, i)[2] == -1.0) {
                MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
                Moments moment = Imgproc.moments(ct2f);

                double x =  (moment.get_m10() / moment.get_m00());
                double y =  (moment.get_m01() / moment.get_m00());
                double full_img_x = x + cropped_x_pix;
                double full_img_y = y + cropped_y_pix;
                double err_x = full_img_x * lr_pix_x.slope() + lr_pix_x.intercept();
                double err_y = full_img_y * lr_pix_y.slope() + lr_pix_y.intercept();
                MatOfPoint2f target_pos_pix = new MatOfPoint2f(new Point(full_img_x, full_img_y));
                MatOfPoint2f un_target_pos_pix = new MatOfPoint2f();
                Calib3d.undistortPoints(target_pos_pix, un_target_pos_pix, camMatrix, dstMatrix);

                double z_c = target_tvec_cam_frame[2];
                target_tvec_cam = new Point3(
                        (un_target_pos_pix.get(0, 0)[0] * z_c) - err_x,
                        (un_target_pos_pix.get(0, 0)[1] * z_c) - err_y,
                        z_c);

            }

        }
        return target_tvec_cam;

    }
}
