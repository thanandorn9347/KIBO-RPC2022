package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

 class CamParams {
    private Mat cameraMatrix;
    private Mat distCoeffsMatrix;
    private double[] distortionArray;

     Mat getCamMatrix() {
        return cameraMatrix;
    }

     Mat getDistortionMat() {
        return distCoeffsMatrix;
    }
     double[] getDistortionArray() {
        return distortionArray;
    }


     CamParams (double[] _cameraMatrix , double[]  _distCoeffs) {
        distortionArray = _distCoeffs;
        cameraMatrix  = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, _cameraMatrix);// (row, col,int[])
        distCoeffsMatrix =  new Mat(1, 5, CvType.CV_64FC1);
        distCoeffsMatrix.put(0, 0, _distCoeffs);
    }

}
