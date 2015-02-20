
 
 /*** 
 * Implemented by : Suryansh Kumar
 * 
 * Kalman Filter Routine
 * 
 * Names of variable are taken from
 * Probabilistic Robotics Text Book by Sebastian Thrun, Wolfram Burgard, Dieter Fox.
 * 
 ***/

#ifndef KALMAN_H
#define KALMAN_H
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;


class kalman{
  
  public : 
  void init_kalman(Mat, Mat, Mat, Mat, Mat, Mat);
  
  void predict();
  
  void update(Mat, int);
  
  void set_p(Mat );
  
  void set_mean(Mat );
  
  Mat get_p();
  
  Mat get_mean();
 
  private:  
    Mat mu; Mat a;
    Mat c;  Mat r;
    Mat q;  Mat p;         
};




#endif
