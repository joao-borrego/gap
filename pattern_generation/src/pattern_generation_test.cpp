#include "pattern_generation/PatternGeneration.h"

cv::Mat PatternGeneration:PatternGeneration()
{


}




cv::Mat PatternGeneration::getChessPattern(const int & blockSize=75, const int & squares=8, const unsigned char & color=0)
{

    int imageSize=blockSize*squares;
    Mat chessBoard(imageSize,imageSize,CV_8UC3,Scalar::all(0));
    unsigned char color=0;

     for(int i=0;i<imageSize;i=i+blockSize){
      color=~color;
       for(int j=0;j<imageSize;j=j+blockSize){
       Mat ROI=chessBoard(Rect(i,j,blockSize,blockSize));
       ROI.setTo(Scalar::all(color));
       color=~color;
      }
      
     }
    imshow("Chess board", chessBoard);
}
