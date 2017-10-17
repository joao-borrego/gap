#include "pattern_generation/PatternGeneration.h"

cv::Scalar PatternGeneration::getRandomColor()
{
    return cv::Scalar( rand()%255, rand()%255, rand()%255);
}


cv::Mat PatternGeneration::getChessTexture(const cv::Scalar & color1, const cv::Scalar & color2, int blockSize, int squares)
{
    int imageSize=blockSize*squares;
    cv::Mat chessBoard(imageSize,imageSize,CV_8UC3,cv::Scalar::all(0));

    cv::Scalar color_;

    for(int i=0;i<imageSize;i=i+blockSize){
        for(int j=0;j<imageSize;j=j+blockSize){
            cv::Mat ROI=chessBoard(cv::Rect(i,j,blockSize,blockSize));
            if((i+j)%2==0) color_=color1;
            else       color_=color2;
            ROI.setTo(color_);
        }
    }

    return chessBoard;
}



cv::Mat PatternGeneration::getFlatTexture(const cv::Scalar & color, const int & imageSize)
{
    cv::Mat flat(imageSize,imageSize,CV_8UC3,color);

    return flat;
}

cv::Mat PatternGeneration::getGradientTexture(const cv::Scalar & color1, const cv::Scalar & color2, const int & imageSize, bool vertical)
{
    cv::Mat gradient(imageSize,imageSize,CV_8UC3,cv::Scalar::all(0));

    cv::Scalar gradient_step(color1-color2);

    if(vertical)
    {
        for(int y = 0; y < imageSize; y++)
        {
            cv::Vec3b val;

            val[0] = color1[0]-y*gradient_step[0]/imageSize; val[1] = color1[1]-y*gradient_step[1]/imageSize; val[2] =color1[2]-y*gradient_step[2]/imageSize;
            for(int x = 0; x < imageSize; x++)
                gradient.at<cv::Vec3b>(y,x) = val;
        }
    }
    else
    {
        for(int x = 0; x < imageSize; x++)
        {
            cv::Vec3b val;

            val[0] = color1[0]-x*gradient_step[0]/imageSize; val[1] = color1[1]-x*gradient_step[1]/imageSize; val[2] =color1[2]-x*gradient_step[2]/imageSize;
            for(int y = 0; y < imageSize; y++)
                gradient.at<cv::Vec3b>(y,x) = val;
        }
    }
    return gradient;
}
