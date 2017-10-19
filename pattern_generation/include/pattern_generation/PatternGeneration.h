#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "pattern_generation/PerlinNoise.h"
#define RGB 0
#define HSV 1
#define HSL 2


class PatternGeneration
{
	public:
                PatternGeneration()
                {};


                cv::Mat getChessTexture(const cv::Scalar & color1, const cv::Scalar & color2, int blockSize=75, int squares=8);
                cv::Mat getFlatTexture(const cv::Scalar & color, const int & imageSize);
                cv::Mat getGradientTexture(const cv::Scalar & color1, const cv::Scalar & color2, const int & imageSize, bool vertical=true);
                cv::Mat getPerlinNoiseTexture(const int & imageSize, const bool & random_colors=true, const double & z1=0.8,  const double & z2=0.8,  const double & z3=0.8);

                cv::Scalar getRandomColor();

};
