#include "pattern_generation/PatternGeneration.h"

PatternGeneration::PatternGeneration(){
    dist=std::shared_ptr<std::uniform_int_distribution<int> > (new std::uniform_int_distribution<int> ());
}


cv::Scalar PatternGeneration::getRandomColor()
{
    std::random_device rd;
    std::mt19937 mt(rd());
    return cv::Scalar( (*dist)(mt)%255, (*dist)(mt)%255, (*dist)(mt)%255);
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

cv::Mat PatternGeneration::getPerlinNoiseTexture(const int & imageSize, const bool & random_colors, const double & z1,  const double & z2,  const double & z3)
{
    // Create an empty PPM image
    cv::Mat image(imageSize,imageSize,CV_8UC3,cv::Scalar::all(0));
    // Create a PerlinNoise object with a random permutation vector generated with seed
    //unsigned int seed = (*dist)(mt);
    PerlinNoise pn;

    std::random_device rd;
    std::mt19937 mt(rd());

    // Visit every pixel of the image and assign a color generated with Perlin noise
    for(int i = 0; i < imageSize; ++i) {     // y
            for(int j = 0; j < imageSize; ++j) {  // x
                double x = (double)j/((double)imageSize);
                double y = (double)i/((double)imageSize);

                cv::Vec3d val;

                // Wood like structure

                if(random_colors)
                {
                    val[0] = 20.0 * pn.noise(x, y, ((double) (*dist)(mt) / (RAND_MAX)));
                    val[0] = val[0] - floor(val[0]);
                    val[0] = floor(255 * val[0]);

                    val[1] = 20.0 * pn.noise(x, y, ((double) (*dist)(mt) / (RAND_MAX)));
                    val[1] = val[1] - floor(val[1]);
                    val[1] = floor(255 * val[1]);

                    val[2] = 20.0 * pn.noise(x, y, ((double) (*dist)(mt) / (RAND_MAX)));
                    val[2] = val[2] - floor(val[2]);
                    val[2] = floor(255 * val[2]);

                } else {

                    val[0] = 20.0 * pn.noise(x, y, z1);
                    val[0] = val[0] - floor(val[0]);
                    val[0] = floor(255 * val[0]);

                    val[1] = 20.0 * pn.noise(x, y, z2);
                    val[1] = val[1] - floor(val[1]);
                    val[1] = floor(255 * val[1]);

                    val[2] = 20.0 * pn.noise(x, y, z3);
                    val[2] = val[2] - floor(val[2]);
                    val[2] = floor(255 * val[2]);

                }

                //std::cout << val << std::endl;
                // Map the values to the [0, 255] interval, for simplicity we use
                // tones of grey
                image.at<cv::Vec3b>(i,j)=val;
            }
        }
    return image;
}
