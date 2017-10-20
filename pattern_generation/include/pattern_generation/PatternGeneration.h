#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <random>
#include "pattern_generation/PerlinNoise.h"
#include <memory>
#define RGB 0
#define HSV 1
#define HSL 2

class PatternGeneration
{
	private:
                std::shared_ptr<std::uniform_int_distribution<int> > dist;

	public:
        
        /**
         * @brief      Constructor
         */
	    PatternGeneration();

	    /**
	     * @brief      Generates a seed for the rng
	     */
	    void seedRandom();

	    /**
	     * @brief      Gets the random color.
	     *
	     * @return     The random color.
	     */
	    cv::Scalar getRandomColor();

	    /**
	     * @brief      Gets a chess texture.
	     *
	     * @param      color1     The color 1
	     * @param      color2     The color 2
	     * @param      blockSize  The block size
	     * @param      squares    The number of squares
	     *
	     * @return     The chess texture.
	     */
        cv::Mat getChessTexture(
        	const cv::Scalar & color1,
        	const cv::Scalar & color2,
        	int blockSize=75,
        	int squares=8);
        
        /**
         * @brief      Gets a flat texture.
         *
         * @param      color      The color
         * @param      imageSize  The image size
         *
         * @return     The flat texture.
         */
        cv::Mat getFlatTexture(
        	const cv::Scalar & color,
        	const int & imageSize);
        
        /**
         * @brief      Gets a gradient texture.
         *
         * @param      color1     The color 1
         * @param      color2     The color 2
         * @param      imageSize  The image size
         * @param      vertical   The vertical
         *
         * @return     The gradient texture.
         */
        cv::Mat getGradientTexture(
        	const cv::Scalar & color1,
        	const cv::Scalar & color2,
        	const int & imageSize,
        	bool vertical=true);
        
        /**
         * @brief      Gets the perlin noise texture.
         *
         * @param      imageSize      The image size
         * @param      random_colors  The random colors
         * @param      z1             The z 1
         * @param      z2             The z 2
         * @param      z3             The z 3
         *
         * @return     The perlin noise texture.
         */
        cv::Mat getPerlinNoiseTexture(
        	const int & imageSize,
        	const bool & random_colors=true,
        	const double & z1=0.8,
        	const double & z2=0.8,
        	const double & z3=0.8);
};
