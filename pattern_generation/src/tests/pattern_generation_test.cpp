#include "pattern_generation/PatternGeneration.h"

int main()
{
    PatternGeneration pattern_generation;
    /* Initialize random seed */
    srand(time(NULL));
    for(int i=0; i<1000;++i)
    {
        int img_size= 500;
        int squares = rand() % 20 + 8;     // v2 in the range 1 to 100
        int blockSize=img_size/squares;
        if(blockSize%2==0) blockSize++;

        //int blockSize = rand() % 25 + 75;     // v2 in the range 1 to 100
        //std::cout << "blocksize: "<< blockSize << std::endl;
        //int blockSize=75;
        //int squares=8;
        cv::Scalar color1=cv::Scalar( rand()%255, rand()%255, rand()%255);
        cv::Scalar color2=cv::Scalar( rand()%255, rand()%255, rand()%255);
        cv::Scalar color3=cv::Scalar( rand()%255, rand()%255, rand()%255);

        cv::Mat chessBoard=pattern_generation.getChessTexture(color1,color2,blockSize,squares);
 	cv::imwrite("/home/rui/shapes/materials/textures/chess_"+std::to_string(i)+".jpg", chessBoard );
        
	cv::imshow("Chess board", chessBoard);
        //cv::waitKey(10);                                          // Wait for a keystroke in the window


        cv::Mat flatTexture=pattern_generation.getFlatTexture(color3,img_size);
 	cv::imwrite("/home/rui/shapes/materials/textures/flat_"+std::to_string(i)+".jpg", flatTexture );

        cv::imshow("Flat texture", flatTexture);
        //cv::waitKey(10);                                          // Wait for a keystroke in the window

        cv::Mat gradientTexture=pattern_generation.getGradientTexture(color1,color2,img_size,false);
 	cv::imwrite("/home/rui/shapes/materials/textures/gradient_"+std::to_string(i)+".jpg", gradientTexture );

        cv::imshow("Gradient texture", gradientTexture);
        //cv::waitKey(10);                                          // Wait for a keystroke in the window


    }

    return 0;
}
