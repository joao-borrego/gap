#include <sstream>
#include <string>

#include "pattern_generation/PatternGeneration.h"

/* Image output directory */
#define OUTPUT_DIR "../../media/model/materials/textures"
/* Show image GUI */
#define SHOW_IMGS false
/* Number of images per class of texture */
#define IMGS_PER_CLASS 1000


int main()
{
    PatternGeneration pattern_generation;
    
    /* Initialize random seed */
    srand(time(NULL));
    
    int img_size = 500;
    int squares;
    int block_size;

    for (int i = 0; i < IMGS_PER_CLASS; ++i)
    {
        /* Generate square texture */
        
        squares = rand() % 20 + 8;     // v2 in the range 1 to 100
        block_size = img_size / squares;
        if (block_size % 2 == 0)
            block_size++;

        cv::Scalar color1 = cv::Scalar(rand()%255, rand()%255, rand()%255);
        cv::Scalar color2 = cv::Scalar(rand()%255, rand()%255, rand()%255);
        cv::Scalar color3 = cv::Scalar(rand()%255, rand()%255, rand()%255);

        cv::Mat chess_board = pattern_generation.getChessTexture(color1, color2, block_size, squares);
        std::stringstream img_filename;
        img_filename << OUTPUT_DIR << "/chess_" << std::to_string(i) << ".jpg";
        if (!cv::imwrite(img_filename.str(), chess_board)){
            std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }
        
        if (SHOW_IMGS)
            cv::imshow("Chess board", chess_board);

        /* Generate flat texture */

        cv::Mat flat_texture = pattern_generation.getFlatTexture(color3,img_size);
        
        img_filename.str(std::string());
        img_filename << OUTPUT_DIR << "/flat_" << std::to_string(i) << ".jpg";
        if (!cv::imwrite(img_filename.str(), flat_texture)){
            std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }

        if (SHOW_IMGS)
            cv::imshow("Flat texture", flat_texture);

        /* Generate gradient texture */

        cv::Mat gradient_texture = pattern_generation.getGradientTexture(color1, color2, img_size, false);
        
        img_filename.str(std::string());
        img_filename << OUTPUT_DIR << "/gradient_" << std::to_string(i) << ".jpg";
        if (!cv::imwrite(img_filename.str(), gradient_texture)){
        std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }

        if (SHOW_IMGS)
            cv::imshow("Gradient texture", gradient_texture);
    }

    return 0;
}
