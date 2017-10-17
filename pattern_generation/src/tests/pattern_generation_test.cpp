#include <sstream>
#include <string>

#include "pattern_generation/PatternGeneration.h"

/* File streams */
#include <fstream>

/* Media output directory */
#define MEDIA_DIR "../../media/materials/"
/* Textures subdir */
#define TEXTURES_DIR MEDIA_DIR "textures/"
/* Scripts subdir */
#define SCRIPTS_DIR MEDIA_DIR "scripts/"

/* Material name prefix */
#define MATERIAL_PREFIX "Plugin/"
/* Material name extension */
#define MATERIAL_EXT ".material"

/* Show image GUI */
#define SHOW_IMGS false
/* Number of images per class of texture */
#define IMGS_PER_CLASS 100
/* Generate .material script */
#define GENERATE_SCRIPT true

void genScript(const std::string material_name){

    std::stringstream script_str;
    std::stringstream file_path;

    if (GENERATE_SCRIPT){
        
        script_str
        << "material " << MATERIAL_PREFIX << material_name
        << std::endl << "{"
        << std::endl << "  technique"
        << std::endl << "  {"
        << std::endl << "    pass"
        << std::endl << "    {"
        << std::endl << "      texture_unit"
        << std::endl << "      {"
        << std::endl << "        texture " << material_name << ".jpg"
        << std::endl << "        filtering anistropic"
        << std::endl << "        max_anisotropy 16" 
        << std::endl << "      }"
        << std::endl << "    }"
        << std::endl << "  }"
        << std::endl <<" }";

        file_path << SCRIPTS_DIR << material_name << MATERIAL_EXT; 

        /* Write to file */
        std::ofstream ofs(file_path.str());
        ofs << script_str.str();
        ofs.close();
    }
}

int main()
{
    /* Initialize random seed */
    srand(time(NULL));
    
    /* Pattern generator object instance */   
    PatternGeneration pattern_generation;

    int img_size = 500;
    int squares;
    int block_size;

    std::stringstream material_name;
    std::stringstream img_filename;

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
        
        /* Reset string stream values */
        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "chess_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), chess_board)){
            std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str());
        
        if (SHOW_IMGS)
            cv::imshow("Chess board", chess_board);


        /* Generate flat texture */

        cv::Mat flat_texture = pattern_generation.getFlatTexture(color3,img_size);
        
        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "flat_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), flat_texture)){
            std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str());
        
        if (SHOW_IMGS)
            cv::imshow("Flat texture", flat_texture);

        /* Generate gradient texture */

        cv::Mat gradient_texture = pattern_generation.getGradientTexture(color1, color2, img_size, false);
                        
        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "gradient_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), gradient_texture)){
        std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl; 
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str());

        if (SHOW_IMGS)
            cv::imshow("Gradient texture", gradient_texture);
    }

    return 0;
}
