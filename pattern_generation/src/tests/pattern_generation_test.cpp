#include <sstream>
#include <string>

#include "pattern_generation/PatternGeneration.h"

/* File streams */
#include <fstream>

/* Media output directory */
#define MEDIA_DIR "../../media/materials/"
/* Textures subdir */
//#define TEXTURES_DIR MEDIA_DIR "textures/"
/* Scripts subdir */
//#define SCRIPTS_DIR MEDIA_DIR "scripts/"

/* Material name prefix */
#define MATERIAL_PREFIX "Plugin/"
/* Material name extension */
#define MATERIAL_EXT ".material"

/* Show image GUI */
#define SHOW_IMGS true

/* Generate .material script */
#define GENERATE_SCRIPT true

void genScript(const std::string material_name, const std::string & SCRIPTS_DIR){

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
        << std::endl << "}";

        file_path << SCRIPTS_DIR << material_name << MATERIAL_EXT; 

        /* Write to file */
        std::ofstream ofs(file_path.str());
        ofs << script_str.str();
        ofs.close();
    }
}

int main(int argc, char **argv)
{
    if(argc<3)
    {
        std::cout << "invalid number of arguments"<< std::endl;
        exit(-1);
    }

    /* Number of images per class of texture */
    int images_per_class = atoi(argv[1]);

    /* root directory */
    std::string media_dir = std::string(argv[2]);

    std::string TEXTURES_DIR=media_dir+"textures/";
    std::string SCRIPTS_DIR=media_dir+"scripts/";

    /* Initialize random device */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist;

    /* Pattern generator object instance */   
    PatternGeneration pattern_generation;

    int img_size = 500;
    int squares;
    int block_size;

    std::stringstream material_name;
    std::stringstream img_filename;

    for (int i = 0; i < images_per_class; ++i)
    {
        /* Generate square texture */

        squares = dist(mt) % 20 + 8;     // v2 in the range 1 to 100
        block_size = img_size / squares;
        if (block_size % 2 == 0)
            block_size++;

        /*if(HSV==domain_)
        {
            cv::applyColorMap(color, HSV, CV_HSV2RGB);

        }
        else if(HSL==domain_){
            cv::applyColorMap(color, HSV, CV_HLS2RGB);

        }*/

        cv::Scalar chess_color_1 = pattern_generation.getRandomColor();
        cv::Scalar chess_color_2 = pattern_generation.getRandomColor();
        cv::Scalar flat_color = pattern_generation.getRandomColor();
        cv::Scalar gradient_color_1 = pattern_generation.getRandomColor();
        cv::Scalar gradient_color_2 = pattern_generation.getRandomColor();

        cv::Mat chess_board = pattern_generation.getChessTexture(chess_color_1, chess_color_2, block_size, squares);

        // Convert to HSV
        cv::applyColorMap(chess_board, chess_board, cv::COLORMAP_HSV);


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
        genScript(material_name.str(), SCRIPTS_DIR);

        if (SHOW_IMGS)
            cv::imshow("Chess board", chess_board);


        /* Generate flat texture */

        cv::Mat flat_texture = pattern_generation.getFlatTexture(flat_color,img_size);

        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "flat_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), flat_texture)){
            std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl;
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str(), SCRIPTS_DIR);

        if (SHOW_IMGS)
            cv::imshow("Flat texture", flat_texture);

        /* Generate gradient texture */

        cv::Mat gradient_texture = pattern_generation.getGradientTexture(gradient_color_1, gradient_color_2, img_size, false);

        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "gradient_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), gradient_texture)){
        std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl;
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str(), SCRIPTS_DIR);

        if (SHOW_IMGS)
            cv::imshow("Gradient texture", gradient_texture);

        /* Generate perlin noise texture */
        double z1=((double) dist(mt) / (RAND_MAX));
        double z2=((double) dist(mt) / (RAND_MAX));
        double z3=((double) dist(mt) / (RAND_MAX));
        srand(time(0));
        int randomval = dist(mt) % 2;

        cv::Mat perlin_texture = pattern_generation.getPerlinNoiseTexture(img_size,randomval,z1,z2,z3);

        material_name.str(std::string());
        img_filename.str(std::string());

        material_name << "perlin_" << std::to_string(i);
        img_filename << TEXTURES_DIR << material_name.str() << ".jpg";

        if (!cv::imwrite(img_filename.str(), perlin_texture)){
        std::cout << "[ERROR] Could not save " << img_filename.str() <<
            ". Please ensure the destination folder exists!" << std::endl;
            exit(EXIT_FAILURE);
        }
        genScript(material_name.str(), SCRIPTS_DIR);

        if (SHOW_IMGS)
            cv::imshow("Perlin noise texture", perlin_texture);


    }

    return 0;
}
