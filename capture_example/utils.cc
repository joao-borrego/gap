#include "utils.hh"

const std::string getUsage(const char* argv_0){
    return \
        "usage:   " + std::string(argv_0) + " [options]\n" +
        "options: -s number of scenes to generate\n"  +
        "         -i index of the first scene\n" +
        "         -m media directory\n" +
        "         -d output dataset directory\n";
}

void parseArgs(
    int argc,
    char** argv,
    unsigned int &scenes,
    unsigned int &start,
    std::string &media_dir,
    std::string &dataset_dir){

    int opt;
    bool s, i, m, d;

    while ((opt = getopt(argc,argv,"s: i: m: d:")) != EOF)
    {
        switch (opt)
        {
            case 's':
                s = true; scenes = atoi(optarg); break;
            case 'i':
                i = true; start = atoi(optarg); break;
            case 'm': 
                m = true; media_dir = optarg; break;
            case 'd':
                d = true; dataset_dir = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl; abort();
        }
    }

    // If arg was not set then assign default values
    if (!s) scenes = ARG_SCENES_DEFAULT;
    if (!i) start = ARG_START_DEFAULT;
    if (!m) media_dir = ARG_MEDIA_DIR_DEFAULT;
    if (!d) dataset_dir = ARG_DATASET_DIR_DEFAULT;

    debugPrint("scenes: '" << scenes <<
        "'; media dir: '" << media_dir <<
        "'; dataset dir: '" << dataset_dir << "'" << std::endl);
}

bool createDirectory(std::string &path){

    boost::filesystem::path dir(path);
    if (boost::filesystem::create_directory(dir)){
        debugPrint("Created directory " << path);
        return true;
    }
    return false;
}

void getFilenamesInDir(std::string &path, std::vector<std::string> &filenames){

    try{
        for (auto &p : boost::filesystem::directory_iterator(path)){
            std::string aux(boost::filesystem::basename(p));
            filenames.push_back(aux.c_str());
        }
    } catch(const boost::filesystem::filesystem_error& e) {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
}

// Random integer generator
std::random_device rng;
std::mt19937 mt_rng(rng());
std::uniform_int_distribution<int> uniform_dist;

int getRandomInt(int min, int max){
    int aux = uniform_dist(mt_rng);
    return aux % (max - min + 1) + min;;
}

double getRandomDouble(double min, double max){
    double aux = ((double) uniform_dist(mt_rng)) / (double) RAND_MAX;
    return aux * (max - min) + min;
}

void shuffleIntVector(std::vector<int> & vector){
    std::shuffle(vector.begin(), vector.end(), mt_rng);
}