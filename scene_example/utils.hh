/* C libraries */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* C++ libraries */

/* I/O streams */
#include <iostream>
/* File system */
#include <boost/filesystem.hpp>

/* Custom debug utilities */
#include "debug.hh"

/* Definitions */

/* Default arg values */
#define ARG_SCENES_DEFAULT      10
#define ARG_START_DEFAULT       0
#define ARG_MEDIA_DIR_DEFAULT   "media"
#define ARG_IMGS_DIR_DEFAULT    "imgs"
#define ARG_DATASET_DIR_DEFAULT "dataset"
#define ARG_DEBUG_DEFAULT       false 

/* Function headers */


const std::string getUsage(const char* argv_0);

void parseArgs(
    int argc,
    char** argv,
    unsigned int &scenes,
    unsigned int &start,
    std::string &media_dir,
    std::string &imgs_dir,
    std::string &dataset_dir,
    bool &debug);

bool createDirectory(std::string &path);

void getFilenamesInDir(std::string &path, std::vector<std::string> &filenames);

int getRandomInt(int min, int max);

double getRandomDouble(double min, double max);

void shuffleIntVector(std::vector<int> & vector);