#include "readFileName.hpp"


void readFileName::GetFileNames()
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(fileAddr.c_str()))){
        std::cout<<"Folder doesn't Exist!"<<std::endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(fileAddr + "/" + ptr->d_name);
    }
    }
    closedir(pDir);
}

std::vector<std::string> readFileName::fileNames()
{
    return filenames;
}

