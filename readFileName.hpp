#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string>  // 如果strcmp()函数报错，则使用<cstring>
#include <cstring>


class readFileName
{
private:
    std::string fileAddr;
    std::vector<std::string> filenames;
public:
    readFileName();
    readFileName(const std::string& addr):fileAddr(addr){};
    
    void GetFileNames();
    std::vector<std::string> fileNames();
};

