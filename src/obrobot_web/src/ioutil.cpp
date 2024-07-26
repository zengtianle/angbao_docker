/**
 * @file ioutil.cpp
 * @author Jiang Ming (jim@lotlab.org)
 * @brief Some wrapper for system function
 * @version 0.1
 * @date 2021-11-04
 * 
 * @copyright Copyright (c) 2021 ESAC
 * 
 */
#include "ob_web/ioutil.h"
#include "ob_web/base64.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>

bool ioutil::RemoveFile(std::string path)
{
    auto ret = remove(path.c_str());
    if (ret != 0)
    {
        std::cerr << "无法删除文件" << path << ", 错误信息为: " << std::endl;
        perror("Remove failed.");
        return false;
    }
    return true;
}

bool ioutil::RenameFile(std::string from, std::string to)
{
    auto ret = rename(from.c_str(), to.c_str());
    if (ret != 0)
    {
        std::cerr << "无法将文件从" << from << "重命名为" << to << ", 错误信息为: " << std::endl;
        perror("Rename failed.");
        return false;
    }
    return true;
}

bool ioutil::CopyFile(std::string from, std::string to)
{
    try
    {
        std::ifstream input(from, std::ios::binary);
        std::ofstream output(to, std::ios::binary);

        output << input.rdbuf();
        input.close();
        output.close();

        return true;
    }
    catch (std::exception e)
    {
        std::cerr << "无法将文件从" << from << "复制到" << to << ", " << e.what() << std::endl;
        return false;
    }
}

pid_t ioutil::ExecWrapper(const char *file, char *const argv[])
{
    pid_t pid = fork();
    if (pid == 0)
    {
        execvp(file, argv);

        std::cerr << "程序运行失败。 Command: " << file << ", args:";
        while (*argv)
            std::cerr << " " << *argv;
        std::cerr << std::endl;

        perror("Exec failed.");
        exit(-1);
    }
    return pid;
}

pid_t ioutil::Exec(std::string file, std::string arg0)
{
    const char *argv[] = {arg0.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

pid_t ioutil::Exec(std::string file, std::string arg0, std::string arg1)
{
    const char *argv[] = {arg0.c_str(), arg1.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

pid_t ioutil::Exec(std::string file, std::string arg0, std::string arg1, std::string arg2)
{
    const char *argv[] = {arg0.c_str(), arg1.c_str(), arg2.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

pid_t ioutil::Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3)
{
    const char *argv[] = {arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

pid_t ioutil::Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3, std::string arg4)
{
    const char *argv[] = {arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), arg4.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

pid_t ioutil::Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3, std::string arg4, std::string arg5)
{
    const char *argv[] = {arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), arg4.c_str(), arg5.c_str(), NULL};
    return ExecWrapper(file.c_str(), (char *const *)argv);
}

bool ioutil::FileExists(std::string file)
{
    return access(file.c_str(), F_OK) == 0;
}

std::string ioutil::ReadFile(std::string file)
{
    std::ifstream fout(file);
    if (!fout)
    {
        std::cerr << "无法打开需要读取的文件：" << file << std::endl;
        return NULL;
    }

    std::ostringstream tmp;
    tmp << fout.rdbuf();
    std::string str = tmp.str();
    std::cout << str;

    return str;
}

bool ioutil::WriteFile(std::string file, std::string content)
{
    std::ofstream fout(file);
    if (!fout)
    {
        std::cerr << "无法打开需要写入的文件：" << file << std::endl;
        return false;
    }
    fout.clear();
    fout << content;
    return true;
}

bool ioutil::WriteBase64File(std::string file, const std::string &base64)
{
    std::ofstream fout(file);
    if (!fout)
    {
        std::cerr << "无法打开需要写入的文件：" << file << std::endl;
        return false;
    }
    fout.clear();
    base64_decode(base64, fout);
    fout.close();
    return true;
}
