#pragma once

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

class ioutil {
public:
    /**
     * @brief 删除一个文件
     * 
     * @param path 
     * @return true 
     * @return false 
     */
    static bool RemoveFile(std::string path);
    /**
     * @brief 重命名一个文件
     * 
     * @param from 
     * @param to 
     * @return true 
     * @return false 
     */
    static bool RenameFile(std::string from, std::string to);
    /**
     * @brief 复制一个文件
     * 
     * @param from 
     * @param to 
     * @return true 
     * @return false 
     */
    static bool CopyFile(std::string from, std::string to);
    /**
     * @brief 执行指定指令
     * 
     * @param file 
     * @param argv 
     * @return pid_t 
     */
    static pid_t ExecWrapper(const char* file, char* const argv[]);
    static pid_t Exec(std::string file, std::string arg0);
    static pid_t Exec(std::string file, std::string arg0, std::string arg1);
    static pid_t Exec(std::string file, std::string arg0, std::string arg1, std::string arg2);
    static pid_t Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3);
    static pid_t Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3, std::string arg4);
    static pid_t Exec(std::string file, std::string arg0, std::string arg1, std::string arg2, std::string arg3, std::string arg4, std::string arg5);

    /**
     * @brief 判断文件是否存在
     * 
     * @param path 
     * @return true 
     * @return false 
     */
    static bool FileExists(std::string path);

    /**
     * @brief 读取指定的文件
     * 
     * @param path 
     * @return std::string 
     */
    static std::string ReadFile(std::string path);
    
    /**
     * @brief 写内容到指定的文件
     * 
     * @param path 
     * @param content 
     * @return true 
     * @return false 
     */
    static bool WriteFile(std::string path, std::string content);

    /**
     * @brief 写内容到指定的文件
     * 
     * @param path 
     * @param content 
     * @return true 
     * @return false 
     */
    static bool WriteBase64File(std::string path, const std::string& content);
};
