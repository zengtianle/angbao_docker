/**
 * @file pgm_map_util.cpp
 * @author Jiang Ming (jim@lotlab.org)
 * @brief Handle PGM format map for ros map server.
 * @version 0.1
 * @date 2021-11-04
 * 
 * @copyright Copyright ESAC (c) 2021
 * 
 */

#include "ob_web/pgm_map_util.h"
#include "ob_web/ioutil.h"
#include <SDL/SDL_image.h>
#include <string>
#include <yaml-cpp/yaml.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

std::string PgmUtil::Filename(std::string path)
{
    auto pos = path.find_last_of('/');
    if (pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
}

std::string PgmUtil::getYamlName(std::string path)
{
    return path + ".yaml";
}

std::string PgmUtil::getPgmName(std::string path)
{
    return path + ".pgm";
}

void PgmUtil::ModifyPgmYaml(std::string path)
{
    auto yamlPath = getYamlName(path);
    auto mapName = getPgmName(Filename(path));

    YAML::Node nodenew = YAML::LoadFile(yamlPath);
    nodenew["image"] = mapName;
    std::ofstream foutnew(yamlPath);
    foutnew << nodenew;
    foutnew.close();
}

void PgmUtil::Copy(std::string from, std::string to)
{
    ioutil::CopyFile(getYamlName(from), getYamlName(to));
    ioutil::CopyFile(getPgmName(from), getPgmName(to));
    ModifyPgmYaml(to);
}

void PgmUtil::Rename(std::string from, std::string to)
{
    ioutil::RenameFile(getYamlName(from), getYamlName(to));
    ioutil::RenameFile(getPgmName(from), getPgmName(to));
    ModifyPgmYaml(to);
}

void PgmUtil::Remove(std::string path)
{
    ioutil::RemoveFile(getYamlName(path));
    ioutil::RemoveFile(getPgmName(path));
}

bool PgmUtil::Exists(std::string path)
{
    return ioutil::FileExists(getYamlName(path));
}

nav_msgs::OccupancyGrid PgmUtil::ReadMap(std::string path, double occTh, double freeTh)
{
    nav_msgs::OccupancyGrid grid;
    auto pgmPath = getPgmName(path);
    auto yamlPath = getYamlName(path);

    SDL_Surface* img;

    // Load the image using SDL.  If we get NULL back, the image load failed.
    if (!(img = IMG_Load(pgmPath.c_str()))) {
        std::string errmsg = std::string("failed to open image file \"") + std::string(pgmPath.c_str()) + std::string("\": ") + IMG_GetError();
        throw std::runtime_error(errmsg);
    }

    YAML::Node nodeWrite = YAML::LoadFile(path + ".yaml");

    // Copy the image data into the map structure
    grid.info.width = img->w;
    grid.info.height = img->h;
    grid.header.frame_id = "grid";
    nodeWrite["resolution"] >> grid.info.resolution;
    nodeWrite["origin"][0] >> grid.info.origin.position.x;
    nodeWrite["origin"][1] >> grid.info.origin.position.y;
    nodeWrite["origin"][2] >> grid.info.origin.position.z;

    // Allocate space to hold the data
    grid.data.resize(grid.info.width * grid.info.height);

    // Get values that we'll need to iterate through the pixels
    int rowstride = img->pitch;
    int n_channels = img->format->BytesPerPixel;

    // NOTE: Trinary mode still overrides here to preserve existing behavior.
    // Alpha will be averaged in with color channels when using trinary mode.
    int avg_channels = n_channels;

    // Copy pixel data into the map structure
    unsigned char* pixels = (unsigned char*)(img->pixels);
    for (int j = 0; j < grid.info.height; j++) {
        for (int i = 0; i < grid.info.width; i++) {
            // Compute mean of RGB for this pixel
            unsigned char* p = pixels + j * rowstride + i * n_channels;
            int color_sum = 0;
            for (int k = 0; k < avg_channels; k++)
                color_sum += *(p + (k));
            int color_avg = color_sum / (double)avg_channels;

            int alpha = (n_channels == 1) ? 1 : *(p + n_channels - 1);

            // If negate is true, we consider blacker pixels free, and whiter
            // pixels occupied.  Otherwise, it's vice versa.
            double occ = (255 - color_avg) / 255.0;

            // Apply thresholds to RGB means to determine occupancy values for
            // map.  Note that we invert the graphics-ordering of the pixels to
            // produce a map with cell (0,0) in the lower-left corner.
            unsigned char value;
            if (occ > occTh) {
                value = +100;
            } else if (occ < freeTh) {
                value = 0;
            } else {
                value = -1;
            }

            grid.data[MAP_IDX(grid.info.width, i, grid.info.height - j - 1)] = value;
        }
    }

    SDL_FreeSurface(img);

    return grid;
}

bool PgmUtil::EditMap(std::string path, nav_msgs::OccupancyGridConstPtr msg)
{
    if (!Exists(path)) {
        std::cerr << "无法编辑地图，因为源文件不存在。" << path << std::endl;
        return false;
    }

    auto pgmPath = getPgmName(path);
    std::ifstream pgmfile(pgmPath, std::ios::binary);

    std::string line1, line2, line3, line4, line5;

    getline(pgmfile, line1); // Must be "P5"
    getline(pgmfile, line2); // Comment line, "# CREATOR: map_saver.cpp 0.010 m/pix"， useless
    getline(pgmfile, line3); // Width, height. "500 500"
    getline(pgmfile, line4); // Max value, "255"
    pgmfile.close();

    std::ofstream fout(pgmPath);
    fout.clear();

    fout << line1.c_str();
    fout << "\n";
    fout << line2.c_str();
    fout << "\n";
    fout << line3.c_str();
    fout << "\n";
    fout << line4.c_str();
    fout << "\n";

    //像素写入
    for (int y = msg->info.height - 1; y >= 0; y--) {
        for (int x = 0; x < msg->info.width; x++) {
            int index = y * msg->info.width + x;
            if (msg->data[index] == 100) {
                fout.put('\0');
            } else if (msg->data[index] == 0) {
                fout.put(char(254));
            } else {
                fout.put(char(205));
            }
        }
    }

    fout << "\n";
    fout.close();

    return true;
}
