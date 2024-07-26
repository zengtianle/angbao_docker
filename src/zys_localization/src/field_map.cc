#include "zys_localization/field_map.h"
#include <iostream>

using std::cout;
using std::endl;


int LikelyHoodFieldMap::valueWithoutResolution(int const& value) const {
    return value * resolution;
}

int LikelyHoodFieldMap::applyResolution(int const& value) const {
    return value / resolution;
}

void LikelyHoodFieldMap::toPGM(std::string const& filename, double const& max_, double const& min_) const {
    cout << "Writing Likelihood Field Map to " << filename << "..." <<endl;
    FILE* out = fopen(filename.c_str(), "w");
    if (!out) { 
        std::cout << "Could not save likelihood field map." << endl;
        return;
    }
    int maxGray = 255/(max_-min_);
    fprintf(out, "P5\n%d %d\n%d\n", (int) max_x, (int) max_y, maxGray);
            std::cout<<"--------------------"<<endl;
    for (int y = max_y; y > 0; --y) {

        for (int x = 0; x < max_x; ++x) {
            int output = (getEntry(x, y-1)-min_) * maxGray;
            fputc(output, out);
        }
    }
    fclose(out);
    cout << "... Finished!" << endl;
}

