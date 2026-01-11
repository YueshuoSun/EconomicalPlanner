#ifndef MAP_PARAM_H
#define MAP_PARAM_H

struct MapParam {
    double map_radius = 0.65;
    double resolution = 0.02;
    double x_first_straight = 0.45;
    double x_second_straight = 0.05;
    double y_straight = 0.1;
    double curve_length = 1.105;
};

#endif