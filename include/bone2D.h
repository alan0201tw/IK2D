#pragma once

typedef struct bone2D
{
    float x, y, angle;
    float length;
    struct bone2D* parent;
} bone2D;
