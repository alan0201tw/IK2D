#pragma once

typedef struct bone2D
{
    float angle;
    float length;
    struct bone2D* parent;
} bone2D;
