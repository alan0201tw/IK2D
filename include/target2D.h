#pragma once

#include "linmath.h"

typedef struct target2D
{
    union
    {
        vec2 position;
        struct
        {
            float x, y;
        };
    };

} target2D;
