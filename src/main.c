#include <stdio.h>
#include <stdlib.h>

#include "renderer2D.h"
#include "bone2D.h"
#include "ik2D.h"

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

int main(int argc, char** argv)
{
    renderer2D_init();

    bone2D tmp[5];
    for (int i = 0; i < 5; i++)
    {
        tmp[i].angle = 0.0f;
        tmp[i].length = 0.05f * (5 - i);
        if (i < 4)
            tmp[i].parent = &(tmp[i + 1]);
        else
            tmp[i].parent = NULL;
    }

    bone2D tmp_jaco[5];
    for (int i = 0; i < 5; i++)
    {
        tmp_jaco[i].angle = 0.0f;
        tmp_jaco[i].length = 0.05f * (5 - i);
        if (i < 4)
            tmp_jaco[i].parent = &(tmp_jaco[i + 1]);
        else
            tmp_jaco[i].parent = &(tmp[4]);
    }

    bone2D tmp_fabrik[5];
    for (int i = 0; i < 5; i++)
    {
        tmp_fabrik[i].angle = 0.0f;
        tmp_fabrik[i].length = 0.05f * (5 - i);
        if (i < 4)
            tmp_fabrik[i].parent = &(tmp_fabrik[i + 1]);
        else
            tmp_fabrik[i].parent = &(tmp[4]);
    }

    target2D target;
    target.x = 0.2f;
    target.y = -0.5f;

    int iteration = 0;

    while (!renderer2D_should_close())
    {
        float timeStep = (float)renderer2D_start_frame();

        t2d_update_position(&target, timeStep);
        t2d_render(&target);

        ik_status_code code0 = ik_ccd_single_iteration(&tmp[0], target);
        ik_status_code code1 = ik_inverse_jacobian_single_iteration(&tmp_jaco[0], target);
        ik_status_code code2 = ik_fabrik_single_iteration(&tmp_fabrik[0], target);

        b2d_render(&tmp[0], 0); // red-ish
        b2d_render(&tmp_fabrik[0], 1); // green-ish
        b2d_render(&tmp_jaco[0], 2); // blue-ish

        ++iteration;

        //tmp[4].angle += timeStep * 2.0f;

        renderer2D_end_frame();
    }

    renderer2D_clean_up();
}
