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

    const int bone_count = 10;
    const float bone_length = 0.1;

    bone2D tmp[bone_count];
    for (int i = 0; i < bone_count; i++)
    {
        tmp[i].angle = 0.0f;
        tmp[i].length = bone_length;
        if (i < bone_count - 1)
            tmp[i].parent = &(tmp[i + 1]);
        else
            tmp[i].parent = NULL;
    }

    bone2D tmp_jaco[bone_count];
    for (int i = 0; i < bone_count; i++)
    {
        tmp_jaco[i].angle = 0.0f;
        tmp_jaco[i].length = bone_length;
        if (i < bone_count - 1)
            tmp_jaco[i].parent = &(tmp_jaco[i + 1]);
        else
            tmp_jaco[i].parent = &(tmp[bone_count - 1]);
    }

    bone2D tmp_fabrik[bone_count];
    for (int i = 0; i < bone_count; i++)
    {
        tmp_fabrik[i].angle = 0.0f;
        tmp_fabrik[i].length = bone_length;
        if (i < bone_count - 1)
            tmp_fabrik[i].parent = &(tmp_fabrik[i + 1]);
        else
            tmp_fabrik[i].parent = &(tmp[bone_count - 1]);
    }

    target2D target;
    target.x = -0.3f;
    target.y = 0.3f;

    float timer = 0;

    while (!renderer2D_should_close())
    {
        float timeStep = (float)renderer2D_start_frame();
        // fprintf(stderr, "fps = %f \n", 1.0f / timeStep);
        // target.x = cosf(timer * 2.0f + 0.001f) * (0.1f + timer / 50.0f);
        // target.y = sinf(timer * 2.0f + 0.001f) * (0.1f + timer / 50.0f);

        t2d_update_position(&target, timeStep);
        t2d_render(&target);

        // ik_status_code code0 = ik_ccd_single_iteration(&tmp[0], target);
        // ik_status_code code1 = ik_inverse_jacobian_single_iteration(&tmp_jaco[0], target);
        // ik_status_code code2 = ik_fabrik_single_iteration(&tmp_fabrik[0], target);

        ik_status_code code0 = ik_ccd(&tmp[0], target);
        ik_status_code code1 = ik_inverse_jacobian(&tmp_jaco[0], target);
        ik_status_code code2 = ik_fabrik(&tmp_fabrik[0], target);

        b2d_render(&tmp[0], 0); // red-ish
        b2d_render(&tmp_fabrik[0], 1); // green-ish
        b2d_render(&tmp_jaco[0], 2); // blue-ish

        timer += timeStep;
        // fprintf(stderr, "timer = %f \n", timer);

        // for (int i = 0; i < bone_count; i++)
        //     fprintf(stderr, "inv_jaco : i = %d, angle = %f \n", i, tmp_jaco[i].angle);

        //tmp[4].angle += timeStep * 2.0f;
        
        renderer2D_end_frame();
    }

    renderer2D_clean_up();
}
