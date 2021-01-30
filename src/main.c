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

    bone2D tmp[15];
    for(int i = 0; i < 15; i++)
    {
        tmp[i].angle = 0.0f;
        // tmp[i].angle = (6.28f / 15.0f);
        // tmp[i].length = 0.2f;
        // tmp[i].length = 0.05f * (5-i);
        tmp[i].length = 0.02f * (i+1);
        if(i < 14)
            tmp[i].parent = &(tmp[i+1]);
        else
            tmp[i].parent = NULL;
    }

    target2D target;
    target.x = 0.2f;
    target.y = -0.5f;

    int iteration = 0;
    const float vel = 0.5f;

	while (!renderer2D_should_close())
	{
		float timeStep = (float)renderer2D_start_frame();

        if(glfwGetKey(renderer2D_window(), GLFW_KEY_W) == GLFW_PRESS)
			target.y += timeStep * vel;
		else if(glfwGetKey(renderer2D_window(), GLFW_KEY_S) == GLFW_PRESS)
			target.y -= timeStep * vel;
		if(glfwGetKey(renderer2D_window(), GLFW_KEY_A) == GLFW_PRESS)
			target.x -= timeStep * vel;
		else if(glfwGetKey(renderer2D_window(), GLFW_KEY_D) == GLFW_PRESS)
			target.x += timeStep * vel;
        
        b2d_render(&tmp[0]);
        t2d_render(&target);

        ik_status_code code = ik_ccd_single_iteration(&tmp[0], target);
        ++iteration;

		renderer2D_end_frame();
	}

    renderer2D_clean_up();
}
