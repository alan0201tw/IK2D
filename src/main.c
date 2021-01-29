#include <stdio.h>
#include <stdlib.h>

#include "renderer2D.h"
#include "bone2D.h"

int main(int argc, char** argv)
{
    renderer2D_init();

    bone2D tmp[15];
    for(int i = 0; i < 15; i++)
    {
        tmp[i].x = 0;
        tmp[i].y = 0;
        tmp[i].angle = (6.28f / 15.0f);
        tmp[i].length = 0.1f;
        if(i < 14)
            tmp[i].parent = &(tmp[i+1]);
        else
            tmp[i].parent = NULL;
    }

	while (!renderer2D_should_close())
	{
		float timeStep = (float)renderer2D_start_frame();
        
        b2d_render(tmp[0]);

		renderer2D_end_frame();
	}

    renderer2D_clean_up();
}
