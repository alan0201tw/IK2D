#pragma once

#include <stdbool.h>

#include "bone2D.h"

void renderer2D_init();

bool renderer2D_should_close();

double renderer2D_start_frame();

void renderer2D_end_frame();

void renderer2D_clean_up();

//
void b2d_render(bone2D b2d);