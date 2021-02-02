#pragma once

#include <stdbool.h>

#include "bone2D.h"
#include "target2D.h"

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

void renderer2D_init();

bool renderer2D_should_close();

GLFWwindow* renderer2D_window();

double renderer2D_start_frame();

void renderer2D_end_frame();

void renderer2D_clean_up();

//
void b2d_render(bone2D* b2d, int color_code);
void t2d_render(target2D* t2d);

void render2D_render_point(float x, float y);

//
void t2d_update_position(target2D* t2d, float timeStep);