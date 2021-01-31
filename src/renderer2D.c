#include "renderer2D.h"

#include <stdio.h>
#include <stdlib.h>

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "linmath.h"

static GLFWwindow* window;
static GLuint program;
static GLuint vertex_array, vertex_buffer, index_buffer;

static GLint P_location = 0;
static GLint M_location = 0;
static GLint Color_location = 0;

static float b2d_color[3] = {0.8f, 0.3f, 0.2f};

static double previousTime = 0.0f;

static float vertices[4 * 3] =
{
     0.0f, -0.5f, 0.0f,
     1.0f, -0.5f, 0.0f,
     1.0f,  0.5f, 0.0f,
     0.0f,  0.5f, 0.0f
};

static const char* vertex_shader_text = 
    "#version 330 core\n"
"\n"
    "layout(location = 0) in vec3 a_Position;\n"
"\n"
    "uniform mat4 P;\n"
    "uniform mat4 M;\n"
"\n"
    "out vec3 v_Position;\n"
"\n"
    "void main()\n"
    "{\n"
    "    v_Position = a_Position;\n"
    "    gl_Position = P * M * vec4(a_Position, 1.0f);\n"
    "}\n"
;

static const char* fragment_shader_text = 
    "#version 330 core\n"
"\n"
    "layout(location = 0) out vec4 color;\n"
"\n"
	"uniform vec3 u_Color;"
"\n"
    "in vec3 v_Position;\n"
"\n"
    "void main()\n"
    "{\n"
    "    color = vec4(u_Color, 1.0f);\n"
    "}\n"
;

void renderer2D_init()
{
	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	
	window = glfwCreateWindow(
		512, 512, "Inverse Kinematics", NULL, NULL);
	if (!window)
	{
        fprintf(stderr, "Failed to create window!\n");
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	int status = gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
	if (!status)
	{
        fprintf(stderr, "Failed to initialize glad!\n");
	}

	glfwSwapInterval(1);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Initialize VAO, VBO, IBO
	glGenVertexArrays(1, &vertex_array);
	glBindVertexArray(vertex_array);

	glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 12, vertices, GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, NULL);

	glGenBuffers(1, &index_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

	unsigned int indices[6] = { 0, 1, 2, 3 };
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float) * 4, indices, GL_STATIC_DRAW);

	// Initialize shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, 0);
    glCompileShader(vertex_shader);

	GLint isCompiled = 0;
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &isCompiled);
	if (isCompiled == GL_FALSE)
	{
		// The maxLength includes the NULL character
		GLchar infoLog[2048];
		glGetShaderInfoLog(vertex_shader, 2048, NULL, &infoLog[0]);

        fprintf(stderr, "%s\n", infoLog);
	}
 
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, 0);
    glCompileShader(fragment_shader);

	isCompiled = 0;
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &isCompiled);
	if (isCompiled == GL_FALSE)
	{
		// The maxLength includes the NULL character
		GLchar infoLog[2048];
		glGetShaderInfoLog(fragment_shader, 2048, NULL, &infoLog[0]);

        fprintf(stderr, "%s\n", infoLog);
	}
 
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

	GLint isLinked = 0;
	glGetProgramiv(program, GL_LINK_STATUS, (int *)&isLinked);
	if (isLinked == GL_FALSE)
	{
		GLchar infoLog[2048];
		glGetShaderInfoLog(program, 2048, NULL, &infoLog[0]);

        fprintf(stderr, "%s\n", infoLog);

		// We don't need the program anymore.
		glDeleteProgram(program);
	}

	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);

	// initialize timer
	previousTime = glfwGetTime();

	P_location = glGetUniformLocation(program, "P");
	M_location = glGetUniformLocation(program, "M");
	Color_location = glGetUniformLocation(program, "u_Color");
}

bool renderer2D_should_close()
{
	return glfwWindowShouldClose(window);
}

GLFWwindow* renderer2D_window()
{
	return window;
}

double renderer2D_start_frame()
{
	double currentTime = glfwGetTime();
	double timeStep = currentTime - previousTime;
	previousTime = currentTime;

	glClearColor(0.1f, 0.1f, 0.1f, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	glUseProgram(program);

	mat4x4 projectionMatrix;
	mat4x4_ortho(projectionMatrix, -1.0f, 1.0f, -1.0f, 1.0f, 1.f, -1.f);
	glUniformMatrix4fv(P_location, 1, GL_FALSE, (const GLfloat*)projectionMatrix);

	mat4x4 modelMatrix;
	mat4x4_identity(modelMatrix);
	mat4x4_rotate_Z(modelMatrix, modelMatrix, 1.0f);
	mat4x4_scale_aniso(modelMatrix, modelMatrix, 0.5f, 0.02f, 1.0f);
	glUniformMatrix4fv(M_location, 1, GL_FALSE, (const GLfloat*)modelMatrix);

	return timeStep;
}

void renderer2D_end_frame()
{
	glfwSwapBuffers(window);
	glfwPollEvents();
}

void renderer2D_clean_up()
{
	glfwDestroyWindow(window);

	glfwTerminate();
	exit(EXIT_SUCCESS);
}

// Obsolete : a hack to by-pass c compiler to return array types from funtion
// typedef struct __internal_hack_mat4x4
// {
// 	mat4x4 matrix;
// } __internal_hack_mat4x4;

static mat4x4 accumulatingMatrix;
static void __internal_b2d_render(bone2D* b2d)
{
	// render parent bone first, which will multiply the accumulatingMatrix
	// with the parent's transformation matrix;
	if(b2d->parent != NULL)
		__internal_b2d_render(b2d->parent);

	// the length of each bone is b2d.length
	mat4x4 localAccumulatingMatrix;
	mat4x4_identity(localAccumulatingMatrix);
	// root bone should not have length, or should have zero length
	// in this case we just ignore it by leaving it out of 
	// the matrix multiplication chain
	mat4x4_rotate_Z(localAccumulatingMatrix, localAccumulatingMatrix, b2d->angle);
	mat4x4_mul(accumulatingMatrix, accumulatingMatrix, localAccumulatingMatrix);

	if(b2d->parent != NULL)
	{
		mat4x4 modelMatrix;
		mat4x4_identity(modelMatrix);
		// mat4x4_rotate_Z(modelMatrix, modelMatrix, b2d->angle);
		mat4x4_scale_aniso(modelMatrix, modelMatrix, b2d->length, 0.02f, 1.0f);
		mat4x4_mul(modelMatrix, accumulatingMatrix, modelMatrix);
		glUniformMatrix4fv(M_location, 1, GL_FALSE, (const GLfloat*)modelMatrix);
		glUniform3f(Color_location, b2d_color[0], b2d_color[1], b2d_color[2]);
		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(vertex_array);
	}

	mat4x4 tMat;
	mat4x4_identity(tMat);
	if(b2d->parent != NULL)
		mat4x4_translate(tMat, b2d->length, 0.0f, 0.0f);
	mat4x4_mul(accumulatingMatrix, accumulatingMatrix, tMat);
}

void b2d_render(bone2D* b2d)
{
	mat4x4_identity(accumulatingMatrix);

	__internal_b2d_render(b2d);
}

void t2d_render(target2D* t2d)
{
	mat4x4 modelMatrix;
	mat4x4_identity(modelMatrix);
	mat4x4_translate(modelMatrix, t2d->x, t2d->y, 0.0f);
	mat4x4_scale(modelMatrix, modelMatrix, 0.05f);
	// mat4x4_scale_aniso(modelMatrix, modelMatrix, 0.05f, 0.05f, 0.05f);

	glUniform3f(Color_location, 0.2f, 0.8f, 0.3f);

	glUniformMatrix4fv(M_location, 1, GL_FALSE, (const GLfloat*)modelMatrix);

	glBindVertexArray(vertex_array);
    glDrawArrays(GL_QUADS, 0, 4);
}

void render2D_render_point(float x, float y)
{
	mat4x4 modelMatrix;
	mat4x4_identity(modelMatrix);
	mat4x4_translate(modelMatrix, x, y, 0.0f);
	mat4x4_scale(modelMatrix, modelMatrix, 0.02f);
	// mat4x4_scale_aniso(modelMatrix, modelMatrix, 0.05f, 0.05f, 0.05f);

	glUniform3f(Color_location, 1.0f, 1.0f, 1.0f);

	glUniformMatrix4fv(M_location, 1, GL_FALSE, (const GLfloat*)modelMatrix);

	glBindVertexArray(vertex_array);
    glDrawArrays(GL_QUADS, 0, 4);
}