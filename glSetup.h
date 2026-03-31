#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include <vector>

std::string readFile(const char* file);

unsigned int initShaders();
//setup the window and viewport as well as initialize the version of OpenGL


class DataStore {

private:

	float **BOArrays;
	unsigned int index, n;
	GLuint *VAO, *VBO;
	GLFWwindow* window;

public:

	GLFWwindow* glSetupWindow(GLuint windowX, GLuint windowY);

	DataStore():index(0),VAO(nullptr),VBO(nullptr) {}

	void setBO(GLuint*& VBO, GLuint*& VAO);

	GLuint *getVBO() {return VBO;}

	GLuint *getVAO() {return VAO;}
	void sendToGPU(unsigned int n);
		
	void glClean();

	unsigned int addShape(float*& shape, unsigned int size);
};