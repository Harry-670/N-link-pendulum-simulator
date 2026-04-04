#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include <vector>

//reads the shader files
std::string readFile(const char* file);

//setup the window and viewport as well as initialize the version of OpenGL
unsigned int initShaders();



class DataStore {

private:

	float **BOArrays;
	unsigned int index, n;
	GLuint *VAO, *VBO;
	GLFWwindow* window;

public:

	DataStore() :BOArrays{}, index{}, n{}, VAO{}, VBO{},window{} {}

	void setBO(GLuint*& VBO, GLuint*& VAO);

	GLuint *getVBO() {return VBO;}

	GLuint *getVAO() {return VAO;}

	//sets up the window, defines the dimensions of the widow and then returns the pointer to that object
	GLFWwindow* glSetupWindow(GLuint windowX, GLuint windowY);

	//generates the VAO and the VBOs for each shape
	void genShapes(unsigned int n);
	
	//cleans up memory, deletes, VAOs, VBOs, BOArrays, destroys window, and terminates GLFW
	void glClean();

	//adds the added shape to the corresponing VBO and VAO, then it returns the index that it is stored in BOArrays
	unsigned int addShape(float*& shape, unsigned int size);
};