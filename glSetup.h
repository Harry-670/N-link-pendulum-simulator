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

//store array of vertices for each shape, the buffer objects, and the window object
class DataStore {
private:
	float **BOArrays;
	unsigned int index, n;
	GLuint *VAO, *VBO;
	GLFWwindow* window;

public:
	DataStore() : BOArrays{}, index{}, n{}, VAO{}, VBO{}, window{} {}

	void setBO(GLuint*& VBO, GLuint*& VAO);
	void setIndex(unsigned int newIndex) { index = newIndex; }
	void setN(unsigned int newN) { n = newN; }

	GLuint *getVBO() { return VBO; }
	GLuint *getVAO() { return VAO; }
	unsigned int getIndex() const { return index; }
	unsigned int getN() const { return n; }

	GLFWwindow* getWindow();
	//sets up the window, defines the dimensions of the window and then returns the pointer to that object
	GLFWwindow* glSetupWindow(GLuint windowX, GLuint windowY);
	//generates the VAO and the VBOs for each shape
	void genShapes(unsigned int n);
	//adds the added shape to the corresponding VBO and VAO, then it returns the index that it is stored in BOArrays
	unsigned int addShape(float*& shape, unsigned int size);
	//cleans up memory, deletes, VAOs, VBOs, BOArrays, destroys window, and terminates GLFW
	void glClean();
};