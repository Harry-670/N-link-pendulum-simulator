#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>

// Reads an entire file into a string. Used for loading shader source
std::string readFile(const char* file);

// Compiles and links the vertex + fragment shaders. Caller owns the returned
// program and must release it with glDeleteProgram
unsigned int initShaders();

// Owns the GL state for every shape: one VAO/VBO pair per shape, the CPU-side
// vertex data, and the window (its lifetime is tied to the GL context)
class DataStore {
private:
	// One float array per shape, kept around so glClean can free them
	float** bufferObjectArrays;

	unsigned int index;   // next free slot, advanced by addShape
	unsigned int n;       // total slots reserved by genShapes

	// Parallel arrays of length n. VAO[i], VBO[i], bufferObjectArrays[i] all
	// describe the same shape
	GLuint* VAO, * VBO;

	GLFWwindow* window;

public:
	DataStore() : bufferObjectArrays{}, index{}, n{}, VAO{}, VBO{}, window{} {}

	void setBO(GLuint*& VBO, GLuint*& VAO);
	void setIndex(unsigned int newIndex) { index = newIndex; }
	void setN(unsigned int newN) { n = newN; }

	GLuint* getVBO() { return VBO; }
	GLuint* getVAO() { return VAO; }
	unsigned int getIndex() const { return index; }
	unsigned int getN() const { return n; }

	GLFWwindow* getWindow();

	// Initializes GLFW and GLAD, creates the window, and makes its context current
	GLFWwindow* glSetupWindow(GLuint windowX, GLuint windowY);

	// Reserves n slots and generates n VAOs and VBOs. Requires an active GL
	// context, so call after glSetupWindow and before addShape.
	void genShapes(unsigned int n);

	// Uploads `shape` into the next free slot and returns its index. `size` is
	// the vertex count. Takes ownership of `shape` — do not delete[] it
	unsigned int addShape(float* shape, unsigned int size);

	// Frees the vertex data, deletes all VAOs/VBOs, destroys the window, and
	// terminates GLFW. Must be called exactly once at shutdown
	void glClean();
};