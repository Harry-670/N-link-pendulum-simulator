#ifndef VERTEX_SHADER_H
#define VERTEX_SHADER_H

#include <fstream>
#include <sstream>

std::string readFile(const char* file);

unsigned int initShaders();

#endif