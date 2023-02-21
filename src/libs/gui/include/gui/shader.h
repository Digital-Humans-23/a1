#pragma once

#include <glad/glad.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <glm/glm.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace crl {
namespace gui {

inline std::vector<std::string> split(const std::string &s, const char delim) {
    std::vector<std::string> v;
    auto p = std::begin(s);
    for (auto q = std::find(p, std::end(s), delim); q != std::end(s);
         q = std::find(++p, std::end(s), delim)) {
        v.emplace_back(p, q);
        p = q;
    }
    if (p != std::end(s)) v.emplace_back(p, std::end(s));
    return v;
}

inline bool fileExists(const std::string &filename) {
    return std::ifstream{filename}.good();
}

class Lines {
    std::istream &is;
    std::string cur;
    void read_one() { std::getline(is, cur); }

public:
    Lines(std::istream &is) : is{is} { read_one(); }

    class iterator {
    public:
        using value_type = std::string;
        using reference = std::string &;
        using pointer = std::string *;
        using const_reference = const std::string &;
        using const_pointer = const std::string *;
        using iterator_category = std::forward_iterator_tag;
        using difference_type = int;

    private:
        Lines *src;
        friend class Lines;
        iterator(Lines *src) noexcept : src{src} {}

    public:
        iterator() noexcept : src{} {}
        reference operator*() noexcept { return src->cur; }
        const_reference operator*() const noexcept { return src->cur; }
        bool operator==(const iterator &other) const noexcept {
            return (src == other.src) || (!src && !other.src->is) ||
                   (!src->is && !other.src);
        }
        bool operator!=(const iterator &other) const noexcept {
            return !(*this == other);
        }
        iterator &operator++() {
            src->read_one();
            return *this;
        }
        iterator operator++(int) {
            auto temp = *this;
            operator++();
            return temp;
        }
    };
    iterator begin() noexcept { return {this}; }
    iterator end() noexcept { return {}; }

private:
    friend class iterator;
};

class Shader {
public:
    unsigned int ID;
    // constructor generates the shader on the fly
    // ------------------------------------------------------------------------
    Shader(const char *vertexPath, const char *fragmentPath) {
        // 1. retrieve the vertex/fragment source code from filePath
        std::string vertexCode = shaderString(vertexPath);
        std::string fragmentCode = shaderString(fragmentPath);
        const char *vShaderCode = vertexCode.c_str();
        const char *fShaderCode = fragmentCode.c_str();
        // 2. compile shaders
        unsigned int vertex, fragment;
        // vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, nullptr);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        // fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, nullptr);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        // shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no
        // longer necessery
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }
    std::string shaderString(const char *shaderPath) {
        std::string shaderCode;
        std::ifstream shaderFile;
        // ensure ifstream objects can throw exceptions:
        shaderFile.exceptions(
            std::ifstream::badbit);  // std::ifstream::failbit fails at eof
        try {
            // open files
            shaderFile.open(shaderPath);
            std::stringstream shaderStream;
            // read file's buffer contents into streams
            for (std::string s; std::getline(shaderFile, s);) {
                auto token = split(s, ' ');
                if (token.size() >= 2 && token[0] == "#include") {
                    token[1].erase(
                        remove(token[1].begin(), token[1].end(), '\"'),
                        token[1].end());
                    std::string includePath =
                        CRL_SHADER_FOLDER + std::string("/") + token[1];
                    std::ifstream sourceFile(includePath);
                    s.assign((std::istreambuf_iterator<char>(sourceFile)),
                             std::istreambuf_iterator<char>());
                }
                shaderStream << s << std::endl;
            }
            // close file handlers
            shaderFile.close();
            // convert stream into string
            shaderCode = shaderStream.str();
        } catch (std::ifstream::failure e) {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ"
                      << std::endl;
        }
        return shaderCode;
    }
    // activate the shader
    // ------------------------------------------------------------------------
    void use() const { glUseProgram(ID); }
    // utility uniform functions
    // ------------------------------------------------------------------------
    void setBool(const std::string &name, bool value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
    }
    // ------------------------------------------------------------------------
    void setInt(const std::string &name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }
    // ------------------------------------------------------------------------
    void setFloat(const std::string &name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }
    // ------------------------------------------------------------------------
    void setVec2(const std::string &name, const glm::vec2 &value) const {
        glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec2(const std::string &name, float x, float y) const {
        glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
    }
    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec3(const std::string &name, float x, float y, float z) const {
        glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const {
        glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec4(const std::string &name, float x, float y, float z,
                 float w) const {
        glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }
    // ------------------------------------------------------------------------
    void setMat2(const std::string &name, const glm::mat2 &mat) const {
        glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE,
                           &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat3(const std::string &name, const glm::mat3 &mat) const {
        glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE,
                           &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat4(const std::string &name, const glm::mat4 &mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE,
                           &mat[0][0]);
    }

private:
    // utility function for checking shader compilation/linking errors.
    // ------------------------------------------------------------------------
    void checkCompileErrors(GLuint shader, std::string type) {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
                std::cout
                    << "ERROR::SHADER_COMPILATION_ERROR of type: " << type
                    << "\n"
                    << infoLog
                    << "\n -- "
                       "--------------------------------------------------- -- "
                    << std::endl;
            }
        } else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
                std::cout
                    << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                    << infoLog
                    << "\n -- "
                       "--------------------------------------------------- -- "
                    << std::endl;
            }
        }
    }
};

}  // namespace gui
}  // namespace crl