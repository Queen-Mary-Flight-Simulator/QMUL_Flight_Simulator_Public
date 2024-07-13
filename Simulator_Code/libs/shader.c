/* +------------------------------+---------------------------------+
   | Module      : shader.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-09              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : GPU shader loader/linker library                 |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>

#include <GL/glew.h>

#include <SIM/shader.h>

GLuint LoadShaders(const char * vertex_file_path, const char * fragment_file_path)
{
    FILE         *FileStream;
    char *       VertexShaderCode;
    char *       FragmentShaderCode;
    char const * VertexSourcePointer;
    char const * FragmentSourcePointer;
    GLint        Result;
    int          InfoLogLength;
    unsigned int p;
    unsigned int i;
    
    // Create the shaders
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    FileStream = fopen(vertex_file_path, "rb");
    if (FileStream == NULL)
    {
        printf("Failed to open file %s\n", vertex_file_path);
        exit(1);
    }

    p = 0;
    while (1)
    {
        if (fgetc(FileStream) == EOF)
        {
            break;
        }
        p += 1;
    }
    
    VertexShaderCode = malloc(p + 1);
    if (VertexShaderCode == NULL)
    {
        printf("Failed to allocate memory for VertexShaderCode: %d bytes\n", p + 1);
        exit(1);
    }

    rewind(FileStream);
    
    for (i=0; i<p; i+=1)
    {
        VertexShaderCode[i] = (char) fgetc(FileStream);
    }
    VertexShaderCode[p] = '\0';
    
    fclose(FileStream);
    
    FileStream = fopen(fragment_file_path, "rb");
    if (FileStream == NULL)
    {
        printf("Failed to open file %s\n", fragment_file_path);
        exit(1);
    }

    p = 0;
    while (1)
    {
        if (fgetc(FileStream) == EOF)
        {
            break;
        }
        p += 1;
    }
    
    FragmentShaderCode = malloc(p + 1);
    if (FragmentShaderCode == NULL)
    {
        printf("Failed to allocate memory for FragmentShaderCode: %d bytes\n", p + 1);
        exit(1);
    }

    rewind(FileStream);

    for (i=0; i<p; i+=1)
    {
        FragmentShaderCode[i] = (char) fgetc(FileStream);
    }
    FragmentShaderCode[p] = '\0';
    
    fclose(FileStream);
    
    /* Compile Vertex Shader */
    VertexSourcePointer = VertexShaderCode;
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
    glCompileShader(VertexShaderID);

    /* Check Vertex Shader */
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
        char * VertexShaderErrorMessage = malloc(InfoLogLength+1);
        glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
        free(VertexShaderErrorMessage);
    }

    /* Compile Fragment Shader */
    FragmentSourcePointer = FragmentShaderCode;
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
    glCompileShader(FragmentShaderID);

    /* Check Fragment Shader */
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
        char * FragmentShaderErrorMessage = malloc(InfoLogLength+1);
        glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
        printf("%s\n", &FragmentShaderErrorMessage[0]);
        free(FragmentShaderErrorMessage);
    }

    /* Link the program */
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    /* Check the program linking */
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
    glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0)
    {
        char * ProgramErrorMessage = malloc(InfoLogLength+1);
        glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);
        free(ProgramErrorMessage);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);
    
    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    free(VertexShaderCode);
    free(FragmentShaderCode);
    
    return ProgramID;
}
