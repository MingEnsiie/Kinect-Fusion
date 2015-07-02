#pragma once

#include <NuiKinectFusionApi.h>
#include "vtkImageRender.h"
#include <vector>
#include <stdio.h>
#include <string.h>


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

enum KinectFusionMeshTypes
{
	Stl = 0,
	Obj = 1
};


/// <summary>
/// Write Binary .STL mesh file
/// see http://en.wikipedia.org/wiki/STL_(file_format) for STL format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteBinarySTLMeshFile(INuiFusionMesh *mesh, char* FileName, bool flipYZ = true);

/// <summary>
/// Write ASCII Wavefront .OBJ mesh file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiObjMeshFile(INuiFusionMesh *mesh, char* FileName, bool flipYZ = true);



/// Set Identity in a Matrix4
void SetIdentityMatrix(Matrix4 &mat);


//read model File 
bool ReadModelFile(char * filename, KinectFusionMeshTypes MeshType);

