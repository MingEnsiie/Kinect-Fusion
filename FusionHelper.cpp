
#include "FusionHelper.h"
#define _USE_MATH_DEFINES




/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

HRESULT WriteBinarySTLMeshFile(INuiFusionMesh *mesh, char *FileName, bool flipYZ)
{
	HRESULT hr = S_OK;
	if (NULL == mesh)
	{
		return E_INVALIDARG;
	}

	unsigned int numVertices = mesh->VertexCount();
	unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
	unsigned int numTriangles = numVertices / 3;

	if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
	{
		return E_INVALIDARG;
	}

	const Vector3 *vertices = NULL;
	hr = mesh->GetVertices(&vertices);
	if (FAILED(hr))
	{
		return hr;
	}
	

	const Vector3 *normals = NULL;
	hr = mesh->GetNormals(&normals);
	if (FAILED(hr))
	{
		return hr;
	}

	const int *triangleIndices = NULL;
	hr = mesh->GetTriangleIndices(&triangleIndices);
	if (FAILED(hr))
	{
		return hr;
	}

	// Open File
	char* pszFileName = NULL;

	pszFileName = FileName;

	FILE *meshFile = NULL;
	errno_t err = fopen_s(&meshFile, pszFileName, "wb");

	// Could not open file for writing - return
	if (0 != err || NULL == meshFile)
	{
		return E_ACCESSDENIED;
	}

	// Write the header line
	const unsigned char header[80] = { 0 };   // initialize all values to 0
	fwrite(header, sizeof(unsigned char), ARRAYSIZE(header), meshFile);

	// Write number of triangles
	fwrite(&numTriangles, sizeof(int), 1, meshFile);
	int i = mesh->NormalCount();
	// Sequentially write the normal, 3 vertices of the triangle and attribute, for each triangle
	for (unsigned int t = 0; t < numTriangles; ++t)
	{
		Vector3 normal = normals[t * 3];

		if (flipYZ)
		{
			normal.y = -normal.y;
			normal.z = -normal.z;
		}

		// Write normal
		fwrite(&normal, sizeof(float), 3, meshFile);

		// Write vertices
		for (unsigned int v = 0; v < 3; v++)
		{
			Vector3 vertex = vertices[(t * 3) + v];

			if (flipYZ)
			{
				vertex.y = -vertex.y;
				vertex.z = -vertex.z;
			}

			fwrite(&vertex, sizeof(float), 3, meshFile);
		}

		unsigned short attribute = 0;
		fwrite(&attribute, sizeof(unsigned short), 1, meshFile);
	}

	fflush(meshFile);
	fclose(meshFile);

    //show the information of the mesh
	std::string str(FileName);
	cout << "Mesh: " << str << endl;
	cout << "Vertices: "<< numVertices << endl;
	cout << "Face: "<< numTriangles << endl;

	return hr;
}



/// <summary>
/// Write ASCII Wavefront .OBJ file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiObjMeshFile(INuiFusionMesh *mesh, char* FileName, bool flipYZ)
{
	HRESULT hr = S_OK;

	if (NULL == mesh)
	{
		return E_INVALIDARG;
	}

	unsigned int numVertices = mesh->VertexCount();
	unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
	unsigned int numTriangles = numVertices / 3;

	if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
	{
		return E_INVALIDARG;
	}

	const Vector3 *vertices = NULL;
	hr = mesh->GetVertices(&vertices);
	if (FAILED(hr))
	{
		return hr;
	}

	const Vector3 *normals = NULL;
	hr = mesh->GetNormals(&normals);
	if (FAILED(hr))
	{
		return hr;
	}

	const int *triangleIndices = NULL;
	hr = mesh->GetTriangleIndices(&triangleIndices);
	if (FAILED(hr))
	{
		return hr;
	}

	

	FILE *meshFile = NULL;
	errno_t err = fopen_s(&meshFile, FileName, "wt");

	// Could not open file for writing - return
	if (0 != err || NULL == meshFile)
	{
		return E_ACCESSDENIED;
	}

	// Write the header line
	std::string header = "#\n# OBJ file created by Microsoft Kinect Fusion\n#\n";
	fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

	const unsigned int bufSize = MAX_PATH * 3;
	char outStr[bufSize];
	int written = 0;

	if (flipYZ)
	{
		// Sequentially write the 3 vertices of the triangle, for each triangle
		for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
		{
			written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n",
				vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z,
				vertices[vertexIndex + 1].x, -vertices[vertexIndex + 1].y, -vertices[vertexIndex + 1].z,
				vertices[vertexIndex + 2].x, -vertices[vertexIndex + 2].y, -vertices[vertexIndex + 2].z);
			fwrite(outStr, sizeof(char), written, meshFile);
		}

		//// Sequentially write the 3 normals of the triangle, for each triangle
		//for (unsigned int t = 0, normalIndex = 0; t < numTriangles; ++t, normalIndex += 3)
		//{
		//	written = sprintf_s(outStr, bufSize, "vn %f %f %f\nvn %f %f %f\nvn %f %f %f\n",
		//		normals[normalIndex].x, -normals[normalIndex].y, -normals[normalIndex].z,
		//		normals[normalIndex + 1].x, -normals[normalIndex + 1].y, -normals[normalIndex + 1].z,
		//		normals[normalIndex + 2].x, -normals[normalIndex + 2].y, -normals[normalIndex + 2].z);
		//	fwrite(outStr, sizeof(char), written, meshFile);
		//}
	}
	else
	{
		// Sequentially write the 3 vertices of the triangle, for each triangle
		for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
		{
			written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n",
				vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
				vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
				vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z);
			fwrite(outStr, sizeof(char), written, meshFile);
		}

		//// Sequentially write the 3 normals of the triangle, for each triangle
		//for (unsigned int t = 0, normalIndex = 0; t < numTriangles; ++t, normalIndex += 3)
		//{
		//	written = sprintf_s(outStr, bufSize, "vn %f %f %f\nvn %f %f %f\nvn %f %f %f\n",
		//		normals[normalIndex].x, normals[normalIndex].y, normals[normalIndex].z,
		//		normals[normalIndex + 1].x, normals[normalIndex + 1].y, normals[normalIndex + 1].z,
		//		normals[normalIndex + 2].x, normals[normalIndex + 2].y, normals[normalIndex + 2].z);
		//	fwrite(outStr, sizeof(char), written, meshFile);
		//}
	}

	// Sequentially write the 3 vertex indices of the triangle face, for each triangle
	// Note this is typically 1-indexed in an OBJ file when using absolute referencing!
	for (unsigned int t = 0, baseIndex = 1; t < numTriangles; ++t, baseIndex += 3) // Start at baseIndex=1 for the 1-based indexing.
	{
		written = sprintf_s(outStr, bufSize, "f %u %u %u\n",
			baseIndex,  baseIndex + 1,  baseIndex + 2);

		fwrite(outStr, sizeof(char), written, meshFile);
	}
	// Note: we do not have texcoords to store, if we did, we would put the index of the texcoords between the vertex and normal indices (i.e. between the two slashes //) in the string above
	fflush(meshFile);
	fclose(meshFile);

	//show the information of the mesh
	std::string str(FileName);
	cout << "Mesh: " << str << endl;
	cout << "Vertices: " << numVertices << endl;
	cout << "Face: " << numTriangles << endl;

	return hr;
}



//function to read stl or obj file
bool ReadModelFile(char * filename, KinectFusionMeshTypes MeshType)
{

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();


	if (MeshType == Obj)
	{
		vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
		reader->SetFileName(filename);
		reader->Update();
		mapper->SetInputConnection(reader->GetOutputPort());
	}

	else if (MeshType == Stl)
	{
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
			reader->SetFileName(filename);
		reader->Update();
		mapper->SetInputConnection(reader->GetOutputPort());
	}
	else
	{
		return false;
	}	

	vtkSmartPointer<vtkActor> actor =vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	vtkSmartPointer<vtkRenderer> renderer =vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(actor);
	renderer->SetBackground(.3, .6, .3); // Background color green

	renderWindow->Render();
	renderWindowInteractor->Start();
	cout << "Reading file successed " <<  endl;
	return true;

}
