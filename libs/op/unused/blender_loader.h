/*
 * BlenderLoadre.h
 *
 *  Created on: 5. 10. 2014
 *      Author: Robert Pěnička
 */

#ifndef BLENDER_LOADER_H_
#define BLENDER_LOADER_H_
#include <vector>
#include <iostream>
#include <fstream>
//#include <glm/glm.hpp>
#include <GL/glu.h>
//#include "CLog.h"
#include "crl/logging.h"
#include "mesh_object.h"
#include <string>
#include <sstream>

#define FILE_HEADER "# Blender v2.72 (sub 0) OBJ File: 'goalConfigurationCubes.blend'" \
					<<	std::endl << "# www.blender.org" << std::endl

class BlenderLoader {
public:
	BlenderLoader();
	virtual ~BlenderLoader();
	static std::vector<MeshObject*> loadObjectsFromFile(std::string filename);
	static int saveObjectsToFile(const char* filename,std::vector<MeshObject*>,bool addObjectPosition = true);
private:
	static void loadMaterialForObjectFromFile(std::vector<MeshObject*> meshObjects, std::string matLibFilename);
};

#endif /* BLENDER_LOADER_H_ */
