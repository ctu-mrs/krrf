#include "hdf5io.h"

#include <iostream>
#include <string.h>

#ifdef USE_SEMAPHORES
#define LOCK() sem_wait(&globalsem);
#define UNLOCK() sem_post(&globalsem);
#else
#define LOCK() 
#define UNLOCK()
#endif

using namespace std;

/** delete an attribute given by its name from object_id (datasetor group). The object_id has to be opened
  * before this function is called (e.g. using H5Dopen or H5Gopen)
  *
  * return 1 if attribute was deleted
  *        -1 if atrribute cannot be  deleted
  */
static int deleteAttribute(hid_t object_id, const char *attributeName) {
    herr_t r = H5Adelete(object_id, attributeName); 
    if (r >= 0) {
        return 1;
    }
    cerr << "Cannot delete attribute '" << attributeName << "'!\n";
    return -1;
}

/** here it is supposed, that attrib_id was opended before by H5Aopen */
static void writeAttributeInt(hid_t attrib_id, const std::vector<int> &data) {

    hid_t space = H5Aget_space(attrib_id);
    const int ndims = H5Sget_simple_extent_ndims(space);

    hsize_t *dimsize = new hsize_t[ndims];
    hsize_t *maxdims = new hsize_t[ndims];
    H5Sget_simple_extent_dims(space,dimsize,maxdims);
    H5Sclose(space);
    const int rows = dimsize[0];
    const int rowsize = maxdims[0];
    delete [] dimsize;
    delete [] maxdims;
    if ((int)data.size() != rowsize) {
        cerr << "Cannot write attribute, dimension of data and space for attribute are not same: attrib.size=" 
            << rowsize << ", data.size="<<data.size()<<"\n";
        return;
    }
    int *d = new int[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    herr_t status = H5Awrite(attrib_id,H5T_NATIVE_INT,d);
    delete [] d;
}

/** here it is supposed, that attrib_id was opended before by H5Aopen */
static void writeAttributeDouble(hid_t attrib_id, const std::vector<double> &data) {

    hid_t space = H5Aget_space(attrib_id);
    const int ndims = H5Sget_simple_extent_ndims(space);

    hsize_t *dimsize = new hsize_t[ndims];
    hsize_t *maxdims = new hsize_t[ndims];
    H5Sget_simple_extent_dims(space,dimsize,maxdims);
    H5Sclose(space);
    const int rows = dimsize[0];
    const int rowsize = maxdims[0];
    delete [] dimsize;
    delete [] maxdims;
    if ((int)data.size() != rowsize) {
        cerr << "Cannot write attribute, dimension of data and space for attribute are not same: attrib.size=" 
            << rowsize << ", data.size="<<data.size()<<"\n";
        return;
    }
    double *d = new double[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    herr_t status = H5Awrite(attrib_id,H5T_NATIVE_DOUBLE,d);
    delete [] d;
}


/** here it is supposed, that attrib_id was opended before by H5Aopen */
static void writeAttributeFloat(hid_t attrib_id, const std::vector<float> &data) {

    hid_t space = H5Aget_space(attrib_id);
    const int ndims = H5Sget_simple_extent_ndims(space);

    hsize_t *dimsize = new hsize_t[ndims];
    hsize_t *maxdims = new hsize_t[ndims];
    H5Sget_simple_extent_dims(space,dimsize,maxdims);
    H5Sclose(space);
    const int rows = dimsize[0];
    const int rowsize = maxdims[0];
    delete [] dimsize;
    delete [] maxdims;
    if ((int)data.size() != rowsize) {
        cerr << "Cannot write attribute, dimension of data and space for attribute are not same: attrib.size=" 
            << rowsize << ", data.size="<<data.size()<<"\n";
        return;
    }
    float *d = new float[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    herr_t status = H5Awrite(attrib_id,H5T_NATIVE_FLOAT,d);
    delete [] d;
}




static void createAndWriteTextAttribute(hid_t dataset, const char *attributeName, const char *value) {
        hsize_t len = strlen(value);
        hsize_t dims = 1;
        hid_t dataspace = H5Screate_simple(1,&dims,NULL);

        hid_t datatype = H5Tcopy(H5T_C_S1);
        herr_t status = H5Tset_size(datatype,len);
        
        hid_t filetype = H5Tcopy(H5T_C_S1);
        status = H5Tset_size(filetype,len);

        hid_t attribute = H5Acreate(dataset,attributeName,filetype,dataspace,H5P_DEFAULT,H5P_DEFAULT);
        status = H5Awrite(attribute,datatype,value);
        H5Aclose(attribute);
        H5Sclose(dataspace);
        H5Tclose(datatype);
        H5Tclose(filetype);
}

static void createAndWriteAttributeInt(hid_t id, const char *attributeName, const std::vector<int> &data) {
    int *d = new int[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    hsize_t adims = data.size();
    hid_t dataspacea_id = H5Screate_simple(1,&adims,NULL);
    hid_t attribute_id = H5Acreate(id,attributeName,H5T_STD_I32BE,dataspacea_id, H5P_DEFAULT,H5P_DEFAULT);
    herr_t status = H5Awrite(attribute_id,H5T_NATIVE_INT,d);
    status = H5Aclose(attribute_id);
    status = H5Sclose(dataspacea_id);
    delete [] d;
 
} 

static void createAndWriteAttributeDouble(hid_t id, const char *attributeName, const std::vector<double> &data) {
    double *d = new double[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    hsize_t adims = data.size();
    hid_t dataspacea_id = H5Screate_simple(1,&adims,NULL);
    hid_t attribute_id = H5Acreate(id,attributeName,H5T_IEEE_F64BE,dataspacea_id, H5P_DEFAULT,H5P_DEFAULT);
    herr_t status = H5Awrite(attribute_id,H5T_NATIVE_DOUBLE,d);
    status = H5Aclose(attribute_id);
    status = H5Sclose(dataspacea_id);
    delete [] d;
} 


static void createAndWriteAttributeFloat(hid_t id, const char *attributeName, const std::vector<float> &data) {
    float *d = new float[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    hsize_t adims = data.size();
    hid_t dataspacea_id = H5Screate_simple(1,&adims,NULL);
    hid_t attribute_id = H5Acreate(id,attributeName,H5T_IEEE_F32BE,dataspacea_id, H5P_DEFAULT,H5P_DEFAULT);
    herr_t status = H5Awrite(attribute_id,H5T_NATIVE_FLOAT,d);
    status = H5Aclose(attribute_id);
    status = H5Sclose(dataspacea_id);
    delete [] d;
} 


/** return number of columns in given dataset. We assume, taht dataset is open!
  */
static int _getNumCols(hid_t dataset_id) {
    hid_t dspace  = H5Dget_space(dataset_id);
    const int dimss = H5Sget_simple_extent_ndims(dspace);

    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(dspace,dimsize,maxdims);
    const int cols = dimsize[1];
    delete [] dimsize;
    delete [] maxdims;
    H5Sclose(dspace);
    return cols;
}

/** return number of rows in given dataset. We assume, that the dataset is open! */
static int _getNumRows(hid_t dataset_id) {
    hid_t dspace  = H5Dget_space(dataset_id);
    const int dimss = H5Sget_simple_extent_ndims(dspace);

    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(dspace,dimsize,maxdims);
    const int rows = dimsize[0];
    delete [] dimsize;
    delete [] maxdims;
    H5Sclose(dspace);
    return rows;
}




HDF::HDF() {
    file_id = -1;
#ifdef USE_SEMAPHORES
    sem_init(&globalsem,0,1);
#endif

}

HDF::~HDF() {
    LOCK();
    if (file_id >= 0) {
        H5Fclose(file_id);
    }
    UNLOCK();
}

bool HDF::isThreaded() const {
#ifdef USE_SEMAPHORES
    return true;
#else
    return false;
#endif
}


void HDF::createFile(const char *filename) {
    LOCK(); 
	file_id = H5Fcreate(filename,H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);
    UNLOCK();
}

void HDF::open(const char *filename) {
    LOCK();
	file_id = H5Fopen(filename,H5F_ACC_RDWR,H5P_DEFAULT);
    UNLOCK();
}


void HDF::openOrCreate(const char *filename) {
    LOCK();
	file_id = H5Fopen(filename,H5F_ACC_RDWR,H5P_DEFAULT);
	if (file_id < 0) {
		cerr << "Cannot open '" << filename << "', creating a new file\n";
		//createFile(filename);
	    file_id = H5Fcreate(filename,H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);
	}
    UNLOCK();
}

void HDF::close() {
    LOCK();
	if (file_id >= 0) {
        H5Fclose(file_id);
		file_id = -1;
	}
    UNLOCK();
}


void HDF::createGroup(const char *groupName) {
    LOCK();
    hid_t group_id = H5Gcreate(file_id,groupName,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    if (group_id < 0) {
        UNLOCK();
        cerr << "cannot create group '" << groupName << "'!\n";
        return;
    }
    H5Gclose(group_id);
    UNLOCK();
}


void HDF::createDatasetString(const char *name, const int rows, const int cols) {
    hid_t filetype = H5Tcopy(H5T_C_S1);
//    herr_t status = H5Tset_size(filetype,stringwidth);
    herr_t status = H5Tset_size(filetype,H5T_VARIABLE);

//    hid_t memtype = H5Tcopy(H5T_C_S1);
//	status = H5Tset_size(filetype,stringwidth);
//    status = H5Tset_size(filetype,H5T_VARIABLE);

    hsize_t dims[2];
    dims[0]	= rows;
	dims[1] = cols;
    hid_t space = H5Screate_simple(2,dims,NULL);

    LOCK();
    hid_t dset = H5Dcreate(file_id,name,filetype,space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    UNLOCK();
    if (dset < 0) {
        cerr << "Cannot create text data set '" << name << "'!\n";
        return;
    }

    H5Dclose(dset);
    H5Sclose(space);
 //   H5Tclose(memtype);
    H5Tclose(filetype);

}

void HDF::createDatasetDouble(const char *name, const int rows, const int cols) {
    LOCK();
    hsize_t dims[2];
    dims[0] = rows;
    dims[1] = cols;
    hid_t dataspace_id = H5Screate_simple(2,dims,NULL);
    hid_t dataset_id = H5Dcreate(file_id,name,H5T_IEEE_F64BE,dataspace_id,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot create dataset '" << name << "'!\n";
        H5Sclose(dataspace_id);
        UNLOCK();
        return;
    }
    H5Sclose(dataspace_id);
    H5Dclose(dataset_id); 
    UNLOCK();
}

void HDF::createDatasetFloat(const char *name, const int rows, const int cols) {
    LOCK()
    hsize_t dims[2];
    dims[0] = rows;
    dims[1] = cols;
    hid_t dataspace_id = H5Screate_simple(2,dims,NULL);
    hid_t dataset_id = H5Dcreate(file_id,name,H5T_IEEE_F32BE,dataspace_id,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot create dataset '" << name << "'!\n";
        H5Sclose(dataspace_id);
        UNLOCK();
        return;
    }
    H5Sclose(dataspace_id);
    H5Dclose(dataset_id); 
    UNLOCK();
}


void HDF::createDatasetInt(const char *name, const int rows, const int cols) {
    LOCK();
    hsize_t dims[2];
    dims[0] = rows;
    dims[1] = cols;
    hid_t dataspace_id = H5Screate_simple(2,dims,NULL);
    hid_t dataset_id = H5Dcreate(file_id,name,H5T_STD_I32BE,dataspace_id,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);

    if (dataset_id < 0) {
        cerr << "Cannot create dataset '" << name << "'!\n";
        H5Sclose(dataspace_id);
        UNLOCK();
        return;
    }
    H5Sclose(dataspace_id);
    H5Dclose(dataset_id); 
    UNLOCK();
}


void HDF::writeGroupAttributeInt(const char *groupName, const char *attributeName, const std::vector<int> &data) {
    LOCK();
    hid_t group_id = H5Gopen(file_id, groupName,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group '" << groupName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeInt(group_id, attributeName,data);
    H5Gclose(group_id);
    UNLOCK();
}

void HDF::writeGroupAttributeDouble(const char *groupName, const char *attributeName, const std::vector<double> &data) {
    LOCK();
    hid_t group_id = H5Gopen(file_id, groupName,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group '" << groupName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeDouble(group_id, attributeName,data);
    H5Gclose(group_id);
    UNLOCK();
}

void HDF::writeGroupAttributeFloat(const char *groupName, const char *attributeName, const std::vector<float> &data) {
    LOCK();
    hid_t group_id = H5Gopen(file_id, groupName,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group '" << groupName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeFloat(group_id, attributeName,data);
    H5Gclose(group_id);
    UNLOCK();
}


void HDF::writeDatasetAttributeInt(const char *datasetName, const char *attributeName, const std::vector<int> &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot open group '" << datasetName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeInt(dataset_id, attributeName,data);
    H5Dclose(dataset_id);
    UNLOCK();
}

void HDF::writeDatasetAttributeDouble(const char *datasetName, const char *attributeName, const std::vector<double> &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot open group '" << datasetName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeDouble(dataset_id, attributeName,data);
    H5Dclose(dataset_id);
    UNLOCK();
}

void HDF::writeDatasetAttributeFloat(const char *datasetName, const char *attributeName, const std::vector<float> &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot open group '" << datasetName << "' for writing attribute '" << attributeName << "'!\n";
        UNLOCK();
        return;
    }
    createAndWriteAttributeFloat(dataset_id, attributeName,data);
    H5Dclose(dataset_id);
    UNLOCK();
}


void HDF::writeDatasetAttributeInt(const char *datasetName, const char *attributeName, const int value) {
    vector<int> tmp(1,value);
    writeDatasetAttributeInt(datasetName,attributeName,tmp);
    tmp.clear();
}
        
void HDF::writeDatasetAttributeDouble(const char *datasetName, const char *attributeName, const double value) {
    vector<double> tmp(1,value);
    writeDatasetAttributeDouble(datasetName,attributeName,tmp);
    tmp.clear();
}

void HDF::writeDatasetAttributeFloat(const char *datasetName, const char *attributeName, const float value) {
    vector<float> tmp(1,value);
    writeDatasetAttributeFloat(datasetName,attributeName,tmp);
    tmp.clear();
}


void HDF::writeGroupAttributeInt(const char *groupName, const char *attributeName, const int value) {
    vector<int> tmp(1,value);
    writeGroupAttributeInt(groupName,attributeName,tmp);
    tmp.clear();
}

void HDF::writeGroupAttributeDouble(const char *groupName, const char *attributeName, const double value) {
    vector<double> tmp(1,value);
    writeGroupAttributeDouble(groupName,attributeName,tmp);
    tmp.clear();
}

void HDF::writeGroupAttributeFloat(const char *groupName, const char *attributeName, const float value) {
    vector<float> tmp(1,value);
    writeGroupAttributeFloat(groupName,attributeName,tmp);
    tmp.clear();
}



void HDF::writeGroupAttributes(const char *groupname, const MapInt &attribs) {
	for(CIntAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeGroupAttributeInt(groupname,name.c_str(), (*i).second);
	}
}

void HDF::writeGroupAttributes(const char *groupname, const MapDouble &attribs) {
	for(CDoubleAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeGroupAttributeDouble(groupname,name.c_str(), (*i).second);
	}

}

void HDF::writeGroupAttributes(const char *groupname, const MapFloat &attribs) {
	for(CFloatAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeGroupAttributeFloat(groupname,name.c_str(), (*i).second);
	}

}

void HDF::writeGroupAttributes(const char *groupname, const MapString &attribs) {
	for(CStrAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeGroupAttributeText(groupname,name.c_str(), (*i).second.c_str());
	}

}
		

void HDF::writeDatasetAttributes(const char *groupname, const MapInt &attribs) {
	for(CIntAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeDatasetAttributeInt(groupname,name.c_str(), (*i).second);
	}
}

void HDF::writeDatasetAttributes(const char *groupname, const MapDouble &attribs) {
	for(CDoubleAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeDatasetAttributeDouble(groupname,name.c_str(), (*i).second);
	}
}


void HDF::writeDatasetAttributes(const char *groupname, const MapFloat &attribs) {
	for(CFloatAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeDatasetAttributeFloat(groupname,name.c_str(), (*i).second);
	}
}


void HDF::writeDatasetAttributes(const char *groupname, const MapString &attribs) {
	for(CStrAttribIter i = attribs.begin(); i != attribs.end(); i++) {
		const string &name = (*i).first;
		writeDatasetAttributeText(groupname,name.c_str(), (*i).second.c_str());
	}
}










        
void HDF::writeDatasetString(const char *datasetName, const int row, const int col, const char *data, hid_t datasetid) {
	LOCK();
    hid_t id = datasetid;
    if (datasetid < 0) {
        id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "'!\n";
			UNLOCK();
            return;
        }
    }

   	hsize_t offset[2];
    offset[0] = row;
    offset[1] = col;

    hsize_t size[2];
    size[0] = 1;
    size[1] = 1;
    
    hid_t mem_space = H5Screate_simple(2,size,NULL);
 
	hsize_t len = strlen(data);
    hid_t datatype = H5Tcopy(H5T_C_S1);
//   herr_t status = H5Tset_size(datatype,len);
    herr_t status = H5Tset_size(datatype,H5T_VARIABLE);

	const char **wdata = new const char*[1];
 	wdata[0] = data;	
	hid_t filespace = H5Dget_space(id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    status = H5Dwrite(id,datatype,mem_space,filespace,H5P_DEFAULT,wdata);
 
//	status = 0;
    if (datasetid < 0) {
        H5Dclose(id);
    }
    H5Sclose(filespace);
    H5Sclose(mem_space);

	delete [] wdata;

    UNLOCK();


}


/** write one column at all */
void HDF::writeDatasetColumnString(const char *datasetName, const int col, const char **data, const int datasize, hid_t datasetid) {
	LOCK();
    hid_t id = datasetid;
    if (datasetid < 0) {
        id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "'!\n";
			UNLOCK();
            return;
        }
    }

	if (datasize > _getNumRows(id)) {
		cerr << "Given data are longer than dataset rows!: maxrows=" << _getNumRows(id) <<", data.size=" << datasize << "\n";
		UNLOCK();
		return;
	}

   	hsize_t offset[2];
    offset[0] = 0;
    offset[1] = col;

    hsize_t size[2];
    size[0] = datasize;
    size[1] = 1;
    
    hid_t mem_space = H5Screate_simple(2,size,NULL);
 
	//hsize_t len = strlen(data);
    hid_t datatype = H5Tcopy(H5T_C_S1);
    herr_t status = H5Tset_size(datatype,H5T_VARIABLE);

	hid_t filespace = H5Dget_space(id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    status = H5Dwrite(id,datatype,mem_space,filespace,H5P_DEFAULT,data);
 
//	status = 0;
    if (datasetid < 0) {
        H5Dclose(id);
    }
    H5Sclose(filespace);
    H5Sclose(mem_space);

	//delete [] wdata;

    UNLOCK();


}







void HDF::writeDatasetRowInt(const char *datasetName, const int row,const std::vector<int> &data,hid_t datasetid) {

    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);

        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }

    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t cols[2];
    cols[0] = 0;
    cols[1] = _getNumCols(dataset_id)-1;
    if ((int)cols[1] != (int)data.size()-1) {
        cerr << "Cannot write data to dataset '" << datasetName << "' of size " << _getNumRows(dataset_id)
            << " x " << _getNumCols(dataset_id) << " with data width (num cols) = " << data.size() << "\n";
        UNLOCK();
        return;
    }

    hsize_t size[2];
    size[0] = 1;
    size[1] = data.size();
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    int *d = new int[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    } 
    
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_INT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    delete [] d;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }
    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}

void HDF::writeDatasetRowDouble(const char *datasetName, const int row,const std::vector<double> &data, hid_t datasetid) {

    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }
    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t cols[2];
    cols[0] = 0;
    cols[1] = data.size()-1;
    //cols[1] = _getNumCols(dataset_id)-1;
    if ((int)cols[1] != (int)data.size()-1) {
        cerr << "Cannot write data to dataset '" << datasetName << "' of size " << _getNumRows(dataset_id)
            << " x " << _getNumCols(dataset_id) << " with data width (num cols) = " << data.size() << "\n";
        UNLOCK();
        return;
    }

    hsize_t size[2];
    size[0] = 1;
    size[1] = data.size();
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    double *d = new double[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    } 
    
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_DOUBLE,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    delete [] d;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}

void HDF::writeDatasetRowFloat(const char *datasetName, const int row,const std::vector<double> &data, hid_t datasetid) {
	vector<float> tmp;
	tmp.reserve(data.size());
	for(int i=0;i<(int)data.size();i++) {
		tmp.push_back((float)data[i]);
	}
	writeDatasetRowFloat(datasetName,row,tmp,datasetid);
	tmp.clear();

}
void HDF::writeDatasetRowFloat(const char *datasetName, const int row,const std::vector<float> &data, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }
    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t cols[2];
    cols[0] = 0;
    cols[1] = data.size()-1;
    //cols[1] = _getNumCols(dataset_id)-1;
    if ((int)cols[1] != (int)data.size()-1) {
        cerr << "Cannot write data to dataset '" << datasetName << "' of size " << _getNumRows(dataset_id)
            << " x " << _getNumCols(dataset_id) << " with data width (num cols) = " << data.size() << "\n";
        UNLOCK();
        return;
    }

    hsize_t size[2];
    size[0] = 1;
    size[1] = data.size();
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    float *d = new float[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    } 
    
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_FLOAT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    delete [] d;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}






void HDF::writeGroupAttributeText(const char *groupName, const char *attributeName, const char *value) {
    LOCK();
    hid_t group_id = H5Gopen(file_id,groupName,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "cannot open group '" << groupName << "' for writing attribute '" << attributeName << "'='"<<value << "' !\n";
        UNLOCK();
        return;
    }

    createAndWriteTextAttribute(group_id,attributeName,value);
    H5Gclose(group_id);
    UNLOCK();
}

void HDF::writeDatasetAttributeText(const char *datasetName, const char *attributeName, const char *value) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,datasetName,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "cannot open dataset '" << datasetName << "' for writing attribute '" << attributeName << "'='"<<value << "' !\n";
        UNLOCK();
        return;
    }
    createAndWriteTextAttribute(dataset_id,attributeName,value);
    H5Dclose(dataset_id);
    UNLOCK();
}


void HDF::writeDatasetColumnInt(const char *datasetName, const int col, const vector<int> &data) {
    int *d = new int[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    writeDatasetColumnInt(datasetName,col,d,data.size());
    delete [] d;
}

void HDF::writeDatasetColumnInt(const char *datasetName, const int col, const int *data, const int datasize) {

    LOCK();
        
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        
    if (dataset_id < 0) {
        cerr << "Cannot open dataset '" << datasetName << "' !\n";
        UNLOCK();
        return;
    }

    const int rows = _getNumRows(dataset_id);

    if (datasize != rows) {
        cerr << "Number of rows and data size differ: row="<<rows <<", data.size="<<datasize << "\n";
        UNLOCK();
        return;
    }
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = col;

    hsize_t size[2];
    size[0] = datasize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_INT,mem_space,filespace,H5P_DEFAULT,data);
    status = 0;
        
    H5Dclose(dataset_id);

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}

void HDF::writeDatasetColumnDouble(const char *datasetName, const int col, const vector<double> &data) {
    double *d = new double[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    writeDatasetColumnDouble(datasetName,col,d,data.size());
    delete [] d;
}

void HDF::writeDatasetColumnDouble(const char *datasetName, const int col, const double *data, const int datasize) {

    LOCK();
        
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        
    if (dataset_id < 0) {
        cerr << "Cannot open dataset '" << datasetName << "' !\n";
        UNLOCK();
        return;
    }

    const int rows = _getNumRows(dataset_id);

    if (datasize != rows) {
        cerr << "Number of rows and data size differ: row="<<rows <<", data.size="<<datasize << "\n";
        UNLOCK();
        return;
    }
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = col;

    hsize_t size[2];
    size[0] = datasize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_DOUBLE,mem_space,filespace,H5P_DEFAULT,data);
    status = 0;
        
    H5Dclose(dataset_id);

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}


/** sometimes it is needed to write double[] data as floats */
void HDF::writeDatasetColumnFloat(const char *datasetName, const int col, const vector<double> &data) {
    float *d = new float[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = (float)data[i];
    }
    writeDatasetColumnFloat(datasetName,col,d,data.size());
    delete [] d;
}

/** sometimes it is needed to write double[] data as floats */
void HDF::writeDatasetColumnFloat(const char *datasetName, const int col, const double *data, const int datasize) {
    float *d = new float[datasize];
    for(int i=0;i<datasize;i++) {
        d[i] = (float)data[i];
    }
    writeDatasetColumnFloat(datasetName,col,d,datasize);
    delete [] d;
}

void HDF::writeDatasetColumnFloat(const char *datasetName, const int col, const vector<float> &data) {
    float *d = new float[data.size()];
    for(int i=0;i<(int)data.size();i++) {
        d[i] = data[i];
    }
    writeDatasetColumnFloat(datasetName,col,d,data.size());
    delete [] d;
}

void HDF::writeDatasetColumnFloat(const char *datasetName, const int col, const float *data, const int datasize) {

    LOCK();
        
    hid_t dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        
    if (dataset_id < 0) {
        cerr << "Cannot open dataset '" << datasetName << "' !\n";
        UNLOCK();
        return;
    }

    const int rows = _getNumRows(dataset_id);

    if (datasize != rows) {
        cerr << "Number of rows and data size differ: row="<<rows <<", data.size="<<datasize << "\n";
        UNLOCK();
        return;
    }
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = col;

    hsize_t size[2];
    size[0] = datasize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_FLOAT,mem_space,filespace,H5P_DEFAULT,data);
    status = 0;
        
    H5Dclose(dataset_id);

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}



void HDF::writeDatasetValueInt(const char *datasetName, const int row,const int col, const int value, hid_t datasetid) {

    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }

    hsize_t offset[2];
    offset[0] = row;
    offset[1] = col;

    hsize_t cols[2];
    cols[0] = 1;
    cols[1] = 1;
    
    hsize_t size[2];
    size[0] = 1;
    size[1] = 1;

    int d[1];
    d[0] = value;
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_INT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }

    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}







void HDF::writeDatasetValueDouble(const char *datasetName, const int row,const int col, const double value, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }

    hsize_t offset[2];
    offset[0] = row;
    offset[1] = col;

    hsize_t cols[2];
    cols[0] = 1;
    cols[1] = 1;
    
//    if (cols[1] != (int)data.size()-1) {
  //      cerr << "Cannot write data to dataset '" << datasetName << "' of size " << _getNumRows(dataset_id)
    //        << " x " << _getNumCols(dataset_id) << " with data width (num cols) = " << data.size() << "\n";
      //  return;
//    }

    hsize_t size[2];
    size[0] = 1;
    size[1] = 1;

    double d[1];
    d[0] = value;
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_DOUBLE,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }


    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}

void HDF::writeDatasetValueFloat(const char *datasetName, const int row,const int col, const float value, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    if (datasetid < 0) {
        dataset_id = H5Dopen(file_id, datasetName,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << datasetName << "' !\n";
            UNLOCK();
            return;
        }
    }

    hsize_t offset[2];
    offset[0] = row;
    offset[1] = col;

    hsize_t cols[2];
    cols[0] = 1;
    cols[1] = 1;
    
//    if (cols[1] != (int)data.size()-1) {
  //      cerr << "Cannot write data to dataset '" << datasetName << "' of size " << _getNumRows(dataset_id)
    //        << " x " << _getNumCols(dataset_id) << " with data width (num cols) = " << data.size() << "\n";
      //  return;
//    }

    hsize_t size[2];
    size[0] = 1;
    size[1] = 1;

    float d[1];
    d[0] = value;
    
    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL); 

    hid_t mem_space = H5Screate_simple(2,size,NULL);
    
    herr_t status = H5Dwrite(dataset_id,H5T_NATIVE_FLOAT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }


    H5Sclose(filespace);
    H5Sclose(mem_space);
    UNLOCK();
}


bool HDF::isGroupCreated(const char *groupname) {
    LOCK();

   // herr_t (*old_func)(void*);
   // void *old_client_data;
   // H5Eget_auto(H5E_DEFAULT, &old_func, &old_client_data);
  //  herr_t e = H5Eset_auto(H5E_DEFAULT,NULL,NULL);


    hid_t gid = H5Gopen(file_id,groupname,H5P_DEFAULT);

 //   herr_t w = H5Eset_auto(H5E_DEFAULT,old_func,old_client_data);
    if (gid < 0) {
        UNLOCK();
        return false;
    }
    H5Gclose(gid);
    UNLOCK();
    return true;
}


/** assumes 2D table, return size of this table */
static void _getDatasetRowsCols(hid_t dataset_id, int &rows, int &cols) {
    hid_t filespace = H5Dget_space(dataset_id);
    const int dimss = H5Sget_simple_extent_ndims(filespace);
    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(filespace,dimsize,maxdims);
    rows = dimsize[0];
    cols = dimsize[1];
    delete [] dimsize;
    delete [] maxdims;
    H5Sclose(filespace);
}

static void _loadDatasetRowDouble(hid_t dataset_id,const int row, const int numcols, vector<double> &data) {
    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t size[2];
    size[0] = 1;
    size[1] = numcols;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

    double *d = new double[numcols];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_DOUBLE,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;

    H5Sclose(filespace);
    H5Sclose(mem_space);
    data.clear();
    data.reserve(numcols);
    for(int i=0;i<numcols;i++) {
        data.push_back(d[i]);
    }
    delete [] d;

}

static void _loadDatasetRowFloat(hid_t dataset_id,const int row, const int numcols, vector<float> &data) {
    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t size[2];
    size[0] = 1;
    size[1] = numcols;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

    float *d = new float[numcols];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_FLOAT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;

    H5Sclose(filespace);
    H5Sclose(mem_space);
    data.clear();
    data.reserve(numcols);
    for(int i=0;i<numcols;i++) {
        data.push_back(d[i]);
    }
    delete [] d;

}



static void _loadDatasetRowInt(hid_t dataset_id,const int row, const int numcols, vector<int> &data) {
    hsize_t offset[2];
    offset[0] = row;
    offset[1] = 0;

    hsize_t size[2];
    size[0] = 1;
    size[1] = numcols;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

    int *d = new int[numcols];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_INT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    H5Sclose(filespace);
    H5Sclose(mem_space);
    data.clear();
    data.reserve(numcols);
    for(int i=0;i<numcols;i++) {
        data.push_back(d[i]);
    }
    delete [] d;
}

/** loads given column from the dataset. The result is stored into d[] array, which has to be created
  * before calling the function. Maximal size of the d[] is maxdsize 
  */
static void _loadDatasetColumnInt(hid_t dataset_id,const int column, int *d, const int maxdsize) {
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = column;

	const int numrows = _getNumRows(dataset_id);
	const int lsize = std::min(numrows, maxdsize);
	if (numrows != maxdsize) {
		cerr << "Num of rows=" << numrows << ",d.size="<<maxdsize<<". we will load " << lsize << "\n";
	}

    hsize_t size[2];
    size[0] = lsize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

//    int *d = new int[size[0]];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_INT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    H5Sclose(filespace);
    H5Sclose(mem_space);
}

/** loads given column from the dataset. The result is stored into d[] array, which has to be created
  * before calling the function. Maximal size of the d[] is maxdsize 
  */
static void _loadDatasetColumnDouble(hid_t dataset_id,const int column, double *d, const int maxdsize) {
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = column;

	const int numrows = _getNumRows(dataset_id);
	const int lsize = std::min(numrows, maxdsize);
	if (numrows != maxdsize) {
		cerr << "Num of rows=" << numrows << ",d.size="<<maxdsize<<". we will load " << lsize << "\n";
	}

    hsize_t size[2];
    size[0] = lsize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

//    int *d = new int[size[0]];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_DOUBLE,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    H5Sclose(filespace);
    H5Sclose(mem_space);
}

/** loads given column from the dataset. The result is stored into d[] array, which has to be created
  * before calling the function. Maximal size of the d[] is maxdsize 
  */
static void _loadDatasetColumnFloat(hid_t dataset_id,const int column, float *d, const int maxdsize) {
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = column;

	const int numrows = _getNumRows(dataset_id);
	const int lsize = std::min(numrows, maxdsize);
	if (numrows != maxdsize) {
		cerr << "Num of rows=" << numrows << ",d.size="<<maxdsize<<". we will load " << lsize << "\n";
	}

    hsize_t size[2];
    size[0] = lsize;
    size[1] = 1;

    hid_t filespace = H5Dget_space(dataset_id);
    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

//    int *d = new int[size[0]];
    hid_t mem_space = H5Screate_simple(2,size,NULL);
    herr_t status = H5Dread(dataset_id,H5T_NATIVE_FLOAT,mem_space,filespace,H5P_DEFAULT,d);
    status = 0;
    H5Sclose(filespace);
    H5Sclose(mem_space);
}

/** loads given column from the dataset. The result is stored into d[] array, which has to be created
  * before calling the function. Maximal size of the d[] is maxdsize 
  */
static void _loadDatasetColumnString(hid_t dataset_id,const int column, char **d, const int maxdsize) {
    hsize_t offset[2];
    offset[0] = 0;
    offset[1] = column;

	const int numrows = _getNumRows(dataset_id);
	const int lsize = std::min(numrows, maxdsize);
	if (numrows != maxdsize) {
		cerr << "Num of rows=" << numrows << ",d.size="<<maxdsize<<". we will load " << lsize << "\n";
	}

    hsize_t size[2];
    size[0] = lsize;
    size[1] = 1;

    hid_t filetype = H5Dget_type(dataset_id);
    hid_t filespace = H5Dget_space(dataset_id);
    hid_t memtype = H5Tcopy(H5T_C_S1);
    herr_t status = H5Tset_size(memtype,H5T_VARIABLE);

    H5Sselect_hyperslab(filespace,H5S_SELECT_SET,offset,NULL,size,NULL);  

    hid_t mem_space = H5Screate_simple(2,size,NULL);

    char **rdata = new char*[ size[0] ];

    status = H5Dread(dataset_id,memtype,mem_space,filespace,H5P_DEFAULT,rdata);
    status = 0;

    for(int i=0;i<lsize;i++) {
        d[i] = new char[ strlen(rdata[i])+1 ];
        strcpy(d[i], rdata[i]);
    }


    H5Dvlen_reclaim(memtype,filespace,H5P_DEFAULT,rdata);
    delete [] rdata;

    H5Sclose(filespace);
    H5Sclose(mem_space);
}


void HDF::loadDatasetColumnString(const char *datasetname, const int col, vector<string> &data) {
	LOCK();
    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	data.clear();
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}
	const int numrows = _getNumRows(dataset_id);
    char **d = new char*[numrows];
	_loadDatasetColumnString(dataset_id,col,d,numrows);

    for(int i=0;i<numrows;i++) {
        data.push_back(string(d[i]));
        delete [] d[i];
    }
    delete [] d;
	H5Dclose(dataset_id);
	UNLOCK();
}


void HDF::loadDatasetColumnInt(const char *datasetname, const int col, vector<int> &data) {
	LOCK();

    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	data.clear();
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}

	const int numrows = _getNumRows(dataset_id);

	int *d = new int[numrows];

	_loadDatasetColumnInt(dataset_id,col,d,numrows);

	data.reserve(numrows);
	for(int i=0;i<numrows;i++) {
		data.push_back(d[i]);
	}
	delete [] d;

	H5Dclose(dataset_id);
	UNLOCK();
}

void HDF::loadDatasetColumnInt(const char *datasetname, const int col, int *d, const int maxdsize) {
	LOCK();
    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}
	_loadDatasetColumnInt(dataset_id,col,d,maxdsize);
	H5Dclose(dataset_id);
	UNLOCK();
}


void HDF::loadDatasetColumnDouble(const char *datasetname, const int col, vector<double> &data) {
	LOCK();

    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	data.clear();
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}

	const int numrows = _getNumRows(dataset_id);

	double *d = new double[numrows];

	_loadDatasetColumnDouble(dataset_id,col,d,numrows);

	data.reserve(numrows);
	for(int i=0;i<numrows;i++) {
		data.push_back(d[i]);
	}
	delete [] d;

	H5Dclose(dataset_id);
	UNLOCK();
}

void HDF::loadDatasetColumnDouble(const char *datasetname, const int col, double *d, const int maxdsize) {
	LOCK();
    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}
	_loadDatasetColumnDouble(dataset_id,col,d,maxdsize);
	H5Dclose(dataset_id);
	UNLOCK();
}

void HDF::loadDatasetColumnFloat(const char *datasetname, const int col, vector<float> &data) {
	LOCK();

    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	data.clear();
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}

	const int numrows = _getNumRows(dataset_id);

	float *d = new float[numrows];

	_loadDatasetColumnFloat(dataset_id,col,d,numrows);

	data.reserve(numrows);
	for(int i=0;i<numrows;i++) {
		data.push_back(d[i]);
	}
	delete [] d;

	H5Dclose(dataset_id);
	UNLOCK();
}

void HDF::loadDatasetColumnFloat(const char *datasetname, const int col, vector<double> &data) {
	LOCK();

    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	data.clear();
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}

	const int numrows = _getNumRows(dataset_id);

	float *d = new float[numrows];

	_loadDatasetColumnFloat(dataset_id,col,d,numrows);

	data.reserve(numrows);
	for(int i=0;i<numrows;i++) {
		data.push_back((double)d[i]);
	}
	delete [] d;

	H5Dclose(dataset_id);
	UNLOCK();
}


void HDF::loadDatasetColumnFloat(const char *datasetname, const int col, float *d, const int maxdsize) {
	LOCK();
    hid_t dataset_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	if (dataset_id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for loading column "<<col << " from it!\n";
		UNLOCK();
		return;
	}
	_loadDatasetColumnFloat(dataset_id,col,d,maxdsize);
	H5Dclose(dataset_id);
	UNLOCK();
}






static void readAttributeDouble(hid_t attr_id, vector<double> &data) {
    data.clear();
    hid_t aspace = H5Aget_space(attr_id);

    const int dimss = H5Sget_simple_extent_ndims(aspace);
    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(aspace,dimsize,maxdims);
    
    double *d = new double[dimsize[0]];
    herr_t status = H5Aread(attr_id,H5T_NATIVE_DOUBLE,d);
    status = 0;
    for(hsize_t i=0;i<dimsize[0];i++) {
        data.push_back(d[i]);
    }
    delete [] dimsize;
    delete [] maxdims;
    delete [] d;
    H5Sclose(aspace);
}

static void readAttributeFloat(hid_t attr_id, vector<float> &data) {
    data.clear();
    hid_t aspace = H5Aget_space(attr_id);

    const int dimss = H5Sget_simple_extent_ndims(aspace);
    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(aspace,dimsize,maxdims);
    
    float *d = new float[dimsize[0]];
    herr_t status = H5Aread(attr_id,H5T_NATIVE_FLOAT,d);
    status = 0;
    for(hsize_t i=0;i<dimsize[0];i++) {
        data.push_back(d[i]);
    }
    delete [] dimsize;
    delete [] maxdims;
    delete [] d;
    H5Sclose(aspace);
}



static void readAttributeInt(hid_t attr_id, vector<int> &data) {
    data.clear();
    hid_t aspace = H5Aget_space(attr_id);

    const int dimss = H5Sget_simple_extent_ndims(aspace);
    hsize_t *dimsize = new hsize_t[dimss];
    hsize_t *maxdims = new hsize_t[dimss];
    H5Sget_simple_extent_dims(aspace,dimsize,maxdims);
    
    int *d = new int[dimsize[0]];
    herr_t status = H5Aread(attr_id,H5T_NATIVE_INT,d);
    status = 0;
    for(hsize_t i=0;i<dimsize[0];i++) {
        data.push_back(d[i]);
    }
    delete [] dimsize;
    delete [] maxdims;
    delete [] d;
    H5Sclose(aspace);
}


static void readAttributeString(hid_t attr_id, std::string &data) {


	hid_t filetype = H5Aget_type(attr_id);
	hsize_t sdim = H5Tget_size(filetype);
	sdim++;

	hid_t aspace = H5Aget_space(attr_id);
	hsize_t dims[1];
	int ndims = H5Sget_simple_extent_dims(aspace,dims,NULL);

	char **rdata = new char*[dims[0]];
	rdata[0] = new char[dims[0]*sdim];

	hid_t mem_type = H5Tcopy(H5T_C_S1);
	herr_t status = H5Tset_size(mem_type,sdim);

	status = H5Aread(attr_id,mem_type,rdata[0]);
//	        cerr << "Attribute textattribute='" << rdata[0] << "'\n";
	data = std::string(rdata[0]);
	H5Tclose(filetype);
	H5Tclose(mem_type);
	H5Sclose(aspace);
	ndims = 0;
    delete [] rdata[0];
    delete [] rdata;
}   




bool HDF::isOpen() {
    LOCK();
    const bool res = (file_id >= 0);
    UNLOCK();
    return res;
}

int HDF::getDatasetSize(const char *datasetName, int &row, int &col) {
    LOCK();
    hid_t ds = H5Dopen(file_id,datasetName,H5P_DEFAULT);
    if (ds < 0) {
        UNLOCK();
        return -1;
    }
    _getDatasetRowsCols(ds,row,col);
    H5Dclose(ds);
    UNLOCK();
    return 1;
}

int HDF::loadDatasetRowInt(const char *name,const int row, std::vector<int> &data, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    
    if (dataset_id < 0) {
        dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << name << "'\n";
            UNLOCK();
            return -1;
        }
    }

    int rows, columns;
    _getDatasetRowsCols(dataset_id,rows,columns);

    if (row < 0 || row >= rows) {
        cerr << "Cannot load row " << row << " from dataset '" << name << "', because its num of rows is " << rows << "!\n";
        UNLOCK();
        return -1;
    }

    data.clear();
    _loadDatasetRowInt(dataset_id,row,columns,data);

    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }
    UNLOCK();
    return 1;
}

int HDF::loadDatasetRowDouble(const char *name,const int row, std::vector<double> &data, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    
    if (dataset_id < 0) {
        dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << name << "'\n";
            UNLOCK();
            return -1;
        }
    }

    int rows, columns;
    _getDatasetRowsCols(dataset_id,rows,columns);
    if (row < 0 || row >= rows) {
        cerr << "Cannot load row " << row << " from dataset '" << name << "', because its num of rows is " << rows << "!\n";
        UNLOCK();
        return -1;
    }

    data.clear();
    _loadDatasetRowDouble(dataset_id,row,columns,data);

    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }
    UNLOCK();
    return 1;
}

int HDF::loadDatasetRowFloat(const char *name,const int row, std::vector<float> &data, hid_t datasetid) {
    LOCK();
    hid_t dataset_id = datasetid;
    

    if (dataset_id < 0) {
        dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
        if (dataset_id < 0) {
            cerr << "Cannot open dataset '" << name << "'\n";
            UNLOCK();
            return -1;
        }
    }

    int rows, columns;
    _getDatasetRowsCols(dataset_id,rows,columns);
    if (row < 0 || row >= rows) {
        cerr << "Cannot load row " << row << " from dataset '" << name << "', because its num of rows is " << rows << "!\n";
        UNLOCK();
        return -1;
    }

    data.clear();
    _loadDatasetRowFloat(dataset_id,row,columns,data);

    if (datasetid < 0) {
        H5Dclose(dataset_id);
    }
    UNLOCK();
    return 1;
}


/*
int HDF::loadDatasetDouble(const char *name,std::vector< std::vector<double> > &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return 0;
    }
    data.clear();

    int rows, columns;
    _getDatasetRowsCols(dataset_id,rows,columns);
    
    data.clear();

    vector<double> tmp;
    for(int i=0;i<rows;i++) {
        _loadDatasetRowDouble(dataset_id,i,columns,tmp);
        data.push_back(tmp);
        tmp.clear();
    }
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}

int HDF::loadDatasetFloat(const char *name,std::vector< std::vector<float> > &data) {
    LOCK(); 
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return 0;
    }
    data.clear();

    int rows, columns;
    _getDatasetRowsCols(dataset_id,rows,columns);
    
    data.clear();

    vector<float> tmp;
    for(int i=0;i<rows;i++) {
        _loadDatasetRowFloat(dataset_id,i,columns,tmp);
        data.push_back(tmp);
        tmp.clear();
    }
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}
*/

int HDF::loadDatasetInt(const char *name,std::vector< std::vector<int> > &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    data.clear();
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return -1;
    }

	const int cols = _getNumCols(dataset_id);
	const int rows = _getNumRows(dataset_id);

	int *d = new int[rows];
	
	if (cols > 0) {
		for(int i=0;i<rows;i++) {
			data.push_back(vector<int>());
		}
	}

	for(int i=0;i<cols;i++) {
		_loadDatasetColumnInt(dataset_id,i,d,rows);
		for(int j=0;j<rows;j++){
			data[j].push_back(d[j]);
		}
    }
	delete [] d;
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}


int HDF::loadDatasetDouble(const char *name,std::vector< std::vector<double> > &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    data.clear();
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return -1;
    }

	const int cols = _getNumCols(dataset_id);
	const int rows = _getNumRows(dataset_id);

	double *d = new double[rows];
	
	if (cols > 0) {
		for(int i=0;i<rows;i++) {
			data.push_back(vector<double>());
		}
	}

	for(int i=0;i<cols;i++) {
		_loadDatasetColumnDouble(dataset_id,i,d,rows);
		for(int j=0;j<rows;j++){
			data[j].push_back(d[j]);
		}
    }
	delete [] d;
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}

int HDF::loadDatasetFloat(const char *name,std::vector< std::vector<float> > &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    data.clear();
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return -1;
    }

	const int cols = _getNumCols(dataset_id);
	const int rows = _getNumRows(dataset_id);

	float *d = new float[rows];
	
	if (cols > 0) {
		for(int i=0;i<rows;i++) {
			data.push_back(vector<float>());
		}
	}

	for(int i=0;i<cols;i++) {
		_loadDatasetColumnFloat(dataset_id,i,d,rows);
		for(int j=0;j<rows;j++){
			data[j].push_back(d[j]);
		}
    }
	delete [] d;
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}


int HDF::loadDatasetFloat(const char *name,std::vector< std::vector<double> > &data) {
    LOCK();
    hid_t dataset_id = H5Dopen(file_id,name,H5P_DEFAULT);
    data.clear();
    if (dataset_id < 0) {
        cerr << "Cannot load dataset " << name << "\n";
        UNLOCK();
        return -1;
    }

	const int cols = _getNumCols(dataset_id);
	const int rows = _getNumRows(dataset_id);

	float *d = new float[rows];
	
	if (cols > 0) {
		for(int i=0;i<rows;i++) {
			data.push_back(vector<double>());
		}
	}

	for(int i=0;i<cols;i++) {
		_loadDatasetColumnFloat(dataset_id,i,d,rows);
		for(int j=0;j<rows;j++){
			data[j].push_back((double)d[j]);
		}
    }
	delete [] d;
    H5Dclose(dataset_id);
    UNLOCK();
    return 1;

}



int HDF::readDatasetAttributeInt(const char *datasetname, const char *attribname, std::vector<int> &values) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	values.clear();
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	readAttributeInt(aid,values);

	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}

int HDF::readDatasetAttributeInt(const char *datasetname, const char *attribname, int &value) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	value = 0;
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	vector<int> tmp;
	readAttributeInt(aid,tmp);
	value = tmp[0];
	tmp.clear();
	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}


int HDF::readDatasetAttributeDouble(const char *datasetname, const char *attribname, std::vector<double> &values) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	values.clear();
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	readAttributeDouble(aid,values);

	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}

int HDF::readDatasetAttributeDouble(const char *datasetname, const char *attribname, double &value) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	value = 0;
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	vector<double> tmp;
	readAttributeDouble(aid,tmp);
	value = tmp[0];
	tmp.clear();
	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}




int HDF::readDatasetAttributeFloat(const char *datasetname, const char *attribname, std::vector<float> &values) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	values.clear();
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	readAttributeFloat(aid,values);

	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}

int HDF::readDatasetAttributeFloat(const char *datasetname, const char *attribname, float &value) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	value = 0;
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	vector<float> tmp;
	readAttributeFloat(aid,tmp);
	value = tmp[0];
	tmp.clear();
	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}


int HDF::readDatasetAttributeFloat(const char *datasetname, const char *attribname, std::vector<double> &values) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	values.clear();
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	vector<float> tmp;
	readAttributeFloat(aid,tmp);
	for(int i=0;i<(int)tmp.size();i++) {
		values.push_back(tmp[i]);
	}
	tmp.clear();

	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}

int HDF::readDatasetAttributeFloat(const char *datasetname, const char *attribname, double &value) {
	LOCK();
	hid_t id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
	value = 0;
	if (id < 0) {
		cerr << "Cannot open dataset '" << datasetname << "' for obtaning value of attribute '" << attribname << "'!\n";
		UNLOCK();
		return -1;
	}
            
    hid_t aid = H5Aopen(id,attribname,H5P_DEFAULT);
	if (aid < 0) {
		cerr << "Cannot open attribute '" << attribname << "' in dataset '" << datasetname << "'!\n";
		H5Dclose(id);
		UNLOCK();
		return -1;
	}

	vector<float> tmp;
	readAttributeFloat(aid,tmp);
	value = (double)tmp[0];
	tmp.clear();
	H5Aclose(aid);
	H5Dclose(id);
	UNLOCK();
	return 1;
}




/** load all attributes from given object. It is assumed, that object_id was open before (e.g. by H5Gopen or H5Dopen)
  *
  * return 1 if everuthing is ok, otherwise return -1
  */
static int readAllAttributes(hid_t object_id, HDF::MapInt &attributes) {
    attributes.clear();

    H5O_info_t info;
    H5Oget_info(object_id,&info);
    const int numattrs = info.num_attrs;
    char name[2000];
    vector<int> tmp;
    for(int i=0;i<numattrs;i++) {
        hid_t attr_id = H5Aopen_by_idx(object_id,".",H5_INDEX_NAME,H5_ITER_INC,i,H5P_DEFAULT,H5P_DEFAULT);
        hid_t attrib_type = H5Aget_type(attr_id);
        H5T_class_t aclass = H5Tget_class(attrib_type);
        if (aclass == H5T_INTEGER) {
			const int strsize = H5Aget_name(attr_id,0,NULL);
            readAttributeInt(attr_id,tmp);
			if (strsize < 2000) {
		        const int ss = H5Aget_name(attr_id,2000,name);
            	attributes[std::string(name)] = tmp;
			} else {
				char *a = new char[strsize+1];
		        const int ss = H5Aget_name(attr_id,strsize,a);
            	attributes[std::string(a)] = tmp;
				delete [] a;
			}
            tmp.clear();                              
        }
		H5Tclose(attrib_type);
        H5Aclose(attr_id);
    }

    return 1;
}


/** read all attrutes stored in given group */
static int readAllAttributes(hid_t object_id, HDF::MapDouble &attributes) {
    attributes.clear();

    H5O_info_t info;
    H5Oget_info(object_id,&info);
    const int numattrs = info.num_attrs;
    char name[2000];
    vector<double> tmp;
    for(int i=0;i<numattrs;i++) {
        hid_t attr_id = H5Aopen_by_idx(object_id,".",H5_INDEX_NAME,H5_ITER_INC,i,H5P_DEFAULT,H5P_DEFAULT);
        hid_t attrib_type = H5Aget_type(attr_id);
        H5T_class_t aclass = H5Tget_class(attrib_type);
        if (aclass == H5T_FLOAT) {
			const int strsize = H5Aget_name(attr_id,0,NULL);
            readAttributeDouble(attr_id,tmp);
			if (strsize < 2000) {
				const int ss = H5Aget_name(attr_id,2000,name);
            	attributes[std::string(name)] = tmp;
			} else {
				char *a = new char[strsize+1];
				const int ss = H5Aget_name(attr_id,strsize,a);
            	attributes[std::string(a)] = tmp;
				delete [] a;
			}
            tmp.clear();                              
        }
		H5Tclose(attrib_type);
        H5Aclose(attr_id);
    }
    return 1;
}

 
/** read all attrutes stored in given group */
static int readAllAttributes(hid_t object_id, HDF::MapFloat &attributes) {

    cerr << "This function is not implemented yet!\n";
    return -1;
    /*
    attributes.clear();

    H5O_info_t info;
    H5Oget_info(object_id,&info);
    const int numattrs = info.num_attrs;
    char name[2000];
    vector<double> tmp;
    for(int i=0;i<numattrs;i++) {
        hid_t attr_id = H5Aopen_by_idx(object_id,".",H5_INDEX_NAME,H5_ITER_INC,i,H5P_DEFAULT,H5P_DEFAULT);
        const int ss = H5Aget_name(attr_id,2000,name);
        name[ss] = 0;
        hid_t attrib_type = H5Aget_type(attr_id);
        H5T_class_t aclass = H5Tget_class(attrib_type);
        cerr << "Attribname 
        if (aclass == H5T_FLOAT) {
            readAttributeDouble(attr_id,tmp);
            attributes[std::string(name)] = tmp;
            tmp.clear();                              
        }
        H5Aclose(attr_id);
    }
    return 1;
    */
}

 
static int readAllAttributes(hid_t object_id, HDF::MapString &attributes) {
    attributes.clear();

    H5O_info_t info;
    H5Oget_info(object_id,&info);
    const int numattrs = info.num_attrs;
    char name[2000];
    string tmp;
    for(int i=0;i<numattrs;i++) {
        hid_t attr_id = H5Aopen_by_idx(object_id,".",H5_INDEX_NAME,H5_ITER_INC,i,H5P_DEFAULT,H5P_DEFAULT);
        hid_t attrib_type = H5Aget_type(attr_id);
        H5T_class_t aclass = H5Tget_class(attrib_type);
        if (aclass == H5T_STRING) {
        	const int strsize = H5Aget_name(attr_id,0,NULL);
            readAttributeString(attr_id,tmp);
			if (strsize < 2000) {
	        	const int ss = H5Aget_name(attr_id,2000,name);
            	attributes[std::string(name)] = tmp;
			} else {
				char *a = new char[strsize+1];
	        	const int ss = H5Aget_name(attr_id,strsize,a);
            	attributes[std::string(a)] = tmp;
				delete [] a;
			}
            tmp.clear();    
        }
		H5Tclose(attrib_type);
        H5Aclose(attr_id);
    }
    return 1;

}

/** read all attrutes stored in given group */
int HDF::readAllGroupAttributes(const char *group, MapInt &attributes) {
    LOCK();
    hid_t group_id = H5Gopen(file_id,group,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group " << group << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(group_id, attributes);
    H5Gclose(group_id);
    UNLOCK();
    return r;
}


/** read all attrutes stored in given group */
int HDF::readAllGroupAttributes(const char *group, MapDouble &attributes) {
    LOCK();
    hid_t group_id = H5Gopen(file_id,group,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group " << group << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(group_id, attributes);
    H5Gclose(group_id);
    UNLOCK();
    return r;
}


/** read all attrutes stored in given group */
int HDF::readAllGroupAttributes(const char *group, MapFloat &attributes) {
    LOCK();
    hid_t group_id = H5Gopen(file_id,group,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group " << group << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(group_id, attributes);
    H5Gclose(group_id);
    UNLOCK();
    return r;
}


        
int HDF::readAllGroupAttributes(const char *group, MapString &attributes) {
    LOCK();
    hid_t group_id = H5Gopen(file_id,group,H5P_DEFAULT);
    if (group_id < 0) {
        cerr << "Cannot open group " << group << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(group_id, attributes);
    H5Gclose(group_id);
    UNLOCK();
    return r;
}



int HDF::readAllDatasetAttributes(const char *datasetname, MapInt &attributes) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(object_id, attributes);
    H5Dclose(object_id);
    UNLOCK();
    return r;
}

int HDF::readAllDatasetAttributes(const char *datasetname, MapDouble &attributes) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(object_id, attributes);
    H5Dclose(object_id);
    UNLOCK();
    return r;
}


int HDF::readAllDatasetAttributes(const char *datasetname, MapFloat &attributes) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(object_id, attributes);
    H5Dclose(object_id);
    UNLOCK();
    return r;
}


int HDF::readAllDatasetAttributes(const char *datasetname, MapString &attributes) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for reading attributes!\n";
        UNLOCK();
        return -1;
    }
    const int r = readAllAttributes(object_id, attributes);
    H5Dclose(object_id);
    UNLOCK();
    return r;
}



int HDF::getNumRows(const char *datasetname) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for obtaining its number of rows!\n";
        UNLOCK();
        return -1;
    }
    const int r = _getNumRows(object_id);
	H5Dclose(object_id);
    UNLOCK();
    return r;
}


int HDF::getNumCols(const char *datasetname) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset " << datasetname << " for obtaining its number of rows!\n";
        UNLOCK();
        return -1;
    }
    const int r = _getNumCols(object_id);
	H5Dclose(object_id);
    UNLOCK();
    return r;
}



     
hid_t HDF::openDataset(const char *datasetName) {
    LOCK();
    hid_t ds = H5Dopen(file_id,datasetName,H5P_DEFAULT);
    UNLOCK();
    return ds;
}
        
void HDF::closeDataset(hid_t dataset_id) {
    LOCK();
    H5Dclose(dataset_id);
    UNLOCK();
}
      

int HDF::assignAttributeFromList(const MapInt &attributes, const char *name, vector<int> &output) {

	CIntAttribIter f = attributes.find(string(name));
	if (f == attributes.end()) {
		return -1;
	} 

	// key was found
	output = (*f).second;
	return 0;
}

int HDF::assignAttributeFromList(const MapInt &attributes, const char *name, int &output) {

	CIntAttribIter f = attributes.find(string(name));
	if (f == attributes.end()) {
		return -1;
	} 

	// key was found
	output = ((*f).second)[0];
	return 0;
}

int HDF::assignAttributeFromList(const MapDouble &attributes, const char *name, vector<double> &output) {

	CDoubleAttribIter f = attributes.find(string(name));
	if (f == attributes.end()) {
		return -1;
	} 

	// key was found
	output = (*f).second;
	return 0;
}

int HDF::assignAttributeFromList(const MapDouble &attributes, const char *name, double &output) {

	CDoubleAttribIter f = attributes.find(string(name));
	if (f == attributes.end()) {
		return -1;
	} 

	// key was found
	output = ((*f).second)[0];
	return 0;
}

int HDF::assignAttributeFromList(const MapString &attributes, const char *name, std::string &output) {

	CStrAttribIter f = attributes.find(string(name));
	if (f == attributes.end()) {
		return -1;
	} 
	output = (*f).second;
	return 0;
}


/** deletes attribute from the dataset. return 1 if atrribute was deleted, otherwise return -1 */
int HDF::deleteDatasetAttribute(const char *datasetname, const char *attribname) {
    LOCK();
    hid_t object_id = H5Dopen(file_id,datasetname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open dataset '" << datasetname << "' for deleting attribute '" << attribname << "' from it!" <<
            " attribute will not be deleted\n";
        UNLOCK();
        return -1;
    }

    const int r = deleteAttribute(object_id, attribname);
    H5Dclose(object_id);
    UNLOCK();
    return r; 
}

/** deletes attribute from the given group. return 1 if atrribute was deleted, otherwise return -1 */
int HDF::deleteGroupAttribute(const char *groupname, const char *attribname) {
    LOCK();
    hid_t object_id = H5Gopen(file_id,groupname,H5P_DEFAULT);
    if (object_id < 0) {
        cerr << "Cannot open group '" << groupname << "' for deleting attribute '" << attribname << "' from it!" <<
            " attribute will not be deleted\n";
        UNLOCK();
        return -1;
    }

    const int r = deleteAttribute(object_id, attribname);
    H5Gclose(object_id);
    UNLOCK();
    return r; 
}


int HDF::updateGroupAttributeInt(const char *groupname, const char *attribname, const int val) {
    return updateGroupAttributeInt(groupname, attribname, vector<int>(1,val));
}


int HDF::updateGroupAttributeInt(const char *groupname, const char *attribname, const vector<int> &val) {
    LOCK();
    hid_t gid = H5Gopen(file_id, groupname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Group does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Atriute does not exiest!\n";
    	H5Gclose(gid);
        UNLOCK();
        return -1;
    }
    writeAttributeInt(aid,val);
    H5Aclose(aid);
    H5Gclose(gid);
    UNLOCK();
    return 1;
}

int HDF::updateGroupAttributeDouble(const char *groupname, const char *attribname, const double val) {
    return updateGroupAttributeDouble(groupname, attribname, vector<double>(1,val));
}


int HDF::updateGroupAttributeDouble(const char *groupname, const char *attribname, const vector<double> &val) {
    LOCK();
    hid_t gid = H5Gopen(file_id, groupname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Group does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Atriute does not exiest!\n";
    	H5Gclose(gid);
        UNLOCK();
        return -1;
    }
    cerr << "%";
    writeAttributeDouble(aid,val);
    H5Aclose(aid);
    H5Gclose(gid);
    UNLOCK();
    return 1;
}


int HDF::updateGroupAttributeFloat(const char *groupname, const char *attribname, const float val) {
    return updateGroupAttributeFloat(groupname, attribname, vector<float>(1,val));
}


int HDF::updateGroupAttributeFloat(const char *groupname, const char *attribname, const vector<float> &val) {
    LOCK();
    hid_t gid = H5Gopen(file_id, groupname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Group does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Atriute does not exiest!\n";
    	H5Gclose(gid);
        UNLOCK();
        return -1;
    }
    cerr << "%";
    writeAttributeFloat(aid,val);
    H5Aclose(aid);
    H5Gclose(gid);
    UNLOCK();
    return 1;
}


int HDF::updateGroupAttributeFloat(const char *groupname, const char *attribname, const double val) {
    return updateGroupAttributeFloat(groupname, attribname, vector<float>(1,(float)val));
}


int HDF::updateGroupAttributeFloat(const char *groupname, const char *attribname, const vector<double> &val) {
    LOCK();
    hid_t gid = H5Gopen(file_id, groupname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Group does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in group '" << groupname <<"'. Atriute does not exiest!\n";
    	H5Gclose(gid);
        UNLOCK();
        return -1;
    }
    vector<float> tmp;
	for(int i=0;i<(int)val.size();i++) {
		tmp.push_back((float)val[i]);
	}
	writeAttributeFloat(aid,tmp);
	tmp.clear();
    H5Aclose(aid);
    H5Gclose(gid);
    UNLOCK();
    return 1;
}



int HDF::updateDatasetAttributeInt(const char *datasetname, const char *attribname, const int val) {
    return updateDatasetAttributeInt(datasetname, attribname, vector<int>(1,val));
}


int HDF::updateDatasetAttributeInt(const char *datasetname, const char *attribname, const vector<int> &val) {
    LOCK();
    hid_t gid = H5Dopen(file_id, datasetname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exiest!\n";
    	H5Dclose(gid);
        UNLOCK();
        return -1;
    }
    writeAttributeInt(aid,val);
    H5Aclose(aid);
    H5Dclose(gid);
    UNLOCK();
    return 1;
}

int HDF::updateDatasetAttributeDouble(const char *datasetname, const char *attribname, const double val) {
    return updateDatasetAttributeDouble(datasetname, attribname, vector<double>(1,val));
}


int HDF::updateDatasetAttributeDouble(const char *datasetname, const char *attribname, const vector<double> &val) {
    LOCK();
    hid_t gid = H5Dopen(file_id, datasetname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exiest!\n";
    	H5Dclose(gid);
        UNLOCK();
        return -1;
    }
    writeAttributeDouble(aid,val);
    H5Aclose(aid);
    H5Dclose(gid);
    UNLOCK();
    return 1;
}

int HDF::updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const float val) {
    return updateDatasetAttributeFloat(datasetname, attribname, vector<float>(1,val));
}


int HDF::updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const vector<float> &val) {
    LOCK();
    hid_t gid = H5Dopen(file_id, datasetname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exiest!\n";
    	H5Dclose(gid);
        UNLOCK();
        return -1;
    }
    writeAttributeFloat(aid,val);
    H5Aclose(aid);
    H5Dclose(gid);
    UNLOCK();
    return 1;
}


int HDF::updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const double val) {
    return updateDatasetAttributeFloat(datasetname, attribname, vector<float>(1,(float)val));
}


int HDF::updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const vector<double> &val) {
    LOCK();
    hid_t gid = H5Dopen(file_id, datasetname, H5P_DEFAULT);
    if (gid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exist!\n";
        UNLOCK();
        return -1;
    }
    hid_t aid = H5Aopen(gid,attribname,H5P_DEFAULT);
    if (aid < 0) {
        cerr << "Cannot update attribute '"<<attribname<<"' in dataset '" << datasetname <<"'. Dataset does not exiest!\n";
    	H5Dclose(gid);
        UNLOCK();
        return -1;
    }

	vector<float> tmp;
	for(int i=0;i<(int)val.size();i++) {
		tmp.push_back(val[i]);
	}
    writeAttributeFloat(aid,tmp);
	tmp.clear();
    H5Aclose(aid);
    H5Dclose(gid);
    UNLOCK();
    return 1;
}










































static herr_t object_info(hid_t loc_id, const char *name, const H5L_info_t *info, void *data) {
	H5O_info_t infobuf;
	vector< pair<bool, string> > * list = (vector< pair<bool, string> > *)data;
	herr_t status = H5Oget_info_by_name(loc_id, name, &infobuf, H5P_DEFAULT);
	switch(infobuf.type) {
		case H5O_TYPE_GROUP: {
				//cerr << "Ob" << name << " is group\n"; 
				list->push_back(pair<bool, string>(false,string(name)));
				break;
		}
		case H5O_TYPE_DATASET: { 
				//cerr << "Ob " << name << " is dataset\n"; 
				list->push_back(pair<bool, string>(true,string(name)));
				break;
		}
		case H5O_TYPE_NAMED_DATATYPE: break;
		case H5O_TYPE_UNKNOWN: break;
		case H5O_TYPE_NTYPES: break;
	}
	return 0;
}


static void listObjects(hid_t obj_id, vector<string> &names, const bool returnDataset) {
	vector<pair<bool, string> > lst;
	herr_t status=H5Literate(obj_id, H5_INDEX_NAME,H5_ITER_NATIVE,NULL,object_info,(void *)&lst);
	names.clear();
	for(int i=0;i<(int)lst.size();i++) {
		if (lst[i].first == returnDataset) {
			names.push_back(lst[i].second);
		}
	}
	lst.clear();
}


/** return list of groups inside the given group */
void HDF::listGroups(const char *groupname, std::vector<std::string> &groupnames) {
	LOCK();
	hid_t id = H5Gopen(file_id, groupname, H5P_DEFAULT);
	if (id < 0) {
		cerr << "Cannot open group '" << groupname << "' for listing its groups!\n";
		UNLOCK();
		return;
	}
	groupnames.clear();
	listObjects(id,groupnames,false);
	H5Gclose(id);
	UNLOCK();
}


/** return datasetnames inside the given group */
void HDF::listDatasets(const char *groupname, std::vector<std::string> &datasetnames) {
	LOCK();
	hid_t id = H5Gopen(file_id, groupname, H5P_DEFAULT);
	datasetnames.clear();
	if (id < 0) {
		cerr << "Cannot open group '" << groupname << "' for listing its groups!\n";
		UNLOCK();
		return;
	}
	listObjects(id,datasetnames,true);
	H5Gclose(id);
	UNLOCK();
}





