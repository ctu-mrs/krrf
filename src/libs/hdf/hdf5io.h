#ifndef HDF5IO_H_
#define HDF5IO_H_

#define USE_SEMAPHORES 1

#include <hdf5.h>
#include <vector>
#include <map>
#include <string>

#ifdef USE_SEMAPHORES
#include <semaphore.h>
#endif

class HDF {
	public:

		typedef std::map< std::string, std::vector<int> > MapInt;
		typedef std::map< std::string, std::vector<double> > MapDouble;
		typedef std::map< std::string, std::vector<float> > MapFloat;
		typedef std::map< std::string, std::string > MapString;

        typedef MapInt::iterator IntAttribIter;
        typedef MapDouble::iterator DoubleAttribIter;
        typedef MapFloat::iterator FloatAttribIter;
        typedef MapString::iterator StrAttribIter;

		typedef MapInt::const_iterator CIntAttribIter;
        typedef MapDouble::const_iterator CDoubleAttribIter;
        typedef MapFloat::const_iterator CFloatAttribIter;
        typedef MapString::const_iterator CStrAttribIter;





		HDF();
        ~HDF();

		void openOrCreate(const char *filename);
		void open(const char *filename);
		void createFile(const char *filename);
        bool isOpen();
		void close();

        bool isThreaded() const;

        int getDatasetSize(const char *datasetName, int &row, int &col);
        void closeDataset(hid_t dataset_id);
        hid_t openDataset(const char *name);

        void createGroup(const char *groupName);
        void createDatasetInt(const char *name, const int rows, const int cols);
        void createDatasetDouble(const char *name, const int rows, const int cols);
        void createDatasetFloat(const char *name, const int rows, const int cols);
        void createDatasetString(const char *name, const int rows, const int cols);

        void writeGroupAttributeInt(const char *groupName, const char *attributeName, const std::vector<int> &data);
        void writeGroupAttributeInt(const char *groupName, const char *attributeName, const int value);
        void writeGroupAttributeDouble(const char *groupName, const char *attributeName, const std::vector<double> &data);
        void writeGroupAttributeDouble(const char *groupName, const char *attributeName, const double value);
        void writeGroupAttributeFloat(const char *groupName, const char *attributeName, const std::vector<float> &data);
        void writeGroupAttributeFloat(const char *groupName, const char *attributeName, const float value);

		void writeGroupAttributes(const char *groupname, const MapInt &attribs);
		void writeGroupAttributes(const char *groupname, const MapDouble &attribs);
		void writeGroupAttributes(const char *groupname, const MapFloat &attribs);
		void writeGroupAttributes(const char *groupname, const MapString &attribs);
		
		void writeDatasetAttributes(const char *groupname, const MapInt &attribs);
		void writeDatasetAttributes(const char *groupname, const MapDouble &attribs);
		void writeDatasetAttributes(const char *groupname, const MapFloat &attribs);
		void writeDatasetAttributes(const char *groupname, const MapString &attribs);

        void writeDatasetAttributeInt(const char *datasetName, const char *attributeName, const std::vector<int> &data);
        void writeDatasetAttributeInt(const char *datasetName, const char *attributeName, const int value);
        void writeDatasetAttributeDouble(const char *datasetName, const char *attributeName, const std::vector<double> &data);
        void writeDatasetAttributeDouble(const char *datasetName, const char *attributeName, const double value);
        void writeDatasetAttributeFloat(const char *datasetName, const char *attributeName, const std::vector<float> &data);
        void writeDatasetAttributeFloat(const char *datasetName, const char *attributeName, const float value);

        void writeDatasetRowInt(const char *datasetName, const int row,const std::vector<int> &data, hid_t datasetid=-1);
        void writeDatasetRowDouble(const char *datasetName, const int row,const std::vector<double> &data, hid_t datasetid = -1);
        void writeDatasetRowFloat(const char *datasetName, const int row,const std::vector<float> &data, hid_t datasetid = -1);
        void writeDatasetRowFloat(const char *datasetName, const int row,const std::vector<double> &data, hid_t datasetid = -1);
        void writeDatasetString(const char *datasetName, const int row, const int col, const char *data, hid_t datasetid = -1);
		void writeDatasetColumnString(const char *datasetName, const int col, const char **data, const int datasize, hid_t datasetid= -1);


        void writeDatasetColumnInt(const char *datasetName, const int col, const std::vector<int> &data);
        void writeDatasetColumnInt(const char *datasetName, const int col, const int *data, const int datasize);
        
        void writeDatasetColumnDouble(const char *datasetName, const int col, const std::vector<double> &data);
        void writeDatasetColumnDouble(const char *datasetName, const int col, const double *data, const int datasize);
        
        void writeDatasetColumnFloat(const char *datasetName, const int col, const std::vector<float> &data);
        void writeDatasetColumnFloat(const char *datasetName, const int col, const float *data, const int datasize);
        void writeDatasetColumnFloat(const char *datasetName, const int col, const double *data, const int datasize);
        void writeDatasetColumnFloat(const char *datasetName, const int col, const std::vector<double> &data);

        void writeDatasetAttributeText(const char *datasetName, const char *attributeName, const char *value);
        void writeGroupAttributeText(const char *groupName, const char *attributeName, const char *value);


        void writeDatasetValueInt(const char *datasetName, const int row,const int col, const int value, hid_t datasetid = -1);
        void writeDatasetValueDouble(const char *datasetName, const int row,const int col, const double value, hid_t datasetid = -1);
        void writeDatasetValueFloat(const char *datasetName, const int row,const int col, const float value, hid_t datasetid = -1);

        bool isGroupCreated(const char *groupname);

        int loadDatasetInt(const char *name,std::vector< std::vector<int> > &data);
        int loadDatasetDouble(const char *name,std::vector< std::vector<double> > &data);
        int loadDatasetFloat(const char *name,std::vector< std::vector<float> > &data);
        int loadDatasetFloat(const char *name,std::vector< std::vector<double> > &data);

        int loadDatasetRowInt(const char *name, const int row, std::vector<int> &data, hid_t datasetid = -1);
        int loadDatasetRowDouble(const char *name, const int row, std::vector<double> &data, hid_t datasetid = -1);
        int loadDatasetRowFloat(const char *name, const int row, std::vector<float> &data, hid_t datasetid = -1);

		void loadDatasetColumnInt(const char *datasetname, const int col, int *d, const int maxdsize);
		void loadDatasetColumnInt(const char *datasetname, const int col, std::vector<int> &data);
		void loadDatasetColumnDouble(const char *datasetname, const int col, double *d, const int maxdsize);
		void loadDatasetColumnDouble(const char *datasetname, const int col, std::vector<double> &data);
		void loadDatasetColumnFloat(const char *datasetname, const int col, float *d, const int maxdsize);
		void loadDatasetColumnFloat(const char *datasetname, const int col, std::vector<float> &data);
		void loadDatasetColumnFloat(const char *datasetname, const int col, std::vector<double> &data);
		
        void loadDatasetColumnString(const char *datasetname, const int col, std::vector<std::string> &data);


        int readAllGroupAttributes(const char *group, MapInt &attributes);
        int readAllGroupAttributes(const char *group, MapDouble &attributes);
        int readAllGroupAttributes(const char *group, MapString &attributes);
        int readAllGroupAttributes(const char *group, MapFloat &attributes);
        
        int readAllDatasetAttributes(const char *datasetname, MapInt &attributes);
        int readAllDatasetAttributes(const char *datasetname, MapDouble &attributes);
        int readAllDatasetAttributes(const char *datasetname, MapString &attributes);
        int readAllDatasetAttributes(const char *datasetname, MapFloat &attributes);

		int assignAttributeFromList(const MapInt &attributes, const char *name, std::vector<int> &output);
		int assignAttributeFromList(const MapInt &attributes, const char *name, int &output);
		
		int assignAttributeFromList(const MapDouble &attributes, const char *name, std::vector<double> &output);
		int assignAttributeFromList(const MapDouble &attributes, const char *name, double &output);
		
		int assignAttributeFromList(const MapString &attributes, const char *name, std::string &output);

        int deleteDatasetAttribute(const char *datasetname, const char *attribname);
        int deleteGroupAttribute(const char *groupname, const char *attribname);

        int updateGroupAttributeInt(const char *groupname, const char *attribname, const int val);
        int updateGroupAttributeInt(const char *groupname, const char *attribname, const std::vector<int> &val);
        int updateGroupAttributeFloat(const char *groupname, const char *attribname, const float val);
        int updateGroupAttributeFloat(const char *groupname, const char *attribname, const std::vector<float> &val);
        int updateGroupAttributeDouble(const char *groupname, const char *attribname, const double val);
        int updateGroupAttributeDouble(const char *groupname, const char *attribname, const std::vector<double> &val);
        int updateGroupAttributeFloat(const char *groupname, const char *attribname, const double val);
        int updateGroupAttributeFloat(const char *groupname, const char *attribname, const std::vector<double> &val);

        int updateDatasetAttributeInt(const char *datasetname, const char *attribname, const int val);
        int updateDatasetAttributeInt(const char *datasetname, const char *attribname, const std::vector<int> &val);
        int updateDatasetAttributeDouble(const char *datasetname, const char *attribname, const double val);
        int updateDatasetAttributeDouble(const char *datasetname, const char *attribname, const std::vector<double> &val);
        int updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const float val);
        int updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const std::vector<float> &val);
        int updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const double val);
        int updateDatasetAttributeFloat(const char *datasetname, const char *attribname, const std::vector<double> &val);
		
		int readDatasetAttributeInt(const char *datasetname, const char *attribname, std::vector<int> &values);
		int readDatasetAttributeInt(const char *datasetname, const char *attribname, int &value);
		int readDatasetAttributeDouble(const char *datasetname, const char *attribname, std::vector<double> &values);
		int readDatasetAttributeDouble(const char *datasetname, const char *attribname, double &value);
		int readDatasetAttributeFloat(const char *datasetname, const char *attribname, std::vector<float> &values);
		int readDatasetAttributeFloat(const char *datasetname, const char *attribname, float &value);
		int readDatasetAttributeFloat(const char *datasetname, const char *attribname, std::vector<double> &values);
		int readDatasetAttributeFloat(const char *datasetname, const char *attribname, double &value);


		void listGroups(const char *groupname, std::vector<std::string> &groupnames);
		void listDatasets(const char *groupname, std::vector<std::string> &datasetnames);

		int getNumRows(const char *datasetname);
		int getNumCols(const char *datasetname);

    private:
        hid_t file_id;

#ifdef USE_SEMAPHORES
        sem_t globalsem;
#endif

//        int readAllAttributes(hid_t object_id, MapInt &attributes);
//        int readAllAttributes(hid_t object_id, MapDouble &attributes);
//        int readAllAttributes(hid_t object_id, MapString &attributes);
//        int readAllAttributes(hid_t object_id, MapFloat &attributes);
};


#endif

