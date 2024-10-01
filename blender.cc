#include "blender.h"

using namespace std;


// create functions in python for loading ipo curves from a file
void blenderDefLoadIpo(ofstream &ofs) {
	ofs << "#load IPO from given file for a given object\n";
	ofs << "def loadIpoFromFile(filename, objectname):\n";
	ofs << "    obj = Blender.Object.Get(objectname);\n";
	ofs << "    ipo = Ipo.New('Object','robotIpo');\n";
	ofs << "    ipo.addCurve('LocX');\n";
	ofs << "    ipo.addCurve('LocY');\n";
	ofs << "    ipo.addCurve('LocZ');\n";
	ofs << "    ipo.addCurve('RotX');\n";
	ofs << "    ipo.addCurve('RotY');\n";
	ofs << "    ipo.addCurve('RotZ');\n";
	ofs << "    x = ipo[Ipo.OB_LOCX];\n";
	ofs << "    y = ipo[Ipo.OB_LOCY];\n";
	ofs << "    z = ipo[Ipo.OB_LOCZ];\n";
	ofs << "    rx = ipo[Ipo.OB_ROTX];\n";
	ofs << "    ry = ipo[Ipo.OB_ROTY];\n";
	ofs << "    rz = ipo[Ipo.OB_ROTZ];\n";
	ofs << "    x.setInterpolation('Constant')\n";
	ofs << "    y.setInterpolation('Constant')\n";
	ofs << "    z.setInterpolation('Constant')\n";
	ofs << "    rx.setInterpolation('Constant')\n";
	ofs << "    ry.setInterpolation('Constant')\n";
	ofs << "    rz.setInterpolation('Constant')\n";
	ofs << "    fin = open(filename,'r')\n";
	ofs << "    i = 0\n";
	ofs << "    for line in fin:\n";
	ofs << "        nums = [ float(xx) for xx in line.split()]\n";
	ofs << "        x[i] = nums[0]; y[i]=nums[1]; z[i]=nums[2];i+=1\n";
	ofs << "    obj.setIpo(ipo);\n";	
	ofs << "    print \"Loaded \",i,\" ipo segments for \",objectname,\".\"\n\n\n";
}

// create functions in python for loading ipo curves from a file
void blenderIpoPrepare(ofstream &ofs) {
	ofs << "#load IPO from given file for a given object\n";
	ofs << "obj = Blender.Object.Get(objectname);\n";
	ofs << "ipo = Ipo.New('Object','robotIpo');\n";
	ofs << "ipo.addCurve('LocX');\n";
	ofs << "ipo.addCurve('LocY');\n";
	ofs << "ipo.addCurve('LocZ');\n";
	ofs << "ipo.addCurve('RotX');\n";
	ofs << "ipo.addCurve('RotY');\n";
	ofs << "ipo.addCurve('RotZ');\n";
	ofs << "x = ipo[Ipo.OB_LOCX];\n";
	ofs << "y = ipo[Ipo.OB_LOCY];\n";
	ofs << "z = ipo[Ipo.OB_LOCZ];\n";
	ofs << "rx = ipo[Ipo.OB_ROTX];\n";
	ofs << "ry = ipo[Ipo.OB_ROTY];\n";
	ofs << "rz = ipo[Ipo.OB_ROTZ];\n";
	ofs << "x.setInterpolation('Constant')\n";
	ofs << "y.setInterpolation('Constant')\n";
	ofs << "z.setInterpolation('Constant')\n";
	ofs << "rx.setInterpolation('Constant')\n";
	ofs << "ry.setInterpolation('Constant')\n";
	ofs << "rz.setInterpolation('Constant')\n";
	ofs << "print \"Loaded \",i,\" ipo segments for \",objectname,\".\"\n\n\n";
}






