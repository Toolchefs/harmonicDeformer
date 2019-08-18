#ifndef HARMONICDEFORMERCMD_H
#define HARMONICDEFORMERCMD_H

#include <maya/MPxCommand.h>
#include <maya/MDagPath.h>
#include <maya/MSyntax.h>
#include <maya/MArgList.h>

class HarmonicDeformerCmd : public MPxCommand
{
public:
	HarmonicDeformerCmd();
	virtual ~HarmonicDeformerCmd();

	MStatus doIt(const MArgList& args);
	static MSyntax newSyntax();

	static	void* creator();
};



#endif 