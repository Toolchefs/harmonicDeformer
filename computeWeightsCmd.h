#pragma once
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>

class ComputeWeightsCmd : public MPxCommand
{
public:
	ComputeWeightsCmd() {};

	virtual MStatus doIt(const MArgList&);

	static MSyntax newSyntax();

	static void* creator();
};