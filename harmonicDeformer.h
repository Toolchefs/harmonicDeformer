#pragma once

#include <maya/MPxDeformerNode.h>
#include <vector>
#include <map>

class HarmonicDeformer : public MPxDeformerNode
{
public:
	HarmonicDeformer();

	virtual ~HarmonicDeformer();

	virtual void postConstructor();

	static void* creator();

	static MStatus initialize();
	// deformation function
	//
	virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

	MStatus setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs);

public:
	// local node attributes
	static MTypeId id;

	static MObject m_refCage;

	static MObject m_cage;

	static MObject m_gridData;
	
	static MObject m_cellSize;

	static MObject m_maxIteration;

	static MObject m_pointWeights;

	static MObject m_threshold;

	static MObject m_dynamicBinding;

private:

	bool m_toBind;

	bool m_weightsUpdated;

	bool m_gridUpdated;

	std::vector<std::map<unsigned int, double> > m_weights;
};