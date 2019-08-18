#include "harmonicDeformer.h"
#include <maya/MItGeometry.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MPoint.h>
#include <maya/MTimer.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMeshData.h>
#include <maya/MThreadUtils.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnDoubleArrayData.h>
#include <tbb/parallel_for.h>

#include "grid.h"
#include "cell.h"

#define MCheckStatus(status,message) \
if( MStatus::kSuccess != status ) { \
cerr << message << "\n"; \
return status; \
}

MTypeId HarmonicDeformer::id(0x00122C05);
MObject HarmonicDeformer::m_cage;
MObject HarmonicDeformer::m_refCage;
MObject HarmonicDeformer::m_gridData;
MObject HarmonicDeformer::m_cellSize;
MObject HarmonicDeformer::m_maxIteration;
MObject HarmonicDeformer::m_pointWeights;
MObject HarmonicDeformer::m_threshold;
MObject HarmonicDeformer::m_dynamicBinding;

const char* harmonicDeformerTemplate = "\
proc string AEgetNode(string $nodeAttr)\
{\
	string $result = \"\";\
	string $buffer[];\
	\
	tokenize($nodeAttr, \".\", $buffer);\
	\
	if (size($buffer) == 3)\
		$result = $buffer[0] + \".\" + $buffer[1];\
	else\
		$result = $buffer[0];\
	\
	return $result;\
}\
\
global proc AEcomputeHarmonicWeightsButton(string $attr) \
{\
	string $node = AEgetNode($attr + \"t\");\
	button -label \"Compute harmonic weights\" -command(\"AEcomputeHarmonicWeights \" + $node) computeHarmonicWeightsButton;\
}\
\
global proc AEcomputeHarmonicWeightsButtonUpdate(string $attr)\
{\
	string $node = AEgetNode($attr + \"t\");\
	button -edit -command(\"AEcomputeHarmonicWeights \" + $node) computeHarmonicWeightsButton;\
}\
\
global proc AEcomputeHarmonicWeights(string $node)\
{\
	float $cellSize = `getAttr ($node+\".cellSize\")`;\
	float $threshold = `getAttr ($node+\".threshold\")`;\
	int $iterations = `getAttr ($node+\".maxIterations\")`;\
	int $dynamicBinding = `getAttr ($node+\".dynamicBinding\")`;\
	if($dynamicBinding)\
	{\
		tcComputeHarmonicWeights -d $node -mi $iterations -cs $cellSize -ts $threshold -sg 1;\
	}\
	else\
	{\
		tcComputeHarmonicWeights -d $node -mi $iterations -cs $cellSize -ts $threshold;\
	}\
}\
\
\
global proc AEtcHarmonicDeformerTemplate(string $nodeName)\
{\
	editorTemplate -beginScrollLayout;\
	\
	editorTemplate -beginLayout \"Binding attributes\" -collapse 0;\
	editorTemplate -addControl \"cellSize\";\
	editorTemplate -addControl \"maxIterations\";\
	editorTemplate -addControl \"threshold\";\
	editorTemplate -addControl \"dynamicBinding\";\
	editorTemplate -callCustom \"AEcomputeHarmonicWeightsButton\" \"AEcomputeHarmonicWeightsButtonUpdate\" \"\";\
	\
	editorTemplate -endLayout;\
	\
	AEdependNodeTemplate $nodeName;\
	\
	editorTemplate -addExtraControls; \
	editorTemplate -endScrollLayout; \
}";

class ParallelFor
{
public:
	ParallelFor(std::vector<std::map<unsigned int, double> >& weights,
				MPointArray& cagePoints,
				MPointArray& cageRefPoints,
				MPointArray& verts,
				double envelope) :
		m_weights(weights),
		m_cagePoints(cagePoints),
		m_cageRefPoints(cageRefPoints),
		m_verts(verts),
		m_envelope(envelope)
	{}

	~ParallelFor() {}

	void operator()(const tbb::blocked_range<size_t>& range) const
	{
		for (size_t i = range.begin(); i != range.end(); ++i)
		{
			for (std::map<unsigned int, double>::iterator it = m_weights[i].begin(); it != m_weights[i].end(); ++it)
			{
				MVector diffPos = m_cagePoints[it->first] - m_cageRefPoints[it->first];
				diffPos *= it->second * m_envelope;
				m_verts[i] += diffPos;
			}
		}
	}

private:
	std::vector<std::map<unsigned int, double> >& m_weights;
	MPointArray& m_cagePoints;
	MPointArray& m_cageRefPoints;
	MPointArray& m_verts;
	const double m_envelope;
};

HarmonicDeformer::HarmonicDeformer() : m_toBind(true), m_weightsUpdated(false), m_gridUpdated(false) {}

HarmonicDeformer::~HarmonicDeformer() {}

void HarmonicDeformer::postConstructor()
{
	MGlobal::executeCommand(harmonicDeformerTemplate, false, false);
}

void* HarmonicDeformer::creator()
{
	return new HarmonicDeformer();
}
MStatus HarmonicDeformer::initialize()
{
	// local attribute initialization
	MStatus status;
	MFnTypedAttribute mAttr;
	MFnNumericAttribute nAttr;
	m_cage = mAttr.create("cage", "cg", MFnMeshData::kMesh);
	mAttr.setStorable(true);

	m_refCage = mAttr.create("refCage", "rcg", MFnMeshData::kMesh);
	mAttr.setStorable(true);

	m_gridData = mAttr.create("gridData", "gd", MFnData::kString);
	mAttr.setStorable(true);
	mAttr.setWritable(true);
	mAttr.setHidden(true);
	
	m_cellSize = nAttr.create("cellSize", "cs", MFnNumericData::kDouble, 1.0);
	nAttr.setStorable(true);
	nAttr.setWritable(true);

	m_maxIteration = nAttr.create("maxIterations", "mi", MFnNumericData::kLong, 20);
	nAttr.setStorable(true);
	nAttr.setWritable(true);

	m_threshold = nAttr.create("threshold", "ts", MFnNumericData::kDouble, 0.00001);
	nAttr.setStorable(true);
	nAttr.setWritable(true);

	m_dynamicBinding = nAttr.create("dynamicBinding", "db", MFnNumericData::kBoolean, false);
	nAttr.setStorable(true);
	nAttr.setWritable(true);

	m_pointWeights = mAttr.create("pointWeights", "pw", MFnData::kDoubleArray);
	mAttr.setStorable(true);
	mAttr.setWritable(true);
	mAttr.setHidden(true);


	// deformation attributes
	status = addAttribute(m_cellSize); MCheckStatus(status, "ERROR in addAttribute m_cellSize\n");
	status = addAttribute(m_maxIteration); MCheckStatus(status, "ERROR in addAttribute m_maxIteration\n");
	status = addAttribute(m_threshold); MCheckStatus(status, "ERROR in addAttribute m_threshold\n");
	status = addAttribute(m_dynamicBinding); MCheckStatus(status, "ERROR in addAttribute m_dynamicBinding\n");
	status = addAttribute(m_pointWeights); MCheckStatus(status, "ERROR in addAttribute m_pointWeights\n");
	status = addAttribute(m_cage); MCheckStatus(status, "ERROR in addAttribute m_cage\n");
	status = addAttribute(m_refCage); MCheckStatus(status, "ERROR in addAttribute m_refCage\n");
	status = addAttribute(m_gridData); MCheckStatus(status, "ERROR in addAttribute m_gridData\n");
	status = attributeAffects(m_cage, outputGeom); MCheckStatus(status, "ERROR in attributeAffects m_cage\n");
	status = attributeAffects(m_refCage, outputGeom); MCheckStatus(status, "ERROR in attributeAffects m_refCage\n");
	status = attributeAffects(m_gridData, outputGeom); MCheckStatus(status, "ERROR in attributeAffects m_gridData\n");
	status = attributeAffects(m_pointWeights, outputGeom); MCheckStatus(status, "ERROR in attributeAffects m_pointWeights\n");
	return MStatus::kSuccess;
}

MStatus HarmonicDeformer::setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs)
{
	
	MObject thisNode = this->thisMObject();
	MPlug gridPlug(thisNode, m_gridData);
	if (plugBeingDirtied == gridPlug)
		m_gridUpdated = true;

	MPlug pointsWeightPlug(thisNode, m_pointWeights);
	if (plugBeingDirtied == pointsWeightPlug)
		m_weightsUpdated = true;

	MPlug inPlug(thisNode, input);
	if ((plugBeingDirtied == inPlug) || (plugBeingDirtied.array() == inPlug) || (plugBeingDirtied.parent().array() == inPlug))
		m_toBind = true;
	//std::cout << plugBeingDirtied.name() << std::endl;
	return MPxNode::setDependentsDirty(plugBeingDirtied, affectedPlugs);
}


MStatus HarmonicDeformer::compute(const MPlug& plug, MDataBlock& data)
{
	// do this if we are using an OpenMP implementation that is not the same as Maya's.
	// Even if it is the same, it does no harm to make this call.
	MThreadUtils::syncNumOpenMPThreads();
	MStatus status = MStatus::kUnknownParameter;

	if (plug.attribute() != outputGeom) {
		return status;
	}

	MDataHandle dynBindHandle = data.inputValue(m_dynamicBinding, &status);
	bool dynBind = dynBindHandle.asBool();

	MDataHandle envelopHandle = data.inputValue(envelope, &status);
	double envelop = envelopHandle.asFloat();

	unsigned int index = plug.logicalIndex();
	MObject thisNode = this->thisMObject();
	// get input value
	MPlug inPlug(thisNode, input);
	inPlug.selectAncestorLogicalIndex(index, input);
	MDataHandle hInput = data.inputValue(inPlug, &status);
	MCheckStatus(status, "ERROR getting input mesh\n");
	// get the input geometry
	MDataHandle inputData = hInput.child(inputGeom);
	if (inputData.type() != MFnData::kMesh) {
		printf("Incorrect input geometry type\n");
		return MStatus::kFailure;
	}
	// get the input groupId - ignored for now...
	MDataHandle hGroup = inputData.child(groupId);
	unsigned int groupId = hGroup.asLong();

	// get ref cage mesh
	MDataHandle refCageData = data.inputValue(m_refCage, &status);
	MCheckStatus(status, "ERROR getting ref cage mesh\n");
	if (refCageData.type() != MFnData::kMesh) {
		printf("Incorrect deformer geometry type %d\n", refCageData.type());
		return MStatus::kFailure;
	}
	MObject refCageObj = refCageData.asMeshTransformed();
	MFnMesh fnRefCacgeMesh(refCageObj);
	MPointArray cageRefPoints;
	fnRefCacgeMesh.getPoints(cageRefPoints);


	// get cage mesh
	MDataHandle cageData = data.inputValue(m_cage, &status);
	MCheckStatus(status, "ERROR getting cage mesh\n");
	if (cageData.type() != MFnData::kMesh) {
		printf("Incorrect deformer geometry type %d\n", cageData.type());
		return MStatus::kFailure;
	}
	MObject cageObj = cageData.asMeshTransformed();
	MFnMesh fnCageMesh(cageObj);
	MPointArray cagePoints;
	fnCageMesh.getPoints(cagePoints, MSpace::kWorld);

	MDataHandle outputData = data.outputValue(plug);
	outputData.copy(inputData);
	if (outputData.type() != MFnData::kMesh) {
		printf("Incorrect output mesh type\n");
		return MStatus::kFailure;
	}
	MItGeometry iter(outputData, groupId, false);

	// get all points at once. Faster to query, and also better for
	// threading than using iterator
	MPointArray verts;
	iter.allPositions(verts, MSpace::kWorld);
	int nPoints = verts.length();


	//if (dynBind && ((m_toBind) || (m_weights.size() != nPoints)))
	if (dynBind && m_gridUpdated)
	{
		m_weights.clear();
		MDataHandle gridDataString = data.inputValue(m_gridData, &status);
		MString gridDataStr = gridDataString.asString();
		tc::Grid grid(1.0);
		if (grid.deserialise(gridDataStr.asChar()))
		{
			std::vector<tc::Vector> points;
			points.resize(nPoints);
			for (unsigned int v = 0; v < nPoints; ++v)
			{
				points[v] = tc::Vector(verts[v].x, verts[v].y, verts[v].z);
			}
			m_weights = grid.getWeights(points);
			/*
			m_weights.resize(nPoints);
			for (unsigned int v = 0; v < nPoints; ++v)
			{
				const tc::Cell& cell = grid.getCell(tc::Vector(verts[v].x, verts[v].y, verts[v].z));

				std::map<unsigned int, double> weights = cell.weights;
				for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
				{
					it->second = grid.getWeight(tc::Vector(verts[v].x, verts[v].y, verts[v].z), it->first);
				}

				//normalize weights
				double totalWeight = 0.0;
				for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
					totalWeight += it->second;

				for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
					it->second = it->second / totalWeight;

				m_weights[v] = weights;
			}
			*/
		}
		m_gridUpdated = false;
	}


	if ((m_weightsUpdated || (m_weights.size() != nPoints)) && (!dynBind))
	{
		MDataHandle weightsHandle = data.inputValue(m_pointWeights, &status);
		MFnDoubleArrayData weightsData(weightsHandle.data(), &status);
		if (status != MS::kSuccess)
		{
			MGlobal::displayError("Invalid weights data");
			return MS::kFailure;
		}

		MDoubleArray wData = weightsData.array();
		if (wData.length() == 0)
		{
			MGlobal::displayError("Invalid number of point weights");
			return MS::kFailure;
		}

		unsigned long counter = 0;
		int numPoints = wData[counter++];
		if (nPoints != numPoints)
		{
			MGlobal::displayError("Invalid number of point weights stored, please compute again the harmonic weights");
			return MS::kFailure;
		}

		m_weights.resize(numPoints);
		unsigned int idxTmp = 0;
		for (unsigned int ii = 0; ii < numPoints; ++ii)
		{
			unsigned int numWeights = wData[counter++];
			for (unsigned int jj = 0; jj < numWeights; ++jj)
            {
                idxTmp = static_cast<unsigned int>(wData[counter++]);
				m_weights[ii][idxTmp] = wData[counter++];
            }
		}

		m_weightsUpdated = false;
	}


	if (m_weights.size() != nPoints)
	{
		MGlobal::displayError("invalid numebr of weights " + m_weights.size());
		return MS::kFailure;
	}

	ParallelFor parallelData(m_weights, cagePoints, cageRefPoints, verts, envelop);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, nPoints), parallelData);
	/*
	for (int i = 0; i<nPoints; i++)
	{
		for (std::map<unsigned int, double>::iterator it = m_weights[i].begin(); it != m_weights[i].end(); ++it)
		{
			MVector diffPos = cagePoints[it->first] - cageRefPoints[it->first];
			diffPos *= it->second;
			verts[i] += diffPos;
		}
	}
	*/
	iter.setAllPositions(verts, MSpace::kWorld);
	return status;
}