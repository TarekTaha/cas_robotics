#ifndef MRFMODEL_H_
#define MRFMODEL_H_

#include "cxcore.h"
#include "pnlHigh.hpp"
#include "pnlException.hpp"
#include "pnl_dll.hpp"
#include "Tokens.hpp"

#if !defined(WIN32)
#define itoa(__val,__str,__rad) sprintf((__str), "%i", (__val))
#endif 

using namespace pnlw;
using namespace std;

class MRFModel
{
public:
	MRFModel();
	virtual ~MRFModel();
	MRF * VerySimpleMRFModel();
	MRF * SimpleMRFModel();
	MRF * GridMRFModel();
	MRF * mrfModel();
	void  TestMRFModelCreate();
	bool  bTokArrEqual(TokArr first, TokArr second, float eps);
	void  TestMRFGetJPD();
	void  TestNodeType();
	void  TestSaveLoadMRF();
	void  testEvidencesManipulationMRF();
	void  testPropertiesMRF();
	void  testLearningMRF();
	void  testPNLObjectsRequestsMRF();
	int   testMRF();
};

#endif /*MRFMODEL_H_*/
