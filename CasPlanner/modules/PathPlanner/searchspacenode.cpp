#include "searchspacenode.h"

namespace CasPlanner
{

SearchSpaceNode::SearchSpaceNode()
	{
		parent = next = NULL;
	};
SearchSpaceNode::~SearchSpaceNode()
	{
		parent = next = NULL;
	};

}
