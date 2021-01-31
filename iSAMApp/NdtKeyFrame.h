#ifndef _NDT_KEY_FRAME_
#define _NDT_KEY_FRAME_

/*
author:zjj
data:2020.12.20
modify:
*/
#include "Geometry/include/geometry.h"
#include "ndt_fuser\ndt_fuser_node.h"
#include "ndt_map\ndt_map.h"
#include "ndt_map\lazy_grid.h"

class CNdtKeyFrame
{
public:
	CPosture m_RobotPose;
	int      m_FrameID;
	perception_oru::NDTMap *ndtmap;

	CNdtKeyFrame()
	{
		m_RobotPose = CPosture(0, 0, 0);
		m_FrameID = -1;
		//ndtmap = new perception_oru::NDTMap();
	}
};

#endif // _NDT_KEY_FRAME_
