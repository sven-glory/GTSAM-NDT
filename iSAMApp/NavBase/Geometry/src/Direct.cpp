//                        - DIRECT.CPP -
//
//   Implementation of classes "CMoveDir" and "CTurnDir".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CMoveDir".

//
//   Copy constructor =: Assignment of move direction.
//
void CMoveDir::operator =(CMoveDir& MoveDir)
{
	m_tagMoveDir = MoveDir.m_tagMoveDir;
}

//
//   Overloaded operator =: Assignment of move direction.
//
void CMoveDir::operator =(MoveDirTag tagMoveDir)
{
	m_tagMoveDir = tagMoveDir;
}

//
//   overloaded operator ==: Test if 2 objects are identical.
//
BOOL CMoveDir::operator ==(CMoveDir& MoveDir)
{
	return (m_tagMoveDir == MoveDir.m_tagMoveDir);
}

//
//   overloaded operator !=: Test if 2 objects are not identical.
//
BOOL CMoveDir::operator !=(CMoveDir& MoveDir)
{
	return (m_tagMoveDir != MoveDir.m_tagMoveDir);
}

//
//   Overloaded operator ==: Test if the object is of the specified turn
//   direction.
//
BOOL CMoveDir::operator ==(MoveDirTag tagMoveDir)
{
	return (m_tagMoveDir == tagMoveDir);
}

//
//   Overloaded operator !=: Test if the object is not of the specified
//   turn direction.
//
BOOL CMoveDir::operator !=(MoveDirTag tagMoveDir)
{
	return (m_tagMoveDir != tagMoveDir);
}

//
//   Overloaded operator !: Get the opposite turn direction.
//
CMoveDir CMoveDir::operator !()
{
	if (m_tagMoveDir == FORWARD)
		return CMoveDir(BACKWARD);
	else
		return CMoveDir(FORWARD);
}


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTurnDir".

//
//   Copy constructor =: Assignment of turn direction.
//
void CTurnDir::operator =(CTurnDir TurnDir)
{
	m_tagTurnDir = TurnDir.m_tagTurnDir;
}

//
//   Overloaded operator =: Assignment of turn direction.
//
void CTurnDir::operator =(TurnDirTag tagTurnDir)
{
	m_tagTurnDir = tagTurnDir;
}

//
//   overloaded operator ==: Test if 2 objects are identical.
//
BOOL CTurnDir::operator ==(CTurnDir& TurnDir)
{
	return (m_tagTurnDir == TurnDir.m_tagTurnDir);
}

//
//   overloaded operator !=: Test if 2 objects are not identical.
//
BOOL CTurnDir::operator !=(CTurnDir& TurnDir)
{
	return (m_tagTurnDir != TurnDir.m_tagTurnDir);
}

//
//   Overloaded operator ==: Test if the object is of the specified turn
//   direction.
//
BOOL CTurnDir::operator ==(TurnDirTag tagTurnDir)
{
	return (m_tagTurnDir == tagTurnDir);
}

//
//   Overloaded operator !=: Test if the object is not of the specified
//   turn direction.
//
BOOL CTurnDir::operator !=(TurnDirTag tagTurnDir)
{
	return (m_tagTurnDir != tagTurnDir);
}

//
//   Overloaded operator !: Get the opposite turn direction.
//
CTurnDir CTurnDir::operator !()
{
	if (m_tagTurnDir == COUNTER_CLOCKWISE)
		return CTurnDir(CLOCKWISE);
	else
		return CTurnDir(COUNTER_CLOCKWISE);
}
