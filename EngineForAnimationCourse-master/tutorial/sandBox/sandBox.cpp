#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>

using namespace std;
using namespace Eigen;
using namespace igl;

double distanceVec = 1;
Vector3d destinationCord(0, 0, 0);
Vector3d currentVec;
Vector3d destVec;

SandBox::SandBox()
{


}

void SandBox::Init(const std::string& config)
{
	std::string item_name;
	
	std::ifstream nameFileout;
	doubleVariable = 0;
	
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file" << config << std::endl;
	}
	else
	{

		int i = 0;
		while (nameFileout >> item_name)
		{
			RowVector3d center(0, 0, -0.8);
			vector<set<int>> OVtemp;
			std::cout << "openning " << item_name << std::endl;
			if (item_name == "C:/Users/RDF637/Desktop/EngineForAnimationCourse-master/tutorial/data/zCylinder.obj")
			{
				for (size_t i = 0; i < numOfzCylinder; i++)
				{
					parents.push_back(-1);
					load_mesh_from_file(item_name);
					data().set_visible(false, 1);
					data().show_overlay_depth = true;
					data().point_size = 10;
					data().line_width = 2;
					data().SetCenterOfRotation(center.transpose());
				}
			}
			else
			{
				shpereIndex = indexShape;
				parents.push_back(-1);
				load_mesh_from_file(item_name);
				data().show_overlay_depth = true;
				data().point_size = 10;
				data().line_width = 2;
				data().set_visible(false, 1);
				data().MyTranslate(Vector3d(5, 0, 0), true);
				updateDistention();
			}

		}
		nameFileout.close();
	}
	for (size_t i = 1; i < data_list.size() - 1; i++)
	{
		parents[i + 1] = i;
		data_list[i + 1].MyTranslate(Eigen::Vector3d(0, 0, 1.6), true);
	}

	for (size_t i = 1; i < data_list.size()+1; i++)
	{

		shapeInfo shapeI;
		shapeI.linkNumber = i;
		shapeinfoVector.push_back(shapeI);
	}
	calcTips();
	data().invert_normals = true;
	for (size_t i = 0; i < shapeinfoVector.size(); i++)
	{
		vecTipForCalc.push_back(shapeinfoVector.at(i).tipCord);
	}


}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{	
	if (isActive)
	{

		//reset vecTipForCalc
		if ((shapeinfoVector.at(0).tipCord - shepreCord).norm() <= 1.6 * numOfzCylinder)
		{
		for (int i = 0; i <= numOfzCylinder - 1; i++)
		{
			vecTipForCalc.at(i) = shapeinfoVector.at(i).tipCord;
		}
		if (distanceVec >= 0.1)
		{
			//fabrik
			//forward
			vecTipForCalc.at(0) = startCord;
			for (int i = 0; i < numOfzCylinder; i++)
			{
				double ri = (vecTipForCalc.at(i) - vecTipForCalc.at(i + 1)).norm();
				double offset = 1.6 / ri;
				vecTipForCalc.at(i + 1) = (1 - offset) * vecTipForCalc.at(i) + vecTipForCalc.at(i + 1) * offset;
			}
			//backward
			vecTipForCalc[numOfzCylinder] = data_list[0].GetCenter();
			for (int i = numOfzCylinder - 1; i >= 0; i--)
			{
				double ri = (vecTipForCalc.at(i) - vecTipForCalc.at(i + 1)).norm();
				double offset = 1.6 / ri;
				vecTipForCalc.at(i) = (1 - offset) * vecTipForCalc.at(i + 1) + vecTipForCalc.at(i) * offset;

			}
		//rotate the object
			for (int i = 0; i < numOfzCylinder; i++)
			{
				destVec = vecTipForCalc.at(i)- vecTipForCalc.at(i + 1);
				currentVec = shapeinfoVector.at(i).tipCord - shapeinfoVector.at(i + 1).tipCord;
				Eigen::Vector4d rotVec(currentVec.cross(destVec)(0), currentVec.cross(destVec)(1), currentVec.cross(destVec)(2), 0);
				data_list.at(i + 1).MyRotate(((CalcParentsTrans(i + 1) * data_list[i + 1].MakeTransd()).inverse() * rotVec).head(3), returnAngle(currentVec, destVec) / 10);
				calcTips();
			}
			//calc the distatnce
			distanceVec = (shapeinfoVector[numOfzCylinder].tipCord - data_list[0].GetCenter()).norm();
		}
		else
		{
			cout << "distance: " << distanceVec << "\n";
			isActive = false;
			distanceVec = 1;

		}
	}
		else
		{
			isActive = false;
			cout << "cannot reach \n";
		}
	}


}



