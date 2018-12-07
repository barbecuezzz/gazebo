#ifndef _BUILD_ROBOT_H_
#define _BUILD_ROBOT_H_
#include<iostream>
#include"Dense"
#include"math.h"

using namespace std;
using namespace Eigen;


typedef struct _DATA_

{
	int id;
	char name[20];
	int sister;
	int child;
	int mother;
	Vector3d p;
	Matrix3d R;
	Vector3d v;
	Vector3d w;
	double q;
	int dq;
	int ddq;
	Vector3d a;
	Vector3d b;
	int m;
	int c;
	int I;
	//Matrix3d t;
	//Matrix3d e;
	/*data::data()
	{

	}
	data::~data(void)
	{

	}*/

}Data;

// typedef struct _TARGET_
// {
// 	Vector3d p;
// 	Matrix3d R;
// }Target;
//data struct

class ListNode
{
public:
	Data data;
	ListNode * child;
	ListNode * sister;
	ListNode * CurNode;
	ListNode * head;
	/*public:*/

	ListNode()
	{
		child = NULL;
		sister = NULL;
		head = CurNode = this;
	}
	ListNode(Data newdata)
	{
		data = newdata;
		child = NULL;
		sister = NULL;
		head = CurNode = this;
	}
	/*ListNode::Data()
	{
	int Id, char Name[20], int Sister, int Child, int Mother, Vector3d P, Matrix3d r, Vector3d V, Vector3d W, int Q, int Dq, int Ddq, Vector3d A, Vector3d B, int M, int C, int i;
	};*/

	/*	ListNode FindNode(ListNode *curNode,int id);*/
	/*void CreatTree(struct data);*/
//class node


	void AppendNodeChild(ListNode Data);
	void AppendNodeSister(ListNode Data);
	void ForwardKinematics(ListNode *from);
	// void InverseKinematics(ListNode *body,ListNode *ankle, Target target);
	/*void FoundData(int id);*/
	// void IK_leg(ListNode *E, ListNode *S);
	int build_robot_model();
	// void get_joint_angle_from_ternimal_position(Vector3d P);

	/*void DispList();
	void DelList();*/
};
#endif