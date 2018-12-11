#include <iostream>
#include <Eigen/Dense>
#include"math.h"
#include"build_robot.h"
// #include <unsupported/Eigen/FFT>

using namespace std;
double q2, q3, q4, q5, q6, q7;
const Vector3d D(0,0.3,0);
const Matrix3d eye1 = Matrix3d::Identity(3, 3);
ListNode I1;
ListNode I2;
ListNode I3;
ListNode I4;
ListNode I5;
ListNode I6;
ListNode I7;
ListNode I8;
ListNode I9;
ListNode I10;
ListNode I11;
ListNode I12;
ListNode I13;

void ListNode::AppendNodeChild(ListNode temp)
{
	ListNode *p;
	p = temp.CurNode;
	this->child = p;
	CurNode = CurNode->child;
	CurNode->sister = NULL;
}

void ListNode::AppendNodeSister(ListNode temp)
{
	ListNode *p;
	p = temp.CurNode;
	this->sister = p;
	CurNode = CurNode->sister;
}

void ListNode::ForwardKinematics(ListNode *from)
{
	if (from != NULL)
	{

		if (from->child != NULL)
		{


			from->child->data.p = from->data.R * from->child->data.b + from->data.p;
			from->child->data.R = (from->data.R) * (Rodrigues(from->child->data.a, from->child->data.q));
			ForwardKinematics(from->sister);
			ForwardKinematics(from->child);

		}

	}
}

void ListNode::IK_leg(ListNode *E, ListNode *S)
{
	Matrix3d R;
	Vector3d r;
	
	double C;
	double c5, q6a, cz, sz;
	


	r = E->data.R.transpose() * ((S->data.p) + (S->data.R * D) - (E->data.p));

	C = sqrt(r(0)*r(0) + r(1)*r(1) + r(2)*r(2));
	c5 = (C*C - A*A - B*B) / (2 * A * B);
	if (c5 >= 1)
		q5 = 0;
	else if (c5 <= -1)
		q5 = PI;
	else
		q5 = acos(c5);
	q6a = asin((A / C) * sin(PI - q5));
	q7 = atan2(r(1), r(2));
	if (q7 > (PI / 2))
		q7 = q7 - PI;
	else if (q7 < (-PI / 2))
		q7 = q7 + PI;
	q6 = -atan2(r(0), (sign(r(2)) * sqrt(r(1)*r(1) + r(2)*r(2)))) - q6a;
	R = (S->data.R.transpose()) * (E->data.R) * Rroll((-q7)) * Rpitch((-q6 - q5));
	q2 = atan2(-R(0, 1), R(1, 1));
	cz = cos(q2);
	sz = sin(q2);
	q3 = atan2(R(2, 1), ((-R(0, 1)) * sz + R(1, 1) * cz));
	q4 = atan2(-R(2, 0), R(2, 2));
	/*q(0) = q2;
	q(1) = q3;
	q(2) = q4;
	q(3) = q5;
	q(4) = q6;
	q(5) = q7;*/
}

Matrix3d Rodrigues(Vector3d a, double q)
{
	Matrix3d temp;
	Vector3d an;
	float a_a;
	float th;
	a_a = sqrt(a(0)*a(0)+a(1)*a(1)+a(2)*a(2));
	th = a_a*q;
	an<<a(0)/a_a,a(1)/a_a,a(2)/a_a;
	temp = eye1 + (sin(th)*Mathat(an)) + ((1 - cos(th))*(Mathat(an)*Mathat(an)));
	return (temp);
}


Matrix3d Rroll(double q)
{
	Matrix3d temp;
	temp << 1, 0, 0,
		0, cos(q), -sin(q),
		0, sin(q), cos(q);
	return temp;
}

Matrix3d Rpitch(double q)
{
	Matrix3d temp;
	temp << cos(q), 0, sin(q),
		0, 1, 0,
		-sin(q), 0, cos(q);
	return temp;
}

Matrix3d Mathat(Vector3d a)
{
	Matrix3d temp1;
	temp1 << 0, (-a(2)), a(1),
		a(2), 0, (-a(0)),
		(-a(1)), a(0), 0;
	//temp1 << 0, a(2), -a(1),
	//	-a(2), 0, a(0),
	//	a(1), -a(0), 0;
	return temp1;
}

int sign(double a)
{
	if (a > 0)
		return 1;
	if (a < 0)
		return -1;
	if (a = 0)
		return 0;
}
// void ListNode::InverseKinematics(ListNode *from,ListNode *to, Target target)
// {
// 	ListNode::ForwardKinematics(from);
// 	for(int n=1;n<=10;n++)
// 	{
// 		Matrix3d J = CalcJacobian(from,to);
// 	}
// }


int build_robot_model()
{
	Vector3d v(0,0,0);

	Vector3d body_p(0,0,1.22);

	Vector3d body_b(0,0,0);
	Vector3d left_hip_joint_r_b(0,0.3,0);
	Vector3d left_hip_joint_p_b(0,0,0);
	Vector3d left_hip_joint_y_b(0,0,0);
	Vector3d left_knee_joint_p_b(0,0,-0.6);
	Vector3d left_ankle_joint_p_b(0,0,-0.55);
	Vector3d left_ankle_joint_r_b(0,0,0);
	Vector3d right_hip_joint_r_b(0,-0.3,0);
	Vector3d right_hip_joint_p_b(0,0,0);
	Vector3d right_hip_joint_y_b(0,0,0);
	Vector3d right_knee_joint_p_b(0,0,-0.6);
	Vector3d right_ankle_joint_p_b(0,0,-0.55);
	Vector3d right_ankle_joint_r_b(0,0,0);
	
	Vector3d body_a(0,0,1);
	Vector3d left_hip_joint_r_a(1,0,0);
	Vector3d left_hip_joint_p_a(0,1,0);
	Vector3d left_hip_joint_y_a(0,0,1);
	Vector3d left_knee_joint_p_a(0,1,0);
	Vector3d left_ankle_joint_p_a(0,1,0);
	Vector3d left_ankle_joint_r_a(1,0,0);
	Vector3d right_hip_joint_r_a(1,0,0);
	Vector3d right_hip_joint_p_a(0,1,0);
	Vector3d right_hip_joint_y_a(0,0,1);
	Vector3d right_knee_joint_p_a(0,1,0);
	Vector3d right_ankle_joint_p_a(0,1,0);
	Vector3d right_ankle_joint_r_a(1,0,0);
	
	
	
	Data temp1 = { 1, "body", 0, 2, 0, body_p, eye1, v, v, 0, 0, 0, body_a, body_b, 0, 0, 0 };
	I1.data = temp1;
	Data temp2 = { 2, "left_hip_joint_r", 8, 3, 1, v, eye1, v, v, 0, 0, 0, left_hip_joint_r_a, left_hip_joint_r_b, 0, 0, 0 };
	I2.data = temp2;
	Data temp3 = { 3, "left_hip_joint_p", 0, 4, 2, v, eye1, v, v, -1*ToRad, 0, 0, left_hip_joint_p_a, left_hip_joint_p_b, 0, 0, 0 };
	I3.data = temp3;
	Data temp4 = { 4, "left_hip_joint_y", 0, 5, 3, v, eye1, v, v, 0, 0, 0, left_hip_joint_y_a, left_hip_joint_y_b, 0, 0, 0 };
	I4.data = temp4;
	Data temp5 = { 5, "left_knee_joint_p", 0, 6, 4, v, eye1, v, v, 1*ToRad, 0, 0, left_knee_joint_p_a, left_knee_joint_p_b, 0, 0, 0 };
	I5.data = temp5;
	Data temp6 = { 6, "left_ankle_joint_p", 0, 7, 5, v, eye1, v, v, 0, 0, 0, left_ankle_joint_p_a, left_ankle_joint_p_b, 0, 0, 0 };
	I6.data = temp6;
	Data temp7 = { 7, "left_ankle_joint_r", 0, 0, 6, v, eye1, v, v, 0, 0, 0, left_ankle_joint_r_a, left_ankle_joint_r_b, 0, 0, 0 };
	I7.data = temp7;
	Data temp8 = { 8, "right_hip_joint_r", 0, 9, 1, v, eye1, v, v, 0, 0, 0, right_hip_joint_r_a, right_hip_joint_r_b, 0, 0, 0 };
	I8.data = temp8;
	Data temp9 = { 9, "right_hip_joint_p", 0, 10, 8, v, eye1, v, v, -1*ToRad, 0, 0, right_hip_joint_p_a, right_hip_joint_p_b, 0, 0, 0 };
	I9.data = temp9;
	Data temp10 = { 10, "right_hip_joint_y", 0, 11, 9, v, eye1, v, v, 0, 0, 0, right_hip_joint_y_a, right_hip_joint_y_b, 0, 0, 0 };
	I10.data = temp10;
	Data temp11 = { 11, "right_knee_joint_p", 0, 12, 10, v, eye1, v, v, 1*ToRad, 0, 0, right_knee_joint_p_a, right_knee_joint_p_b, 0, 0, 0 };
	I11.data = temp11;
	Data temp12 = { 12, "right_ankle_joint_p", 0, 13, 11, v, eye1, v, v, 0, 0, 0, right_ankle_joint_p_a, right_ankle_joint_p_b, 0, 0, 0 };
	I12.data = temp12;
	Data temp13 = { 13, "right_ankle_joint_r", 0, 0, 12, v, eye1, v, v, 0, 0, 0, right_ankle_joint_r_a, right_ankle_joint_r_b, 0, 0, 0 };
	I13.data = temp13;

	I1.AppendNodeChild(I2);//connect part of the model
	I2.AppendNodeChild(I3);
	I3.AppendNodeChild(I4);
	I4.AppendNodeChild(I5);
	I5.AppendNodeChild(I6);
	I6.AppendNodeChild(I7);
	I2.AppendNodeSister(I8);                        
	I8.AppendNodeChild(I9);
	I9.AppendNodeChild(I10);
	I10.AppendNodeChild(I11);
	I11.AppendNodeChild(I12);
	I12.AppendNodeChild(I13);
	I1.ForwardKinematics(&I1);
	cout<<"-------------"<<endl;
}