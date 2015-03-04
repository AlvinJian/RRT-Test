#ifndef _RRT_H
#define _RRT_H
#include <vector>
#include <algorithm>
#include <utility>
#include <cmath>
#include <cstdlib>
#include <ctime>
using std::vector;
using std::pair;

 struct node{
	node* parent;
	double x, y;
};

bool compPair(pair<node*, double> p1, pair<node*, double> p2);
bool nearSide(double x1, double y1, double x2, double y2, double space);
bool collisionCheck(node* p1, node* p2, vector< pair<double, double> >& obs);
node* interSecCalc(node* p1, node* p2, node* p3);
double d2edgeCalc(node* p1, node* p2, node* p3);
bool t_connect(vector<node*>& total, node* Xrand, vector< pair<double, double> >& obs);
vector<node*> tracePath(node *start, node *goal);
node* randPoint(node* n1);
pair<int, int> randIndex(vector<node*>& path);
void pathShorten(vector<node*>& path, vector<node *> &tot, vector<pair<double, double> > &obs);
void clearAll(vector<node*>& total_nodes);
vector<node *> killRedund(vector<node *> path, double step);
#endif
