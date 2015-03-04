#include "rrt.h"
#include <conio.h>
using namespace std;

vector<pair<double, double> > rRandTree(vector<pair<double, double> >& obstacles, pair<double, double>& _current, pair<double, double>& _end)
{	
	vector<node*> total_nodes, true_path;
	vector<pair<double, double> > rslt_path;
	node* start = new node;
	start->x = _current.first; start->y = _current.second;
	node* goal = new node;
	goal->x = _end.first; goal->y = _end.second;
	node* node_ptr = start;
	total_nodes.push_back(node_ptr);
    //srand(time(NULL));
	_cprintf("隨機樹開始成長...\n");
	while(1)
    {
        if(t_connect(total_nodes, goal, obstacles))
            break;	//reach the goal, break the loop
		/*
        double x = rand() % 1400 + 1; 
        double y = rand() % 1400 + 1;
        x /= 100; y /= 100;
		*/
		//區域RRT
		double x = rand() % 1400 + 1; 
		double y = rand() % 1400 + 1;
		//make the random point surround the vehicle
		x /= 100; y /= 100;
		x -= 7; y-= 7; 
		x += _current.first; y += _current.second;
        node_ptr = new node;
        node_ptr -> x = x; node_ptr -> y = y;
        //connect
        if(!t_connect(total_nodes, node_ptr, obstacles))
            delete node_ptr;
    }
	_cprintf("成長完畢!\n");
	true_path = tracePath(start, goal);
	if(true_path.size()>2)
    {
		_cprintf("開始路徑縮短...\n");
        int cnt = 500;
        for(int i = 0; i < cnt; ++i)
        {
            pathShorten(true_path, total_nodes, obstacles); //add two nodes as shortcut if they exist
            true_path = tracePath(start, goal);	// tracePath must be called after calling pathShorten
        }
    }
	_cprintf("縮短完畢!\n");
	_cprintf("刪除多餘路徑點...\n");
	true_path =  killRedund(true_path, 2.0);
	_cprintf("刪除完畢!\n");
	for(unsigned int i = 0; i < true_path.size(); ++i)
	{
		pair<double, double> _p = make_pair(true_path[i]->x, true_path[i]->y);
		rslt_path.push_back(_p);
	}
	reverse(rslt_path.begin(), rslt_path.end());
	clearAll(total_nodes); //clearAll must be called after all processes done 
	return rslt_path;
}

bool compPair(pair<node*, double> p1, pair<node*, double> p2) {return p1.second < p2.second;}

bool nearSide(double x1, double y1, double x2, double y2, double space)
{
	double dst = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	dst = sqrt(dst);
	return dst < space;
}

bool collisionCheck(node* p1, node* p2, vector<pair<double, double> >& obs)
{	//return 1 if no collision occurrs
	node tmp; 
	tmp.x = p1->x; tmp.y = p1->y;
	double dst = (p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y);
	dst = sqrt(dst);
	while( !nearSide(tmp.x, tmp.y, p2->x, p2->y, 0.01) )
	{
        for(unsigned int i = 0; i < obs.size(); ++i)
		{
			if(nearSide(tmp.x, tmp.y, obs[i].first, obs[i].second, 1.5))
				return 0;
		}
		tmp.x += 0.02 * (p2->x - p1->x) / dst;
		tmp.y += 0.02 * (p2->y - p1->y) / dst;
	}
	return 1;
}

node* interSecCalc(node* p1, node* p2, node* p3)	//p1 is the start, p2 is the end, p3 is the random point
{
	double u = (p3->x - p1->x) * (p2->x - p1->x) + (p3->y - p1->y) * (p2->y - p1->y);
	u /= ( (p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y) );
	//PIntE is the intersection of newPnt and tree's edge
	node* PIntE = new node; 
	PIntE->x = p1->x + u * (p2->x - p1->x);
	PIntE->y = p1->y + u * (p2->y - p1->y);
	return PIntE;
}

double d2edgeCalc(node* p1, node* p2, node* p3)	//p1 is the start, p2 is the end, p3 is the random point
{
	node* PIntE = interSecCalc(p1, p2, p3); 
	//check whether the intersection lies between p1 and p2 
	double dist1 = (PIntE->x - p1->x) * (PIntE->x - p1->x)
						+ (PIntE->y - p1->y) * (PIntE->y - p1->y);
	dist1 = sqrt(dist1);
	double dist2 = (PIntE->x - p2->x) * (PIntE->x - p2->x)
						+ (PIntE->y - p2->y) * (PIntE->y - p2->y);
	dist2 = sqrt(dist2);
	double dist_p1p2 = (p1->x - p2->x) * (p1->x - p2->x)
							+ (p1->y - p2->y) * (p1->y - p2->y);
	dist_p1p2 = sqrt(dist_p1p2);
    delete PIntE;
	if(dist1 < dist_p1p2 && dist2 < dist_p1p2)
	{
		//intersection point lies between two nodes
		double rslt = (PIntE->x - p3->x) * (PIntE->x - p3->x) + (PIntE->y - p3->y) * (PIntE->y - p3->y);
		rslt = sqrt(rslt);
		return rslt;
	}else
		return 10000;
}

bool t_connect(vector<node*>& total, node* Xrand, vector<pair<double, double> >& obs)
{
	if(total.size() == 1)
	{
		if( collisionCheck(total[0], Xrand, obs) )
		{
			Xrand->parent = total[0];
			total.push_back(Xrand);
			return 1;
		}else
			return 0;
	}
	//calculate distance between n and each node of the tree
    vector<pair<node*, double> > d2node;
    for(unsigned int i = 0; i < total.size(); ++i)
	{ 
		double x = Xrand -> x - total[i]->x;
		double y = Xrand -> y - total[i]->y;
		x *= x; y *= y;
		pair<node*, double> p1 = make_pair(total[i], sqrt(x + y));
		d2node.push_back(p1);
	}
	//calculate distance between n and each edge of the tree
    vector<pair<node*, double> > d2edge;
    for(unsigned int ii = 1; ii < total.size(); ++ii)
	{
		pair<node*, double> p_edge; p_edge.first = total[ii];
		p_edge.second = d2edgeCalc(total[ii], total[ii]->parent, Xrand);
		d2edge.push_back(p_edge);
	}
	//find the minimum values among each d2node and d2edge
	pair<node*, double>  min_d2node = *min_element(d2node.begin(), d2node.end(), compPair);
	pair<node*, double> min_d2edge = *min_element(d2edge.begin(), d2edge.end(), compPair);
	if(min_d2node.second <= min_d2edge.second)
	{
		if(collisionCheck(Xrand, min_d2node.first, obs))
		{
			Xrand->parent = min_d2node.first;
			total.push_back(Xrand);
			return 1;
		}else
			return 0;
	}else{
		node* PIntE = interSecCalc(min_d2edge.first, min_d2edge.first->parent, Xrand);
		if(collisionCheck(Xrand, PIntE, obs))
		{
			PIntE->parent = min_d2edge.first->parent;
			Xrand->parent = PIntE;
			min_d2edge.first->parent = PIntE;
			total.push_back(PIntE);
			total.push_back(Xrand);
			return 1;
		}else
		{
			delete PIntE;
			return 0;
		}
	}
}

vector<node*> tracePath(node* start, node* goal)
{
    //trace the true path
    vector<node*> true_path;
    node* tmp = goal;
    while(tmp->x != start->x || tmp->y != start->y)
    {
        true_path.push_back(tmp);
        tmp = tmp->parent;
    }
    true_path.push_back(tmp);
    return true_path;
}

node* randPoint(node* n1)
{
    //select a point between n1 and its parent randomly
    node* n2 = n1->parent;
    double len = (n1->x - n2->x) * (n1->x - n2->x) +
                        (n1->y - n2->y) * (n1->y - n2->y);
    len = sqrt(len);
    if(len < 0.2)
        return NULL;
    len *= 10;
    double coeff = rand() % (int)len + 1;
	if(coeff == len)
        --coeff; 
    coeff /= 10; len /= 10;
    node* rand_node = new node;
    rand_node->x = n1->x + coeff * (n2->x - n1->x) / len;
    rand_node->y = n1->y + coeff * (n2->y - n1->y) / len;
    return rand_node;
}

pair<int, int> randIndex(vector<node*>& path)
{
    while(1)
    {
        int i1 = rand() % (path.size() - 1);
        int i2 = rand() % (path.size() - 1);
        if(i1 == i2)
            continue;
        //check if i1 is larger than i2
        if(i1 > i2)
            swap(i1, i2);
        return make_pair(i1, i2);
    }
}

void pathShorten(vector<node*>& path, vector<node*>& tot,vector<pair<double, double> > &obs)
{
    //select two segment randomly
    pair<int, int> index = randIndex(path);
    int i1 = index.first; int i2 = index.second;
    node* n1; node* n2;
    n1 = randPoint(path[i1]); n2 = randPoint(path[i2]);
    if(n1 == NULL || n2 == NULL)
	{
		if(n1 != NULL)
            delete n1;
        if(n2 != NULL)
            delete n2;
        return;
	}
    double n1n2 = (n1->x - n2->x)*(n1->x - n2->x) + (n1->y - n2->y)*(n1->y - n2->y);
    n1n2 = sqrt(n1n2);
    //if(n1n2 < 0.5)
    //    return;
    if(collisionCheck(n1, n2, obs))
    {
        //insert these two node
        path[i1]->parent = n1;
        n1->parent = n2;
        n2->parent = path[i2]->parent;
        path.push_back(n1);
        path.push_back(n2);
        //add them to total nodes
        tot.push_back(n1);
        tot.push_back(n2);
    }else
    { delete n1; delete n2; }
}

void clearAll(vector<node*>& total_nodes)
{
    if(total_nodes.size() > 0)
    {
        for(unsigned int i = 0; i < total_nodes.size(); ++i)
            delete total_nodes[i];
    }
}

vector<node*> killRedund(vector<node*> path, double step)
{
    if(path.size() < 3)
        return path;
    vector<node*> rslt_path;
    rslt_path.push_back(path[0]);
    for(unsigned int i = 0; i < path.size() - 1; ++i)
    {
        node* tmp = path[i+1];
        double dist = (tmp->x - path[i]->x) * (tmp->x - path[i]->x) +
                (tmp->y - path[i]->y) * (tmp->y - path[i]->y);
        dist = sqrt(dist);
        if(dist < step)
        {
            bool add = 0;
            for(unsigned int ii = 2; i + ii < path.size(); ++ii)
            {
                tmp = path[i+ii];
                dist = (tmp->x - path[i]->x) * (tmp->x - path[i]->x) +
                                (tmp->y - path[i]->y) * (tmp->y - path[i]->y);
                dist = sqrt(dist);
                if(dist > step)
                {
                    path[i]->parent = path[i+ii];
                    rslt_path.push_back(path[i+ii]);
                    //i += ii - 1;//cancel the increment in the next turn within for loop
                    i += ii; --i;
					add = 1;
                    break;
                }
            }
            if(!add)
            {
                rslt_path.pop_back();
                path[i-1]->parent = path[path.size()-1];
                rslt_path.push_back(path[path.size()-1]);
            }
        }else
            rslt_path.push_back(path[i+1]);
    }
    return rslt_path;
}
