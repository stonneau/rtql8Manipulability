#include "rtql8Manipulability/world/WorldParserObj.h"
#include "rtql8Manipulability/world/Obstacle.h"
#include "rtql8Manipulability/sampling/SampleGenerator.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

using namespace Eigen;
using namespace std;
using namespace rtql8::kinematics;
using namespace manip_core;

WorldParserObj::WorldParserObj()
{
	// NOTHING
}

WorldParserObj::~WorldParserObj()
{
	// NOTHING
}

namespace
{
string doubleSlash(const string& s)
{
    //Remplace "//" par "/1/".
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(i<s.size()-1&&s[i]=='/'&&s[i+1]=='/')
        {
            s1+="/1/";
            i++;
        }
        else
            s1+=s[i];
    }
    return s1;
}

string remplacerSlash(const string& s)
{
    //Remplace les '/' par des espaces.
    string ret="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/')
            ret+=' ';
        else
            ret+=s[i];
    }
    return ret;
}

vector<string> splitSpace(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '||i==s.size()-1)
        {
            if(i==s.size()-1)
                s1+=s[i];
            ret.push_back(s1);
            s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
}

void WorldParserObj::CreateWorld(const std::string& filename)
{
	string line;
	ifstream myfile (filename.c_str());
	std::vector<std::string> lines;
	if (myfile.is_open())
	{
		while ( myfile.good() )
		{
			getline (myfile, line);
			if(line.find("t ") == 0)
			{
				char t[255];
				sscanf(line.c_str(),"t %s",t);
				//manager_.SetNextTexture(strtod (t, NULL));
			}
			if(line.find("c ") == 0)
			{
				char r[255],g[255],b[255],t[255];
				sscanf(line.c_str(),"c %s %s %s %s",r,g,b,t);
				//manager_.SetNextColor(strtod (r, NULL), strtod(g, NULL), strtod(b, NULL));
				//manager_.SetNextTransparency(strtod (t, NULL));
			}
			if(line.find("v ") == 0)
			{
				char x[255],y[255],z[255];
                sscanf(line.c_str(),"v %s %s %s",x,y,z);
                points_.push_back(Vector3d(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("vn ") == 0)
			{
				char x[255],y[255],z[255];
                sscanf(line.c_str(),"vn %s %s %s",x,y,z);
                normals_.push_back(Vector3d(strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("f ") == 0)
			{
				lines.push_back(line);
			}
		}
		myfile.close();
		for(std::vector<std::string>::const_iterator it = lines.begin(); it != lines.end(); ++it)
		{
			CreateObstacle(*it);
		}
	}
}


namespace
{
	typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > T_Point;

	double isLeft( const VectorXd& P0, const VectorXd& P1, const Vector3d& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	const Vector3d& LeftMost(const T_Point& points)
	{
		unsigned int i = 0;
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			if(points[j].x() < points[i].x())
			{
				i = j;
			}
		}
		return points[i];
	}

	T_Point ConvexHull(const T_Point& points)
	{
		T_Point res;
		Vector3d pointOnHull = LeftMost(points);	
		Vector3d endPoint;
		int i = 0;
		do
		{
			++i;
			VectorXd pi = pointOnHull;
			endPoint = points[0];
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				if((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0))
				{
					endPoint = points[j];
                }
			}
			res.push_back(pi);
			pointOnHull = endPoint;
		} while(endPoint != res[0]);
		res.push_back(endPoint);
		return res;
	}
}

void WorldParserObj::CreateObstacle(const std::string& line)
{
	if(line.find("c ") == 0)
	{
		char r[255],g[255],b[255],t[255];
		sscanf(line.c_str(),"c %s %s %s %s",r,g,b,t);
		//manager_.SetNextColor(strtod (r, NULL), strtod(g, NULL), strtod(b, NULL));
		//manager_.SetNextTransparency(strtod (t, NULL));
		return;
	}
	vector<long int> indices;
	vector<long int> normalindices;
    T_Point points;
	for(int i =0; i<1; ++i)
	{
		string ligne = doubleSlash(line);
		ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"
		vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")
		int ndonnees=(int)termes.size()/3;
		for(int i=0; i <ndonnees;i++) //On aurait très bien pu mettre i<ndonnees mais je veux vraiment limiter à 3 ou 4
		{
			long int idx = (strtol(termes[i*3].c_str(),NULL, 10)) - 1;
			long int idn = (strtol(termes[i*3+2].c_str(),NULL, 10)) - 1;
			std::vector<long int>::iterator it = indices.begin();
			it = find (indices.begin(), indices.end(), idx);
			if(it == indices.end())
			{
				indices.push_back(idx);
				points.push_back(points_[(int)idx]);
			}
			it = find (normalindices.begin(), normalindices.end(), idn);
			if(it == normalindices.end())
			{
				normalindices.push_back(idn);
			}
		}
	}

	Vector3d p1(points[0]);
	Vector3d p2(points[1]);
	Vector3d p3(points[2]);

	Vector3d u_(p3-p1);
	Vector3d v_(p2-p1);
	if(abs( u_.dot(v_)) > 0.001) { v_ = u_; u_ = p2-p1;}
	//we need convex hull of this crap
	Vector3d normal (u_.cross(v_));

	normals_[(int)normalindices[0]];

    SampleGenerator * generator = SampleGenerator::GetInstance();
    Obstacle * obstacle;

    if(normal.dot(normals_[(int)normalindices[0]]) > 0 )
	{
        generator->AddObstacle(new Obstacle(p1, p2, p3));
	}
	else
	{
        generator->AddObstacle(new Obstacle(p3, p2, p1));
    }
}

