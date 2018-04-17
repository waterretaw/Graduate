#ifndef BASE_H_INCLUDED
#define BASE_H_INCLUDED

// OpenGL
#define GLUT_DISABLE_ATEXIT_HACK
#include <windows.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <stdlib.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <iomanip>
#include <limits>
#include <string>
#include <fstream>

#include  <io.h>

using namespace std;

// ���ȵĲ�ֵת��Ϊγ�ȵĲ�ֵ dlo ���ȸı��� dla γ�ȸı���
double dlo2dla(double dlo,double la);
// γ�ȵĲ�ֵת��Ϊ���ȵĲ�ֵ
double dla2dlo(double dla,double la);

// ¥��߶Ȳ�ֵת��Ϊγ�ȵĲ�ֵ
double dfl2dla(int floor);

const float fl2m = 3.0; // һ��¥�߶�
const float dla2m = 111000.0; // ͬһ������γ�����1�ȵľ���

class Point;
class Obstacle;
class Cube;
class Octree;
class ObsFile;
class PathPlan;

ostream &operator << (ostream &stream, Point &p);
Point operator + (Point x, Point y);
Point operator * (Point x, double n);
double getDisc(Point&,Point&); // ��������֮��ľ���
int getIndex(Point&,Point&);   // �õ�ĳ������һ��Ŀռ䷽λ��Ӧ��������

class Point
{
private:
    friend Cube;
    friend Obstacle;
    friend PathPlan;
    double x;  // longitude
    double y;  // latitude
    double z;  // altitude �߶ȴ洢��Ӧ��γ�ȸı������߶�/111km
public:
    Point():x(0),y(0),z(0) {};
    Point(double x,double y,double z):x(x),y(y),z(z) {};

    Point& operator = (Point p) // ��ֵ����
    {
        this->x = p.x;
        this->y = p.y;
        this->z = p.z;
        return *this;
    };

    void setxyz(double x,double y,double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    };

    void getxyz(double &x,double &y,double &z)
    {
        x = this->x;
        y = this->y;
        z = this->z;
    };

    // ��һ�������ڵڶ��������ķ�λ��Ӧ��������
    friend int getIndex(Point &p, Point &c)
    {
        // ����P��c��ͬ������Ĵ���
        int index=0;
        if(p.x-c.x>=0)
        {
            index+=1;
        }
        index=index<<1;
        if(p.y-c.y>=0)
        {
            index+=1;
        }
        index=index<<1;
        if(p.z-c.z>=0)
        {
            index+=1;
        }
        return index;
    };

    friend double getDisc(Point& a,Point& b) // ��������֮��ľ���
    {
        return sqrt(pow(dlo2dla(a.x-b.x,b.y),2)
                    +pow(a.y-b.y,2)
                    +pow(a.z-b.z,2));
    }

    friend Point operator * (Point x,double n)
    {
        return Point(x.x*n,x.y*n,x.z*n);
    };

    friend Point operator + (Point d, Point c)
    {
        return Point(dla2dlo(d.x+c.x,c.y),d.y+c.y,d.z+c.z);
    };

    friend ostream &operator << (ostream &stream, Point &p)
    {
        return stream<<p.x<<" "<<p.y<<" "<<p.z;
    };

};

ostream &operator << (ostream &stream, Obstacle &o);
class Obstacle
{
    // Ŀǰ�ϰ���ֻ���������崦��
private:
    friend Cube;
    friend Octree;
    friend PathPlan;
    Point llb; // lower left bottom
    Point urt; // upper right top
    double xd,yd,zd; // ÿ������ı߳�
public:
    Obstacle():llb(),urt(),xd(0),yd(0),zd(0) {};
    Obstacle(Point l,Point u):llb(l),urt(u)
    {
        this->xd = this->urt.x - this->llb.x;
        this->yd = this->urt.y - this->llb.y;
        this->zd = this->urt.z - this->llb.z;
    };

    bool isOut(Point&); // �ж�һ���Ƿ����ϰ����ⲿ

    friend ostream &operator << (ostream &stream, Obstacle &o)
    {
        return stream<<o.llb<<"--\n"<<o.urt<<"--\n("<<o.xd<<","<<o.yd<<","<<o.zd<<") ";
    };
};

ostream &operator << (ostream &stream, Cube &c);
class Cube
{
private:
    friend Octree;
    friend PathPlan;
    Point origin;  // ���ĵ�
    double radius;  // ���ĵ㵽��ľ���
    int state;  // 0λ�� 1λ�� -1Ϊ���
    Cube* children; // ��Ϊ��Ͻڵ� ����children
    Cube* parent; // ���ڵ�
    double bound; // �Ե�Ŀ���Ĺ��� ��·���滮ʱʹ��
    double cost; // ����㵽��ǰ��ľ��� ��·���滮ʱʹ��
public:
    Cube():origin(),radius(0),state(-1),
        children(NULL),parent(NULL),bound(0),cost(0) {};
    Cube(Point o,double r,int state=-1):origin(o),radius(r),state(state),
        children(NULL),parent(NULL),bound(0),cost(0) {};

    void setParm(Point o,double r,int s)
    {
        this->origin = o;
        this->radius = r;
        this->state = s;
    };

    void setBound(double b)
    {
        this->bound = b;
    };

    double getBound()
    {
        return this->bound;
    };

    Cube* getChildren()
    {
        return this->children;
    }

    bool isOut(Point&); // �ж�һ���Ƿ����������ⲿ
    bool isOut(Obstacle&); // �ж��ϰ�����ⲿ���Ƿ����������ⲿ

    void calcCost(Cube* pre)
    {
        this->cost = pre->cost + this->radius + pre->radius;
//        this->cost = pre->cost + getDisc(this->origin,pre->origin);
    };

    void calcBound(Cube* end)
    {
        this->bound = getDisc(this->origin,end->origin);
    };

    friend ostream &operator << (ostream &stream, Cube &c)
    {
        return stream<<c.origin<<" "<<c.radius<<" "<<c.state<<"\n";
//        <<"\nchild:"<<c.children<<" parent:"<<c.parent<<" bound:"<<c.bound<<" cost:"<<c.cost<<"\n";
    };
};

ostream &operator << (ostream &stream, Octree &c);
void print(ostream &stream,Cube*c,int len);
class Octree
{
private:
//    ofstream debug;
    friend PathPlan;
    const int len = 8, dim = 3;
    Point dir[8] = {{-1,-1,-1,},{-1,-1,1},{-1,1,-1},{-1,1,1},
        {1,-1,-1,},{1,-1,1},{1,1,-1},{1,1,1}
    };
    Cube root;
    double minr; // cube��С��r;
public:
    Octree():root(),minr() {};
    Octree(Point o,double r,double minr):root(o,r),minr(minr) {};

    void setParm(Point o,double r,double minr)
    {
//        debug.open("E:/Graduate Design/output/debug.txt");
        this->root.origin = o;
        this->root.radius = r;
        this->minr = minr;
    };

    void change(Cube *from,int dire,double dist,Cube *to);
    // �ⲿ���õ�build����
    void build(vector<Obstacle>& obs)
    {
        build(obs,this->root);
    };
    void build(vector<Obstacle>&,Cube&);  // �ڲ��ݹ���õ�build����

    // �õ�Cube��Obstacle�Ĺ�ϵ 0����Cubeȫ�� 1����Cubeȫ�� -1����Cube���
    int getRela(Cube&,Obstacle&);

    Cube* getCube(Point&,double r=0); // �õ������ڵ�cube rΪcube�İ뾶��������

    void getNeigh(vector<Cube*>&,Cube*); // �õ�ĳ�ڵ���ڽӽڵ�
    void getChildren(vector<Cube*>&,Cube*,int,int); // �õ�����ڽӽڵ������ڵ��ӽڵ�

    // ���˲�����Ϣ���
    void printOct()
    {
        cout<<*this;
    };

    // ���˲�����Ϣ����Ϊ�ı�
    void dump(string filePath);

    // ���ı��д洢�İ˲�����Ϣ����
    void load(string filePath);
    void loadChild(ifstream &in,Cube* parent);

    friend ostream &operator << (ostream &stream, Octree &c)
    {
        stream<<c.root;
        Cube *p = c.root.getChildren();
        print(stream,p,c.len);
        return stream;
    }

    friend void print(ostream &stream,Cube*c,int len)  // �ڲ����õ�print����
    {
        if(c)
            for(int i=0; i<len; i++)
            {
                stream<<*(c+i);
                print(stream,(c+i)->getChildren(),len);
            }
    }
};

class ObsFile
{
private:
    double ox=0,oy=0,oz=0,r=0;
    vector<Obstacle> obs;
public:
    // �ȽϺ���
    void more(double& recode,double value,char rela)
    {
        switch(rela)
        {
        case '<':
            if(value<recode)
                recode = value;
            break;
        case '>':
            if(value>recode)
                recode = value;
            break;
        }
    };

    void parseFile(string path)
    {
        ifstream file(path.c_str());
        int i,j,count,num;
        string title;
        double x,y,z,left=180,lower=90,right=0,upper=0,top=0,bottom=0;
        Point llb,urt;
        file>>count; // ��¼�ܸ���
        for(i=0; i<count; i++)
        {
            file>>title>>num; // type  5
            file>>title>>num; // floor 42
            z = dfl2dla(num);
            this->more(top,z,'>');
            file>>title>>num; // bbox 2
            file>>x>>y;  // ����
            llb = Point(x,y,0);
            this->more(left,x,'<');
            this->more(lower,y,'<');
            file>>x>>y;  // ����
            urt = Point(x,y,z);
            this->more(right,x,'>');
            this->more(upper,y,'>');
            obs.push_back(Obstacle(llb,urt));
            file>>title>>num; // points 8
            for(j=0; j<num; j++)
                file>>x>>y;
        }
        file.close();

        ox = (left+right)/2.0;
        oy = (lower+upper)/2.0;
        r = top-bottom;
        this->more(r,upper-lower,'>');
        this->more(r,right-left,'>');
        r = r/2.0;
        oz = r;
        cout<<obs.size()<<endl;
    };

    // ���ؽ��������ϰ�������
    vector<Obstacle>& getObs()
    {
        return this->obs;
    };

    // ���ذ˲��������������ĵ�
    Point getOrigin()
    {
        return Point(ox,oy,oz);
    };

    // ���ذ˲�����������İ뾶
    double getRadius()
    {
        return r;
    };
};

class PathPlan
{
private:
    struct cmp
    {
        bool operator()(Cube *a, Cube *b) const
        {
            return a->getBound() > b->getBound();
        }
    };
    Cube*s,*e;
    static Octree octree;
    priority_queue<Cube*,vector<Cube*>,cmp> open;
    static vector<Cube*> close;
    vector<Cube*> neigh; // �ڽӽڵ�洢
    static vector<Obstacle> *obs; // �ϰ�������
    ObsFile of; // �ϰ�����Ϣ�ļ�
public:
    // OpenGL ��Ҫ
    static float scale;
    static float ra; // ��ת�Ƕ�
    static float rx; // ��ת��x����
    static float ry; // ��ת��y����
    static float rz; // ��ת��z����
    static float tranx;
    static float trany;
    static int showObsFlag; // ��¼�л��ϰ����ָ�����������ͼ
    static int showResFlag; // ��¼�Ƿ���ʾ·���滮���
    static int showResCubeFlag; //��¼�Ƿ���ʾ·���滮�������������ͼ
    static double rgb[3];

    static void display(void); // OpenGL���û�ͼ����
    static void displayCube(Cube* c,double* color=NULL); // ��һ�������庯��
    static void displayOctree(Cube*); // ���ָ��İ˲�����ʾ
    static double* calcColor(double height); // ���뽨����߶� �����ʾ����ɫ

    // ���Եĳ�ʼ������
    void init0(Point root,double r,double minr,vector<Obstacle> &o)
    {
        this->octree.setParm(root,r,minr);
        this->obs = &o;
        this->octree.build(o);
    }

    void init(double minr,string obsPath,string octPath)
    {
        of.parseFile(obsPath); // �����ϰ����ļ�
        this->obs = &(of.getObs()); // �õ��ϰ�������

        if( -1 == (_access(octPath.c_str(),0)))
        {
            // �˲�����Ϣ�ļ������ڵ��������
            // ��ʼ�������Լ��˲����ķָ��
            this->octree.setParm(of.getOrigin(),of.getRadius(),minr);
            this->octree.build(of.getObs());
//            this->octree.printOct();
            // ���ָ�õİ˲�����Ϣ����Ϊ�ļ�
            this->octree.dump(octPath.c_str());
        }
        else
        {
            // ���ļ���ȡ�˲�������Ϣ
            this->octree.load(octPath.c_str());
        }
    };
    void plan(Point&,Point&);  // �滮·��
    void putCubes(Cube*);
    void showResult()
    {
        Cube* tmp;
        cout<<"show result"<<endl;
        for(int i=0; i<this->close.size(); i++)
        {
            tmp = this->close.at(i);
            cout<<*tmp<<endl;
        }
    };
};

#endif // BASE_H_INCLUDED
