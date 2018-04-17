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

// 经度的差值转化为纬度的差值 dlo 精度改变量 dla 纬度改变量
double dlo2dla(double dlo,double la);
// 纬度的差值转化为经度的差值
double dla2dlo(double dla,double la);

// 楼层高度差值转化为纬度的差值
double dfl2dla(int floor);

const float fl2m = 3.0; // 一层楼高度
const float dla2m = 111000.0; // 同一经线上纬度相差1度的距离

class Point;
class Obstacle;
class Cube;
class Octree;
class ObsFile;
class PathPlan;

ostream &operator << (ostream &stream, Point &p);
Point operator + (Point x, Point y);
Point operator * (Point x, double n);
double getDisc(Point&,Point&); // 计算两点之间的距离
int getIndex(Point&,Point&);   // 得到某点在另一点的空间方位对应的索引号

class Point
{
private:
    friend Cube;
    friend Obstacle;
    friend PathPlan;
    double x;  // longitude
    double y;  // latitude
    double z;  // altitude 高度存储相应的纬度改变量即高度/111km
public:
    Point():x(0),y(0),z(0) {};
    Point(double x,double y,double z):x(x),y(y),z(z) {};

    Point& operator = (Point p) // 赋值操作
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

    // 第一个参数在第二个参数的方位对应的索引号
    friend int getIndex(Point &p, Point &c)
    {
        // 对于P和c相同的情况的处理
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

    friend double getDisc(Point& a,Point& b) // 计算两点之间的距离
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
    // 目前障碍物只当做长方体处理
private:
    friend Cube;
    friend Octree;
    friend PathPlan;
    Point llb; // lower left bottom
    Point urt; // upper right top
    double xd,yd,zd; // 每个方向的边长
public:
    Obstacle():llb(),urt(),xd(0),yd(0),zd(0) {};
    Obstacle(Point l,Point u):llb(l),urt(u)
    {
        this->xd = this->urt.x - this->llb.x;
        this->yd = this->urt.y - this->llb.y;
        this->zd = this->urt.z - this->llb.z;
    };

    bool isOut(Point&); // 判断一点是否在障碍物外部

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
    Point origin;  // 中心点
    double radius;  // 中心点到面的距离
    int state;  // 0位空 1位满 -1为混合
    Cube* children; // 若为混合节点 则有children
    Cube* parent; // 父节点
    double bound; // 对到目标点的估计 在路径规划时使用
    double cost; // 从起点到当前点的距离 在路径规划时使用
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

    bool isOut(Point&); // 判断一点是否在立方体外部
    bool isOut(Obstacle&); // 判断障碍物的外部点是否在立方体外部

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
    double minr; // cube最小的r;
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
    // 外部调用的build方法
    void build(vector<Obstacle>& obs)
    {
        build(obs,this->root);
    };
    void build(vector<Obstacle>&,Cube&);  // 内部递归调用的build方法

    // 得到Cube与Obstacle的关系 0代表Cube全空 1代表Cube全满 -1代表Cube混合
    int getRela(Cube&,Obstacle&);

    Cube* getCube(Point&,double r=0); // 得到点所在的cube r为cube的半径限制条件

    void getNeigh(vector<Cube*>&,Cube*); // 得到某节点的邻接节点
    void getChildren(vector<Cube*>&,Cube*,int,int); // 得到混合邻接节点中相邻的子节点

    // 将八叉树信息输出
    void printOct()
    {
        cout<<*this;
    };

    // 将八叉树信息保存为文本
    void dump(string filePath);

    // 将文本中存储的八叉树信息解析
    void load(string filePath);
    void loadChild(ifstream &in,Cube* parent);

    friend ostream &operator << (ostream &stream, Octree &c)
    {
        stream<<c.root;
        Cube *p = c.root.getChildren();
        print(stream,p,c.len);
        return stream;
    }

    friend void print(ostream &stream,Cube*c,int len)  // 内部调用的print方法
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
    // 比较函数
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
        file>>count; // 记录总个数
        for(i=0; i<count; i++)
        {
            file>>title>>num; // type  5
            file>>title>>num; // floor 42
            z = dfl2dla(num);
            this->more(top,z,'>');
            file>>title>>num; // bbox 2
            file>>x>>y;  // 左下
            llb = Point(x,y,0);
            this->more(left,x,'<');
            this->more(lower,y,'<');
            file>>x>>y;  // 右上
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

    // 返回解析到的障碍物数组
    vector<Obstacle>& getObs()
    {
        return this->obs;
    };

    // 返回八叉树根立方体中心点
    Point getOrigin()
    {
        return Point(ox,oy,oz);
    };

    // 返回八叉树根立方体的半径
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
    vector<Cube*> neigh; // 邻接节点存储
    static vector<Obstacle> *obs; // 障碍物数组
    ObsFile of; // 障碍物信息文件
public:
    // OpenGL 需要
    static float scale;
    static float ra; // 旋转角度
    static float rx; // 旋转轴x坐标
    static float ry; // 旋转轴y坐标
    static float rz; // 旋转轴z坐标
    static float tranx;
    static float trany;
    static int showObsFlag; // 记录切换障碍物或分割后立方体的视图
    static int showResFlag; // 记录是否显示路径规划结果
    static int showResCubeFlag; //记录是否显示路径规划结果的立方体视图
    static double rgb[3];

    static void display(void); // OpenGL调用画图函数
    static void displayCube(Cube* c,double* color=NULL); // 画一个立方体函数
    static void displayOctree(Cube*); // 将分割后的八叉树显示
    static double* calcColor(double height); // 输入建筑物高度 输出显示的颜色

    // 测试的初始化函数
    void init0(Point root,double r,double minr,vector<Obstacle> &o)
    {
        this->octree.setParm(root,r,minr);
        this->obs = &o;
        this->octree.build(o);
    }

    void init(double minr,string obsPath,string octPath)
    {
        of.parseFile(obsPath); // 解析障碍物文件
        this->obs = &(of.getObs()); // 得到障碍物数组

        if( -1 == (_access(octPath.c_str(),0)))
        {
            // 八叉树信息文件不存在的情况处理
            // 初始化参数以及八叉树的分割建立
            this->octree.setParm(of.getOrigin(),of.getRadius(),minr);
            this->octree.build(of.getObs());
//            this->octree.printOct();
            // 将分割好的八叉树信息保存为文件
            this->octree.dump(octPath.c_str());
        }
        else
        {
            // 从文件读取八叉树的信息
            this->octree.load(octPath.c_str());
        }
    };
    void plan(Point&,Point&);  // 规划路径
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
