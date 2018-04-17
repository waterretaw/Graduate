#include "base.h"

double dlo2dla(double dlo,double la)
{
    return dlo;
    return dlo*1.0/cos(la*1.0/180);
}

double dla2dlo(double dla,double la)
{
    return dla;
    return dla*cos(la*1.0/180);
}

double dfl2dla(int floor)
{
    return floor*1.0*fl2m/dla2m;
}


bool Obstacle::isOut(Point& p)
{
    if(p.x-this->llb.x<=0||p.x-this->llb.x>=this->xd
            ||p.y-this->llb.y<=0||p.y-this->llb.y>=this->yd
            ||p.z-this->llb.z<=0||p.z-this->llb.z>=this->zd)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Cube::isOut(Point& p)
{
    if(dlo2dla(abs(p.x-this->origin.x),this->origin.y)>=this->radius
            ||abs(p.y-this->origin.y)>=this->radius
            ||abs(p.z-this->origin.z)>=this->radius)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Cube::isOut(Obstacle& o)
{
    double x[2] = {o.llb.x,o.urt.x},
                  y[2] = {o.llb.y,o.urt.y},
                         z[2] = {o.llb.z,o.urt.z};
    Point p;
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++)
            for(int k=0; k<2; k++)
            {
                p = Point(x[i],y[j],z[k]);
                if(!this->isOut(p))
                    return false;
            }
    return true;
}

int Octree::getRela(Cube& c,Obstacle& o)
{

    if(!c.isOut(o))
    {
        return -1;
    }

    if(o.isOut(c.origin))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void Octree::change(Cube *from,int dire,double dist,Cube *to)
{
    to->origin = this->dir[dire]*dist + from->origin;
    to->radius = dist;
    to->parent = from;
}

void Octree::build(vector<Obstacle>& obs,Cube& c)
{
    // 判断立方体与障碍物的关系
    int rela,i;
    for(i=0; i<obs.size(); i++)
    {
        rela = this->getRela(c,obs[i]);
        if(rela)
            break;
    }
    c.state = rela; // 记录障碍物的状态
    double r = c.radius*1.0/2;
    // 如果立方体为混合状态且大小符合要求递归分割
    if(-1 == rela && r>=this->minr)
    {
        Cube* t = new Cube[this->len];
        c.children = t;
        for(int i=0; i<this->len; i++)
        {
            // 对新创建立方体的属性赋值
            this->change(&c,i,r,t+i);
            // 递归分割
            this->build(obs,*(t+i));
        }
    }
}


void Octree::dump(string filePath)
{
    ofstream out(filePath.c_str());
    out.precision(std::numeric_limits<double>::digits10);
    out<<this->minr<<"\n";
    out<<*this;
    out.close();
}

void Octree::load(string filePath)
{
    ifstream in(filePath.c_str());
    double x=0,y=0,z=0,r=0;
    int state=-2;
    Cube* parent;

    in>>r;
    this->minr = r;

    in>>x>>y>>z>>r>>state;

    parent = &this->root;
    parent->setParm(Point(x,y,z),r,state);
    if(-1 == state)
    {
        this->loadChild(in,parent);
    }
    in.close();
}

void Octree::loadChild(ifstream& in,Cube* parent)
{
    if(!in.eof())
    {
        double x=0,y=0,z=0,r=0;
        int state=-2,i;
        Cube* children;

        children = new Cube[this->len];
        parent->children = children;
        for(i=0; i<this->len; i++)
        {
            in>>x>>y>>z>>r>>state;
            (children+i)->setParm(Point(x,y,z),r,state);
            (children+i)->parent = parent;
            if(state==-1&&r>=2*this->minr)
                this->loadChild(in,children+i);
        }
    }
}


Cube* Octree::getCube(Point& p,double r)
{
    Cube* c = &this->root;
    int i=0;
    while(c->children&&c->radius>=r)
    {
        i=getIndex(p,c->origin);
        c=c->children+i;
    }
    return c;
}

// 得到六个方向上的邻居节点
void Octree::getNeigh(vector<Cube*>& neigh,Cube* c)
{
//    debug<<"==="<<*c<<endl;
    // 获取有效的邻接节点
    int dx[6]= {1,-1,0,0,0,0},
               dy[6]= {0,0,1,-1,0,0},
                      dz[6]= {0,0,0,0,1,-1},
                             i=0,fixed,ci;
    double nx,ny,nz;
    Cube* n;
    Point p;

    for(i=0; i<this->dim*2; i++)
    {
        c->origin.getxyz(nx,ny,nz);
        nx += c->radius*dx[i]*1.1;
        ny += c->radius*dy[i]*1.1;
        nz += c->radius*dz[i]*1.1;
        p.setxyz(nx,ny,nz);

        n = getCube(p,c->radius);
        if(n->state == 0 && n->bound == 0)
        {
//            debug<<"+++"<<*n<<endl;
            neigh.push_back(n);
        }
        else if(n->state == -1 && n->bound == 0)
        {
            // 对于邻接节点为混合的情况 即 大到小的情况
            fixed = 1<<(2-i/2);
            ci = getIndex(n->origin,n->parent->origin);
            getChildren(neigh,n->parent->children,fixed,ci);
        }
    }
}

// fixed即坐标xyz中不变的一位  ci即children index  first为children first
void Octree::getChildren(vector<Cube*>& neigh,Cube* first,int fixed,int ci)
{
    int di[4],k=0,t=1,ni,i;
    Cube* n;
    di[k++]=0;
    di[k++]=fixed^7;
    for(i=0; i<this->dim; i++)
    {
        ni = (t<<i);
        if(ni==fixed)
            continue;
        di[k++] = ni;
    }
    for(i=0; i<k; i++)
        di[i] ^= ci;
    for(i=0; i<k; i++)
    {
        n = first + di[i];
        if(n->state == 0&& n->bound==0)
        {
//            debug<<"***"<<*n<<endl;
            neigh.push_back(n);
        }
        else if(n->state==-1&&n->bound==0&&n->children)
        {
            getChildren(neigh,n->children,fixed,ci);
        }
    }

}

void PathPlan::plan(Point& s,Point& e)
{
    Cube *c;
    this->s = this->octree.getCube(s);
    this->e = this->octree.getCube(e);
    this->s->calcBound(this->e);
    this->open.push(this->s);
    while(!this->open.empty())
    {
        c = this->open.top();
        this->open.pop();
//        cout<<"POP:"<<*c<<endl;
//        cout<<"POP_:"<<open.size()<<endl;
        this->close.push_back(c);
        if(c==this->e)
            break;
        this->putCubes(c);
    }

    // 测试
//    while(!this->open.empty())
//    {
//        c = this->open.top();
//        this->open.pop();
//        this->close.push_back(c);
//    }

//    this->showResult();
}

void PathPlan::putCubes(Cube*c)
{
    Cube* tmp;
    this->neigh.clear();
    this->octree.getNeigh(this->neigh,c);
    while(!this->neigh.empty())
    {
        tmp = this->neigh.back();
        this->neigh.pop_back();
        tmp->calcCost(c); // 先计算cost
        tmp->calcBound(this->e); // 后计算bound
        this->open.push(tmp);
//        cout<<"IN:"<<*tmp<<endl;
    }
}


vector<Obstacle> *PathPlan::obs = NULL;
Octree PathPlan::octree;
vector<Cube*> PathPlan::close;
float PathPlan::scale = 1.0;
float PathPlan::ra=0.0;
float PathPlan::rx=0.0;
float PathPlan::ry=0.0;
float PathPlan::rz=0.0;
float PathPlan::tranx = 0.0;
float PathPlan::trany = 0.0;
int PathPlan::showObsFlag = -1;
int PathPlan::showResFlag = -1;
int PathPlan::showResCubeFlag = -1;
double PathPlan::rgb[3]= {0,0,0};

double* PathPlan::calcColor(double height)
{
    double temp;
    if(octree.root.origin.z)
        temp = height/octree.root.origin.z*2.0;
    else
        temp = height/octree.root.radius;

    // 按照灰度图像转彩色图像的方法处理 将高度归一化的结果看做为灰度
    if(temp<=0.25)
    {
        rgb[2] = 1;  //blue
        rgb[1] = 1 - 4*temp;  //green
        rgb[0] = 0;  //red
    }
    else if(temp<=0.5)
    {
        rgb[2] = 2-4*temp;  //blue
        rgb[1] = 4*temp -1;  //green
        rgb[0] = 0;  //red
    }
    else if(temp<=0.75)
    {
        rgb[2] = 0;  //blue
        rgb[1] = 1;  //green
        rgb[0] = 4*temp-2;  //red
    }
    else
    {
        rgb[2] = 0;  //blue
        rgb[1] = 4-4*temp;  //green
        rgb[0] = 1;  //red

    }
    return rgb;
}

void PathPlan::display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-1.5*octree.root.radius,
            1.5*octree.root.radius,
            -1.5*octree.root.radius,
            1.5*octree.root.radius,
            -1.5*octree.root.radius,
            1.5*octree.root.radius);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity() ;


    glScaled(scale,scale,scale);
    glRotated(ra,rx,ry,rz);
    glTranslated(tranx,trany,0);

    if(showObsFlag>0)
    {
        // 显示障碍物
        for(int i=0; i<obs->size() ; i++)
        {
            glPushMatrix();
            glColor3dv(calcColor(obs->at(i).urt.z)); // 不同的高度显示不同的颜色
            // 以立方体中心点 作为坐标原点
            glTranslated((obs->at(i).llb.x+obs->at(i).urt.x)*1.0/2-octree.root.origin.x,
                         (obs->at(i).llb.y+obs->at(i).urt.y)*1.0/2-octree.root.origin.y,
                         (obs->at(i).llb.z+obs->at(i).urt.z)*1.0/2-octree.root.origin.z);
            // 将立方体拉伸画出长方体
            glScaled(obs->at(i).xd,obs->at(i).yd,obs->at(i).zd);
            glutWireCube(1); // 参数为边长
            glPopMatrix();
        }
    }
    else
    {
        // 显示分割后的立方体
        Cube *p = &octree.root;
        if(p->children)
            displayOctree(p->children);
        else
            displayCube(p);
    }

    if(showResFlag>0)
    {
        // 显示路径规划结果
        glPushMatrix();
        glColor3d(1.0,0.0,1.0);
        glBegin(GL_LINE_STRIP);
        Cube* tmp;
        for(int i=0; i<close.size(); i++)
        {
            tmp = close.at(i);
            glVertex3d(tmp->origin.x-octree.root.origin.x,
                       tmp->origin.y-octree.root.origin.y,
                       tmp->origin.z-octree.root.origin.z);
        }
        glEnd();
        glPopMatrix();

    }

    if(showResCubeFlag>0)
    {
        // 显示路径规划结果的立方体视图
        Cube* tmp;
        double color[3] = {1.0,1.0,1.0};
        for(int i=0; i<close.size(); i++)
        {
            tmp = close.at(i);
            displayCube(tmp,color);
        }
    }

    glutSwapBuffers();
}

void PathPlan::displayCube(Cube* c,double* color)
{
    // 不同状态不同颜色
    if(color)
    {
        glColor3dv(color);
    }
    else
    {
        switch(c->state)
        {
        case 0:
            glColor3d(0,1,0);
            break;
        case 1:
            glColor3d(1,0,0);
            break;
        case -1:
            glColor3d(0,0,1);
            break;
        default:
            break;
        }
    }

    glPushMatrix();
    // 以立方体中心点 作为坐标原点
    glTranslated(c->origin.x-octree.root.origin.x,
                 c->origin.y-octree.root.origin.y,
                 c->origin.z-octree.root.origin.z);
    glutWireCube(2*c->radius); // 参数为边长
    glPopMatrix();
}

void PathPlan::displayOctree(Cube* c)
{
    for(int i=0; i<octree.len; i++)
    {
        Cube* children = (c+i)->children;
        if(children)
            displayOctree(children);
        else
            displayCube(c+i);
    }
}
