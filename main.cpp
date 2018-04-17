#include <iostream>
#include <queue>
#include <sstream>

#include "base.h"
using namespace std;

bool mouse_left_down, mouse_right_down;
int width = 600,height = 600,
    lrx = 0, // last rotate mouse x
    lry = 0, // last rotate mouse y
    ltx = 0, // last translate mouse x
    lty = 0; // last translate mouse y


// 计算向量的z坐标
double calcZ(double &x,double &y)
{
    double square = x*x + y*y,z=0,length=0;
    if ( square <= 1.0)
    {
        z = sqrt( 1.0 - square);
    }
    else
    {
        length = sqrt(square);
        x /= length;
        y /= length;
        z = 0.0;
    }
    return z;
}

// Convert the screen coordinates (in pixels) to camera coordinates (in [-1, 1])
double normalize(int v,char axis)
{
    double nv=0;
    switch(axis)
    {
    case 'x':
        nv = 2.0*v/width - 1.0;
        break;
    case 'y':
        nv = 1.0 - 2.0*v/height;
        break;
    }
    return nv;
}

// ArcBall
void calcRotate(int lx,int ly,int cx,int cy)
{
    // 坐标归一化转换
    double lxd = normalize(lx,'x'),
           lyd = normalize(ly,'y'),
           cxd = normalize(cx,'x'),
           cyd = normalize(cy,'y'),
           // 计算向量
           lzd = calcZ(lxd,lyd),
           czd = calcZ(cxd,cyd);

    // 计算旋转角 向量的点积
    PathPlan::ra = acos(lxd*cxd+lyd*cyd+lzd*czd)*180;
    // 计算旋转轴 向量的叉积
    PathPlan::rx = lyd*czd - lzd*cyd;
    PathPlan::ry = cxd*lzd - lxd*czd;
    PathPlan::rz = lxd*cyd - cxd*lyd;
}


// 键盘输入事件处理
static void key(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27 :
    case 'q':
        exit(0);
        break;

    case '+':
        PathPlan::scale += 0.05;
        break;

    case '-':
        if (PathPlan::scale>0.1)
        {
            PathPlan::scale -= 0.05;
        }
        break;
    case 'o':
        PathPlan::showObsFlag*=-1; // 切换障碍物或分割后立方体的视图
        break;
    case 'r':
        PathPlan::showResFlag*=-1; // 切换是否显路径规划的结果
        break;
    case 'c':
        PathPlan::showResCubeFlag*=-1; //切换是否显示路径规划结果的立方体视图
        break;
    }
    glutPostRedisplay();
}

// 鼠标按键事件处理
void mouse(int button,int state,int x,int y)
{
//    cout<<button<<" "<<state<<" "<<x<<" "<<y<<endl;
    switch(button)
    {
    case GLUT_LEFT_BUTTON:
        if(state==GLUT_DOWN)
        {
            mouse_left_down = true;
            lrx = x;
            lry = y;
        }
        else
        {
            mouse_left_down = false;
        }
        break;
    case GLUT_RIGHT_BUTTON:
        if(state==GLUT_DOWN)
        {
            mouse_right_down = true;
            ltx = x;
            lty = y;
        }
        else
        {
            mouse_right_down = false;
        }
        break;
    }

}

// 鼠标移动事件处理
void motion(int x,int y)
{
    int dx,dy;
    if(mouse_left_down)
    {
        calcRotate(lrx,lry,x,y);
    }
    if(mouse_right_down)
    {
        dx=x-ltx;
        dy=y-lty;
        PathPlan::tranx = dx*1.0/width;
        PathPlan::trany = -dy*1.0/height;
    }
    glutPostRedisplay();
}

// 窗口改变事件处理
void resize(int width, int height)
{
    glViewport(0, 0, width, height);
}


int main(int argc, char *argv[])
{
    std::cout.precision(std::numeric_limits<double>::digits10);

    string octPath = "E:/Graduate Design/output/octree.txt", // 八叉树文件 不存在会新建
           obsPath = "E:/Graduate Design/output/output_4.txt"; // 障碍物文件 必须要有
    PathPlan pp;

    pp.init(0.00000001,obsPath,octPath);
//    Point s(108.935188,34.230758,0),e(108.941508,34.229194,0); // 70
    Point s(108.937955,34.230528,0),e(108.939617,34.231015,0); // 4
    pp.plan(s,e);

/*
    // 测试数据
    Point p(0,0,0),
          s(7,7,7),e(-7,-7,-7),
          llb(-2,-2,2),urt(2,2,2),
          llb1(-6,-6,-6),urt1(-2,-2,-2),
          llb2(2,2,2),urt2(6,6,6);
    Obstacle o(llb,urt),
             o1(llb1,urt1),
             o2(llb2,urt2);
    vector<Obstacle> obs1,obs2;
    obs1.push_back(o);
    obs2.push_back(o1);
    obs2.push_back(o2);

    PathPlan pp;
    pp.init0(Point(0,0,0),8,0,obs2);
    pp.plan(s,e);
    // 测试数据结束
*/

    // OpenGL 相关函数
    glutInit(&argc, argv);
    glutInitWindowSize(width,height);
    glutInitWindowPosition(10,10);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

    glutCreateWindow("my demo");

    glutDisplayFunc(PathPlan::display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(key);
    glutMainLoop();

    return EXIT_SUCCESS;
}
