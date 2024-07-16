#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#define screenWidth 500
#define screenHeight 500
#define MY_PI 3.1415926535
#define Num_Points 3

using namespace sf;
using namespace std;
using namespace Eigen;

/*********************************************************************************************************/
/*                  参数初始化部分                                                                        */
/*********************************************************************************************************/
// 摄像机位置
Eigen::Vector3f eye_pos = { 0, 1, 5 };
// 摄像机朝向
Eigen::Vector3f eye_dir = { 0, 0, -1 };
// 摄像机上方方向
Eigen::Vector3f eye_updir = { 0, 1, 0 };
// FOV
float eye_FOV = 45;

// 初始点的位置，也用于初始化
vector<Eigen::Vector3f> Initial_points{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };
// 记录现在点的位置，用于处理旋转操作
vector<Eigen::Vector3f> Now_points = Initial_points;
// ModelView变换后的点的位置，用于computeCov2D带入参数，不用重复计算
vector<Eigen::Vector3f> ModelView_points = Initial_points;
// MVP，Viewport投影后点的位置
vector<Eigen::Vector3f> Projected_triangle_points = Initial_points;
// 中心位置，用做旋转轴过的点，这样能保持图形在屏幕中心旋转
Eigen::Vector3f Geometry_center = { 0, 1, -2 };

// 最终绕XYZ轴的旋转矩阵
// 在主函数中每帧计算
Eigen::Matrix4f Totalrotation_matirx = Matrix4f::Identity();

/*********************************************************************************************************/
/*                  矩阵计算部分                                                                          */
/*********************************************************************************************************/

// 得到 绕任意轴旋转任意角度 旋转矩阵，运用罗德里格斯旋转公式
// 函数里会把axis_direction正则化，因此方向坐标可以直接用物体坐标减相机坐标
// 任意轴的定义方法：过任意点的任意向量（不一定是单位向量，指明方向即可）
// 任意点默认为原点(0, 0, 0)，但是实际上运算的时候应该要带入中心点Geometry_center
// 代入的方向与点都是三维的，不需要考虑齐次坐标
Eigen::Matrix4f get_rotation_arbitrary_axis(Eigen::Vector3f axis_direction, float rotation_angle, Eigen::Vector3f point = Eigen::Vector3f::Zero());

// 得到MV变换矩阵，用于将摄像机和全部物体移动
// 移动原则：相机固定在原点(0, 0, 0)，面朝-Z方向，正上方为Y方向
Eigen::Matrix4f get_modelview_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f eye_dir, Eigen::Vector3f eye_updir);

// 得到投影变换矩阵
// eye_fov取水平FOV
// aspect_ratio是宽高比
// 包括了Pers->Ortho和Ortho两个变换
Eigen::Matrix4f get_projection_matrix(float eye_fov = 45, float aspect_ratio = 1, float zNear = 0.1, float zFar = 50);

// viewport变换，用于把透视后正则化2x2x2的立方体投到screenWidth x screenHeight的屏幕上
Eigen::Matrix4f get_viewport_matrix(int screenwidth = screenWidth, int screenheight = screenHeight);

// 在计算旋转矩阵的基础上，计算出绕X, Y, Z轴旋转的最终结果
// 基本上就是把原来实现的Rotation_operation函数里计算旋转矩阵的部分拆出来
// 因为要计算绕X, Y, Z轴旋转的最终结果，因此键盘鼠标操作逻辑也在此函数中（包括按R键Initiate初始化复原）
Eigen::Matrix4f get_totalrotation_matrix(RenderWindow& window, bool moveflag = 0);

/*********************************************************************************************************/
/*                  正常光栅化部分                                                                        */
/*********************************************************************************************************/
// 初始化三角形点
void Initialize();

// ifinside函数，用于判断给定屏幕上点P是否在三角形ABC中
// 因为不需要考虑z坐标，因此P点用二维向量
bool ifinside(Eigen::Vector2f P, Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C);

// 用于存储深度
// 在Points_Projection函数中存入深度
vector<float> depth_list(Num_Points);

// 对每个点进行投影操作，从真实坐标投影到屏幕上
// 并且将每个点的深度存入深度数组
// 将ModelView变换后的结果单独存入ModelView_points，下标对应
void Points_Projection();

// SFML逐像素操作真是闹麻了
// 正常光栅化图形
void Rasterizer_Normal(RenderWindow& window);

/*********************************************************************************************************/
/*                  图形旋转部分                                                                          */
/*********************************************************************************************************/
// 记录鼠标上一次的位置，在主函数每一帧的最后记录当前鼠标位置
sf::Vector2i mouseLastPosition;
// 鼠标移动flag，在主函数的事件处理函数中更新，在主函数的每帧结束时置0
bool Mousemoveflag = 0;

// 最需要注意的就是，旋转矩阵是对原始点乘的，并不是投影过来的点乘这个矩阵
// 这个问题基本上卡了一下午，笑死我了
// 现在把这个函数拆成两部分：计算绕XYZ轴最终的旋转矩阵，对所有点应用这个旋转矩阵
// 该函数仅保留对所有点应用这个旋转矩阵操作，计算旋转矩阵部分写成一个单独的函数：get_totalrotation_matrix
// 已经把最终旋转矩阵变成一个全局变量，因此该函数带入的参数全部删除
void Rotation_operation();

/*********************************************************************************************************/
/*                  球谐函数部分                                                                          */
/*********************************************************************************************************/
// 定义常量
// 全部为float，不能是double，因为eigen库不能类型转换
const double SH_C0 = 0.28209479177387814;
const double SH_C1 = 0.4886025119029199;
VectorXf SH_C2{ {1.0925484305920792, -1.0925484305920792, 0.31539156525252005, -1.0925484305920792, 0.5462742152960396} };
VectorXf SH_C3{ {-0.5900435899266435, 2.890611442640554, -0.4570457994644658, 0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435} };
// sh是16x3的矩阵
// 返回的是一个3维列向量，所有元素限制在0和1之间
// 代入的pos是Now_points
Eigen::Vector3f computeColorFromSH(Eigen::Vector3f pos, Eigen::Vector3f campos, Eigen::MatrixXf sh, int deg = 3);

/*********************************************************************************************************/
/*                  3DGS部分                                                                             */
/*********************************************************************************************************/

// 统一用Matrix3f
// covariance = RS[S^T][R^T]
// mod也是缩放，默认为1
// rotation就是带入最终旋转矩阵
Eigen::Matrix3f computeCov3D(Eigen::Vector3f scale, double mod, Eigen::Matrix4f rotation = Matrix4f::Identity());


// 雅克比矩阵相关计算，带入参数ModelView_point为经过ModelView变换的点
// 不过每次都需要把MV_matrix矩阵代入
// 已与源代码结果对比，两个compute计算结果完全一致
// 返回的按理来说一个是一个2x2的矩阵，但因为是对称的，所以只返回3个值即可
Eigen::Vector3f computeCov2D(Eigen::Vector3f ModelView_point, float focal_x, float focal_y, float tan_FOV, Eigen::Matrix3f cov3D, Eigen::Matrix4f MV_matrix);

// 是否处理该点，默认都处理
// 需要更新这个数组为{ 1,1,1 }
bool pointFlags[Num_Points] = { 1,1,1 };

// scales: Num_Pointsx3的全1矩阵
Eigen::MatrixXf scales = Eigen::MatrixXf::Ones(Num_Points, 3);
// cov3Ds
vector<Eigen::Matrix3f> cov3Ds(Num_Points);
// conic_opacity
vector<Eigen::Vector4f> conic_opacity(Num_Points);
// radius
vector<float> radiuslist(Num_Points);
// 高斯球的颜色，用sf::Color存储
// 这里存两种是因为作对比，如果用Color运算结果为红色，如果用Vector运算，最后再类型转换为int最后结果为灰色，很奇怪
vector<sf::Color> RGB_3DGS(Num_Points);
vector<Eigen::Vector3f> RGB_3DGS_Tmp(Num_Points);

// shs，每一个都是16x3的矩阵
// 为了方便检查结果，设置为固定值
// sh数组的值会导致左上角消失，右下角清晰
//vector<Eigen::MatrixXf> shs(Num_Points, Eigen::MatrixXf::Constant(16, 3, 0.5f));
vector<Eigen::MatrixXf> shs(Num_Points, Eigen::MatrixXf::Constant(16, 3, 0.1f));

// 预处理函数
// 代入的参数tan_half_FOV是FOV的一半的tan值
// 中间经过计算，更新cov3Ds, pointFlags, conic_opacity, radiuslist, RGB_3DGS
void Preprocess(float focal_x, float focal_y, float tan_half_FOV);

// 定义结构体，存储深度信息和下标
struct DepthIndex {
    float depth;
    int index;
};

// 定义比较函数
// 按深度升序排序，因为看向-Z方向，越远的深度越小，越近的深度越大
// 渲染的时候先渲染远处的，再渲染近处的，能把远处的遮挡住
bool compareDepth(const DepthIndex& a, const DepthIndex& b);

// 3DGS渲染结果画到屏幕上
// 失败了失败了失败了失败了失败了失败了失败了失败了，为什么只有一个高斯球？
// 破案了，是前面颜色设置的时候用的255*(int)result(0)，直接设置为0了
// 算是实现功能了
void Rasterizer_3DGS(RenderWindow& window);


/*********************************************************************************************************/
/*                  主函数部分                                                                            */
/*********************************************************************************************************/
int main()
{
    sf::RenderWindow window_normal(sf::VideoMode(screenWidth, screenHeight), "Rasterizer");
    sf::RenderWindow window_3DGS(sf::VideoMode(screenWidth, screenHeight), "3DGS");

    // 获取屏幕的分辨率
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int FullscreenWidth = desktop.width;
    int FullscreenHeight = desktop.height;

    // 计算窗口的新位置，居中并且两个窗口相互紧挨着
    sf::Vector2i position1(FullscreenWidth / 2 - screenWidth, FullscreenHeight / 2 - screenHeight / 2);
    sf::Vector2i position2(FullscreenWidth / 2 - screenWidth + screenWidth + 3, FullscreenHeight / 2 - screenHeight / 2);

    // 设置窗口的位置
    window_normal.setPosition(position1);
    window_3DGS.setPosition(position2);


    while (window_normal.isOpen() && window_3DGS.isOpen())
    {
        sf::Event event;
        while (window_normal.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window_normal.close();
            // 鼠标移动事件
            if (event.type == sf::Event::MouseMoved)
            {
                Mousemoveflag = 1;
            }
        }

        window_normal.clear(Color::White);
        window_3DGS.clear(Color::White);

        // MVP, Viewport投影
        Points_Projection();
        // 正常光栅化
        Rasterizer_Normal(window_normal);
        // 渲染3DGS
        Rasterizer_3DGS(window_3DGS);

        // 计算最终的旋转矩阵
        Totalrotation_matirx = get_totalrotation_matrix(window_normal, Mousemoveflag);
        // 对全部点进行旋转操作
        Rotation_operation();

        window_normal.display();
        window_3DGS.display();

        // 记录这一帧鼠标的位置，用作下一帧的LastPosition
        mouseLastPosition = sf::Mouse::getPosition(window_normal);
        // 鼠标flag置0
        Mousemoveflag = 0;
    }


    return 0;
}

/*********************************************************************************************************/
/*                  函数实现部分                                                                          */
/*********************************************************************************************************/
// 得到 绕任意轴旋转任意角度 旋转矩阵，运用罗德里格斯旋转公式
// 函数里会把axis_direction正则化，因此方向坐标可以直接用物体坐标减相机坐标
// 任意轴的定义方法：过任意点的任意向量（不一定是单位向量，指明方向即可）
// 任意点默认为原点(0, 0, 0)，但是实际上运算的时候应该要带入中心点
// 代入的方向与点都是三维的，不需要考虑齐次坐标
Eigen::Matrix4f get_rotation_arbitrary_axis(Eigen::Vector3f axis_direction, float rotation_angle, Eigen::Vector3f point)
{
    // 先将这个向量过的点移回原点
    Eigen::Matrix4f Translate;
    Translate << 1, 0, 0, -point(0),
        0, 1, 0, -point(1),
        0, 0, 1, -point(2),
        0, 0, 0, 1;
    // 再旋转相应角度，运用罗德里格斯旋转公式
    // 方向向量正则化
    axis_direction.normalize();
    // 计算角度
    float theta = rotation_angle / 180 * MY_PI;
    // 计算cos和sin
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    // 根据公式计算旋转矩阵，公式来源: GAMES101_Lecture_04 P10
    // Rodrigues’ Rotation Formula: Rotation by angle theta around axis n
    // R(n, theta) = cos(theta) * I + (1 - cos(theta)) * n * n.T + sin(theta) * N
    // N = { 0, -nz, ny,
    //      nz, 0, -nx,
    //      -ny, nx, 0 }
    // 注意：这个矩阵算出来是3x3的，最后要加齐次坐标变成4x4的旋转矩阵
    Matrix3f N;
    N << 0, -axis_direction(2), axis_direction(1),
        axis_direction(2), 0, -axis_direction(0),
        -axis_direction(1), axis_direction(0), 0;
    Matrix3f Rotation_3x3;
    Matrix3f Eye = Matrix3f::Identity();
    Rotation_3x3 = cos_theta * Eye + (1 - cos_theta) * axis_direction * axis_direction.transpose() + sin_theta * N;
    // 将3x3旋转矩阵扩展为4x4旋转矩阵
    Matrix4f Rotation_4x4 = Matrix4f::Identity();
    Rotation_4x4.block<3, 3>(0, 0) = Rotation_3x3; // 将3x3旋转矩阵嵌入到左上角
    // 最后将这个向量过的点移回去
    Eigen::Matrix4f Translate_back;
    Translate_back << 1, 0, 0, point(0),
        0, 1, 0, point(1),
        0, 0, 1, point(2),
        0, 0, 0, 1;
    // 计算最终的4x4矩阵
    Eigen::Matrix4f result = Translate_back * Rotation_4x4 * Translate;
    return result;
}

// 得到MV变换矩阵，用于将摄像机和全部物体移动
// 移动原则：相机固定在原点(0, 0, 0)，面朝-Z方向，正上方为Y方向
Eigen::Matrix4f get_modelview_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f eye_dir, Eigen::Vector3f eye_updir)
{
    Eigen::Matrix4f MV_matrix = Eigen::Matrix4f::Identity();

    // 先将相机移回原点
    Eigen::Matrix4f Translate;
    Translate << 1, 0, 0, -eye_pos(0),
        0, 1, 0, -eye_pos(1),
        0, 0, 1, -eye_pos(2),
        0, 0, 0, 1;
    // 再计算对应旋转矩阵
    Eigen::Matrix4f Rotation;
    // 相机的两个方向需要标准化
    eye_dir.normalize();
    eye_updir.normalize();
    // 相机朝向方向与相机上方方向叉乘
    Eigen::Vector3f crossproduct = eye_dir.cross(eye_updir);
    Rotation << crossproduct(0), crossproduct(1), crossproduct(2), 0,
        eye_updir(0), eye_updir(1), eye_updir(2), 0,
        -eye_dir(0), -eye_dir(1), -eye_dir(2), 0,
        0, 0, 0, 1;

    MV_matrix = Rotation * Translate;
    return MV_matrix;
}

// 得到投影变换矩阵
// eye_fov取水平FOV
// aspect_ratio是宽高比
// 包括了Pers->Ortho和Ortho两个变换
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // GAMES101课程中讲的zNear和zFar都是负数
    // 但是代入的值都是正数
    // 因此按课程内容进行运算时，就要乘-1
    //zNear *= -1;
    //zFar *= -1;

    // 透视矩阵Pers->Ortho
    Eigen::Matrix4f Persp;
    Persp << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // 通过fov和宽高比例计算出l, r, b, t
    // aspect_ratio就是宽高比
    float left, right, bottom, top;
    float angle = eye_fov / 180 * MY_PI;
    // 注意课程内容里是用长度，计算时要加 abs 算绝对值
    left = -tan(angle / 2) * abs(zNear);
    right = tan(angle / 2) * abs(zNear);
    bottom = left / aspect_ratio;
    top = right / aspect_ratio;

    // Ortho变换，即先平移再缩放
    // 位移矩阵
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(left + right) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    // 缩放矩阵
    Eigen::Matrix4f scale;
    scale << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    projection = scale * translate * Persp * projection;

    return projection;
}

// viewport变换，用于把透视后正则化2x2x2的立方体投到screenWidth x screenHeight的屏幕上
Eigen::Matrix4f get_viewport_matrix(int screenwidth, int screenheight)
{
    Eigen::Matrix4f Viewport;
    Viewport << screenwidth / 2, 0, 0, screenwidth / 2,
        0, screenheight / 2, 0, screenheight / 2,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return Viewport;
}

// 初始化三角形点
void Initialize()
{
    Projected_triangle_points = Initial_points;
    Now_points = Initial_points;
}

// ifinside函数，用于判断给定屏幕上点P是否在三角形ABC中
// 因为不需要考虑z坐标，因此P点用二维向量
bool ifinside(Eigen::Vector2f P, Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C)
{
    Eigen::Vector2f AB{ {B(0) - A(0),B(1) - A(1)} };
    Eigen::Vector2f BC{ {C(0) - B(0),C(1) - B(1)} };
    Eigen::Vector2f CA{ {A(0) - C(0),A(1) - C(1)} };

    Eigen::Vector2f AP{ {P(0) - A(0),P(1) - A(1)} };
    Eigen::Vector2f BP{ {P(0) - B(0),P(1) - B(1)} };
    Eigen::Vector2f CP{ {P(0) - C(0),P(1) - C(1)} };

    //// 不能用cross函数算二维向量的叉乘，会报错
    //Eigen::Vector2f cross_product1 = AP.cross(AB);
    //Eigen::Vector2f cross_product2 = BP.cross(BC);
    //Eigen::Vector2f cross_product3 = CP.cross(CA);
    // 直接用公式计算叉乘
    float cross_product1 = AP(0) * AB(1) - AP(1) * AB(0);
    float cross_product2 = BP(0) * BC(1) - BP(1) * BC(0);
    float cross_product3 = CP(0) * CA(1) - CP(1) * CA(0);

    // 三个方向全相同说明点就在三角形里
    if (cross_product1 * cross_product2 >= 0 && cross_product1 * cross_product3 >= 0)
    {
        return true;
    }

    return false;
}

// SFML逐像素操作真是闹麻了
// 正常光栅化图形
void Rasterizer_Normal(RenderWindow& window)
{
    // 创建一个新的Image
    Image Screen_image;
    Screen_image.create(screenWidth, screenHeight, Color::White);

    // 对应位置填充这个Image
    // 寻找三角形的bound矩形
    int xmin = (int)Projected_triangle_points[0](0);
    int ymin = (int)Projected_triangle_points[0](1);
    int xmax = xmin;
    int ymax = ymin;

    for (int i = 1; i < Projected_triangle_points.size(); i++)
    {
        int x = (int)Projected_triangle_points[i](0);
        int y = (int)Projected_triangle_points[i](1);

        if (x < xmin)
        {
            xmin = std::max(0, x);
        }
        else if (x > xmax)
        {
            xmax = std::min(screenWidth - 1, x);
        }

        if (y < ymin)
        {
            ymin = std::max(0, y);
        }
        else if (y > ymax)
        {
            ymax = std::min(screenHeight - 1, y);
        }
    }

    cout << "A: " << Projected_triangle_points[0](0) << "       " << Projected_triangle_points[0](1) << "       " << Projected_triangle_points[0](2) << endl;
    cout << "B: " << Projected_triangle_points[1](0) << "       " << Projected_triangle_points[1](1) << "       " << Projected_triangle_points[1](2) << endl;
    cout << "C: " << Projected_triangle_points[2](0) << "       " << Projected_triangle_points[2](1) << "       " << Projected_triangle_points[2](2) << endl;
    cout << endl << endl;

    if (ymin < 0) ymin = 0;
    if (ymin > screenHeight - 1) ymin = screenHeight - 1;
    if (xmin < 0) xmin = 0;
    if (xmin > screenWidth - 1) xmin = screenWidth - 1;

    for (int y = ymin; y < ymax; y++)
    {
        for (int x = xmin; x < xmax; x++)
        {
            if (x<0 || x>screenWidth || y<0 || y>screenHeight)
            {
                continue;
            }
            // Eigen库不能自动转换数据类型，因此统一使用float
            Eigen::Vector2f P{ {(float)x + 0.5f,(float)y + 0.5f} };
            if (ifinside(P, Projected_triangle_points[0], Projected_triangle_points[1], Projected_triangle_points[2]))
            {
                Screen_image.setPixel(x, y, Color::Red);
            }
        }
    }


    // 创建一个新的Texture
    Texture Screen_texture;
    Screen_texture.loadFromImage(Screen_image);
    // 创建一个新的Sprite并设置其纹理
    Sprite Screen_sprite;
    Screen_sprite.setTexture(Screen_texture);
    // 把这个Sprite画在屏幕上
    window.draw(Screen_sprite);
}

// 最需要注意的就是，旋转矩阵是对原始点乘的，并不是投影过来的点乘这个矩阵
// 这个问题基本上卡了一下午，笑死我了
// 现在把这个函数拆成两部分：计算绕XYZ轴最终的旋转矩阵，对所有点应用这个旋转矩阵
// 该函数仅保留对所有点应用这个旋转矩阵操作，计算旋转矩阵部分写成一个单独的函数：get_totalrotation_matrix
// 已经把最终旋转矩阵变成一个全局变量，因此该函数带入的参数全部删除
void Rotation_operation()
{
    // 对物体的每个点应用旋转矩阵
    for (int i = 0; i < Num_Points; i++)
    {
        // 转为带齐次坐标的坐标
        Eigen::Vector4f HomogeneousTmp{ {Now_points[i](0), Now_points[i](1), Now_points[i](2), 1} };
        // 应用旋转矩阵，Totalrotation_matirx是全局变量，记录最终旋转矩阵
        HomogeneousTmp = Totalrotation_matirx * HomogeneousTmp;
        // 记录变换后的坐标
        Now_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        Now_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        Now_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
    }
}


// sh是16x3的矩阵
// 返回的是一个3维列向量，所有元素限制在0和1之间
// 代入的pos是Now_points
Eigen::Vector3f computeColorFromSH(Eigen::Vector3f pos, Eigen::Vector3f campos, Eigen::MatrixXf sh, int deg)
{
    Eigen::Vector3f dir = pos - campos;
    dir.normalize();

    RowVector3f result = SH_C0 * sh.row(0);

    if (deg > 0)
    {
        double x = dir(0), y = dir(1), z = dir(2);
        result = result - SH_C1 * y * sh.row(1) + SH_C1 * z * sh.row(2) - SH_C1 * x * sh.row(3);

        if (deg > 1)
        {
            double xx = x * x, yy = y * y, zz = z * z, xy = x * y, yz = y * z, xz = x * z;
            result = result
                + SH_C2[0] * xy * sh.row(4)
                + SH_C2[1] * yz * sh.row(5)
                + SH_C2[2] * (2.0 * zz - xx - yy) * sh.row(6)
                + SH_C2[3] * xz * sh.row(7)
                + SH_C2[4] * (xx - yy) * sh.row(8);

            if (deg > 2)
            {
                result = result
                    + SH_C3[0] * y * (3.0 * xx - yy) * sh.row(9)
                    + SH_C3[1] * xy * z * sh.row(10)
                    + SH_C3[2] * y * (4.0 * zz - xx - yy) * sh.row(11)
                    + SH_C3[3] * z * (2.0 * zz - 3.0 * xx - 3.0 * yy) * sh.row(12)
                    + SH_C3[4] * x * (4.0 * zz - xx - yy) * sh.row(13)
                    + SH_C3[5] * z * (xx - yy) * sh.row(14)
                    + SH_C3[6] * x * (xx - 3.0 * yy) * sh.row(15);
            }
        }
    }
    result.array() += 0.5;

    // 将result数组中的所有元素限制在0和1之间
    return result.cwiseMax(0).cwiseMin(1);
}

// 在计算旋转矩阵的基础上，计算出绕X, Y, Z轴旋转的最终结果
// 基本上就是把原来实现的Rotation_operation函数里计算旋转矩阵的部分拆出来
// 因为要计算绕X, Y, Z轴旋转的最终结果，因此键盘鼠标操作逻辑也在此函数中（包括按R键Initiate初始化复原）
Eigen::Matrix4f get_totalrotation_matrix(RenderWindow& window, bool moveflag)
{
    float rotationAngle_X = 0;
    float rotationAngle_Y = 0;
    float rotationAngle_Z = 0;

    // 设置旋转轴
    Eigen::Vector3f rotationAxis_X{ { 1, 0, 0 } };
    Eigen::Vector3f rotationAxis_Y{ { 0, 1, 0 } };
    Eigen::Vector3f rotationAxis_Z{ { 0, 0, 1 } };


    sf::Vector2i mouseCurrentPosition = sf::Mouse::getPosition(window);
    // 计算鼠标的移动距离
    sf::Vector2i delta = mouseCurrentPosition - mouseLastPosition;
    if (moveflag)
    {
        // 如果鼠标左键被按下，那么旋转物体
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            // 计算旋转角度，假设鼠标的移动距离和旋转角度成正比
            rotationAngle_X = delta.y * 0.7f;
            rotationAngle_Y = -delta.x * 0.1f;
        }
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
    {
        rotationAngle_Y = 5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
    {
        rotationAngle_Y = -5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
    {
        rotationAngle_X = 5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
    {
        rotationAngle_X = -5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::E))
    {
        rotationAngle_Z = 5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
    {
        rotationAngle_Z = -5;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::R))
    {
        Initialize();
    }

    // 计算旋转矩阵
    // 绕X轴
    Eigen::Matrix4f rotationMatrix_X = get_rotation_arbitrary_axis(rotationAxis_X, rotationAngle_X, Geometry_center);
    // 绕Y轴
    Eigen::Matrix4f rotationMatrix_Y = get_rotation_arbitrary_axis(rotationAxis_Y, rotationAngle_Y, Geometry_center);
    // 绕Z轴
    Eigen::Matrix4f rotationMatrix_Z = get_rotation_arbitrary_axis(rotationAxis_Z, rotationAngle_Z, Geometry_center);

    // 返回最终的旋转矩阵
    return rotationMatrix_Z * rotationMatrix_Y * rotationMatrix_X;
}

// 对每个点进行投影操作，从真实坐标投影到屏幕上
// 并且将每个点的深度存入深度数组
// 将ModelView变换后的结果单独存入ModelView_points，下标对应
void Points_Projection()
{
    // viewport * Ortho * Pers->Ortho * ModelView * 原始坐标
    for (int i = 0; i < Num_Points; i++)
    {
        // 带齐次坐标的原始坐标
        Eigen::Vector4f HomogeneousTmp{ {Now_points[i](0), Now_points[i](1), Now_points[i](2), 1} };
        //// 这一行是一次进行MVP, Viewport变换，但是我现在需要把ModelView变换后的结果单独拆出来，因此一步一步来
        //HomogeneousTmp = get_viewport_matrix(screenWidth, screenHeight) * get_projection_matrix(45, 1, 0.1, 50) * get_modelview_matrix(eye_pos, eye_dir, eye_updir) * HomogeneousTmp;
        // 这一行进行MV变换
        HomogeneousTmp = get_modelview_matrix(eye_pos, eye_dir, eye_updir) * HomogeneousTmp;
        // MV变换存入数组
        ModelView_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        ModelView_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        ModelView_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
        // 再进行剩下的变换
        HomogeneousTmp = get_viewport_matrix(screenWidth, screenHeight) * get_projection_matrix(45, 1, 0.1, 50) * HomogeneousTmp;

        // 记录变换后的坐标
        Projected_triangle_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        Projected_triangle_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        Projected_triangle_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
        // 深度信息存入深度数组
        depth_list[i] = Projected_triangle_points[i](2);
    }
}

// 统一用Matrix3f
// covariance = RS[S^T][R^T]
// mod也是缩放，默认为1
// rotation就是带入最终旋转矩阵
Eigen::Matrix3f computeCov3D(Eigen::Vector3f scale, double mod, Eigen::Matrix4f rotation)
{
    // create scaling matrix
    Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
    S(0, 0) = scale(0) * mod;
    S(1, 1) = scale(1) * mod;
    S(2, 2) = scale(2) * mod;

    // normalize quaternion to get valid rotation
    // we use rotation matrix
    Eigen::Matrix3f R = rotation.block<3, 3>(0, 0);

    // compute 3d world covariance matrix Sigma
    Eigen::Matrix3f M = R * S;
    Eigen::Matrix3f cov3D = M * M.transpose();

    return cov3D;
}

// 雅克比矩阵相关计算，带入参数ModelView_point为经过ModelView变换的点
// 不过每次都需要把MV_matrix矩阵代入
// 已与源代码结果对比，两个compute计算结果完全一致
// 返回的按理来说一个是一个2x2的矩阵，但因为是对称的，所以只返回3个值即可
Eigen::Vector3f computeCov2D(Eigen::Vector3f ModelView_point, float focal_x, float focal_y, float tan_FOV, Eigen::Matrix3f cov3D, Eigen::Matrix4f MV_matrix)
{
    // 进行一定的限制
    float limX = 1.3 * tan_FOV;
    float limY = 1.3 * tan_FOV;
    float txtz = ModelView_point(0) / ModelView_point(2);
    float tytz = ModelView_point(1) / ModelView_point(2);
    ModelView_point(0) = std::min(limX, std::max(-limX, txtz)) * ModelView_point(2);
    ModelView_point(1) = std::min(limY, std::max(-limY, tytz)) * ModelView_point(2);

    // 雅可比矩阵
    Eigen::Matrix3f J;
    J << focal_x / ModelView_point[2], 0, -(focal_x * ModelView_point[0]) / (ModelView_point[2] * ModelView_point[2]),
        0, focal_y / ModelView_point[2], -(focal_y * ModelView_point[1]) / (ModelView_point[2] * ModelView_point[2]),
        0, 0, 0;
    // 剪切MV矩阵，只保留前三行前三列
    Eigen::Matrix3f W = MV_matrix.block<3, 3>(0, 0);
    Eigen::Matrix3f T = J * W;

    Eigen::Matrix3f cov = T * cov3D;
    cov = cov * T.transpose();

    // EWA splatting滤波
    // Apply low - pass filter
    // Every Gaussia should be at least one pixel wide / high
    // Discard 3rd row and column
    cov(0, 0) += 0.3;
    cov(1, 1) += 0.3;

    Eigen::Vector3f result{ cov(0,0),cov(0,1),cov(1,1) };
    return result;
}

// 预处理函数
// 代入的参数tan_half_FOV是FOV的一半的tan值
// 中间经过计算，更新cov3Ds, pointFlags, conic_opacity, radiuslist, RGB_3DGS
void Preprocess(float focal_x, float focal_y, float tan_half_FOV)
{
    Eigen::Matrix4f MV_matrix = get_modelview_matrix(eye_pos, eye_dir, eye_updir);

    for (int i = 0; i < Num_Points; i++)
    {
        // 默认点都进行处理，后续行列式如果为0就置0
        pointFlags[i] = 1;

        // compute 3d covarance by scaling and rotation parameters
        Eigen::Vector3f scale{ scales.row(i)(0), scales.row(i)(1), scales.row(i)(2) };
        Eigen::Matrix3f cov3D = computeCov3D(scale, 1, Totalrotation_matirx);
        cov3Ds[i] = cov3D;

        // compute 2D screen-space covariance matrix
        // based on splatting, -> JW Sigma W^ T J^ T
        Eigen::Vector3f cov = computeCov2D(ModelView_points[i], focal_x, focal_y, tan_half_FOV, cov3D, MV_matrix);

        // invert covarance(EWA splatting)
        // 计算行列式
        float determinant = cov(0) * cov(2) - cov(1) * cov(1);
        // 如果行列式等于0，则跳过这个点，处理下一个点
        // 跳过点的过程：用一个flag数组存储，是否对点进行操作，0为不操作
        if (determinant == 0)
        {
            pointFlags[i] = 0;
            continue;
        }
        float determinant_inv = 1 / determinant;
        Eigen::Vector3f conic{ cov[2] * determinant_inv, -cov[1] * determinant_inv, cov[0] * determinant_inv };
        conic_opacity[i](0) = conic(0);
        conic_opacity[i](1) = conic(1);
        conic_opacity[i](2) = conic(2);
        conic_opacity[i](3) = 1; // 此处应该是opacity数组的第i个，这个数组是n行1列，全是1，因此这里简写为1，实现功能后再修改

        // compute radius, by finding eigenvalues of 2d covariance
        // transfrom point from NDC to Pixel
        float mid = 0.5 * (cov(0) + cov(1));
        // std::max的两个参数要匹配，因此把0.1写成0.1f
        float lambda1 = mid + (float)sqrt(std::max(0.1f, mid * mid - determinant));
        float lambda2 = mid - (float)sqrt(std::max(0.1f, mid * mid - determinant));
        float my_radius = (float)std::ceil(3 * (float)sqrt(std::max(lambda1, lambda2)));
        radiuslist[i] = my_radius;

        // convert spherical harmonics coefficients to RGB color
        Eigen::MatrixXf sh = shs[i];
        Eigen::Vector3f result = computeColorFromSH(Now_points[i], eye_pos, sh, 3);

        // result算出来是0-1的小数，这里乘255
        // 注意不能用(int)result(0)，这样只能得到0
        RGB_3DGS[i] = sf::Color((int)(255 * result(0)), (int)(255 * (int)result(1)), (int)(255 * (int)result(2)));

        RGB_3DGS_Tmp[i](0) = result(0) * 255;
        RGB_3DGS_Tmp[i](1) = result(1) * 255;
        RGB_3DGS_Tmp[i](2) = result(2) * 255;
    }
}

// 定义比较函数
// 按深度升序排序，因为看向-Z方向，越远的深度越小，越近的深度越大
// 渲染的时候先渲染远处的，再渲染近处的，能把远处的遮挡住
bool compareDepth(const DepthIndex& a, const DepthIndex& b)
{
    return a.depth < b.depth;
}

// 3DGS渲染结果画到屏幕上
// 失败了失败了失败了失败了失败了失败了失败了失败了，为什么只有一个高斯球？
// 破案了，是前面颜色设置的时候用的255*(int)result(0)，直接设置为0了
// 算是实现功能了
void Rasterizer_3DGS(RenderWindow& window)
{
    // 创建一个新的Image
    Image Screen_image;
    Screen_image.create(screenWidth, screenHeight, Color::White);

    // 此部分操作屏幕每一点像素

    //再调用预处理函数
    float FOVangle = eye_FOV / 180 * MY_PI;

    // 先计算focal
    //float focal_y = screenHeight / (2 * tan(FOVangle / 2));
    //float focal_x = screenWidth / (2 * tan(FOVangle / 2));
    float focal_y = screenHeight / (1 * 1);
    float focal_x = screenWidth / (1 * 1);

    Preprocess(focal_x, focal_y, tan(FOVangle / 2));

    // 创建结构体列表，存储深度和下标
    vector<DepthIndex> depth_index_list(Num_Points);
    for (int i = 0; i < Num_Points; i++)
    {
        depth_index_list[i].depth = depth_list[i];
        depth_index_list[i].index = i;
    }
    // 按深度升序排序，因为越远的深度越小，越近的深度越大
    // 故最近的会被最后渲染
    sort(depth_index_list.begin(), depth_index_list.end(), compareDepth);



    // 遍历整个图
    for (int y = 0; y < screenHeight; y++)
    {
        for (int x = 0; x < screenWidth; x++)
        {
            // 这里改成x,y
            Eigen::Vector2i pixf{ x,y };
            sf::Color background_color = sf::Color::Black;
            sf::Color C = sf::Color::Black;

            Eigen::Vector3f background_color_Tmp{ 0,0,0 };
            Eigen::Vector3f C_Tmp{ 0,0,0 };

            float T = 1;
            // 不考虑深度排序
            //for (int index = 0; index < Num_Points; index++)
            // 考虑深度排序
            for (int i = 0; i < Num_Points; i++)
            {
                // 不行，完全失败
                // 太奇怪了，用这个index反而转的都有问题
                // 我感觉是只画了一个高斯球，如果考虑了深度排序，会在高斯球深度数组改变时画另一个球
                // 按D，直接画左边那个球，这就证明了我的猜想 ↑
                int index = depth_index_list[i].index;

                if (pointFlags[index] == 0)
                {
                    continue;
                }

                // init helper variables, transmirrance
                T = 1;

                // 高斯混合：首先计算了每个点的高斯混合权重，这个权重是通过计算像素点到高斯中心的距离，然后代入高斯函数得到的
                // 这个权重反映了像素点的颜色应该受到这个高斯球颜色的多大影响
                // Resample using conic matrix
                // (cf. "Surface Splatting" by Zwicker et al., 2001)
                // center of 2d gaussian
                Eigen::Vector3f gaussian_center = Projected_triangle_points[index];

                // distance from center of pixel
                Eigen::Vector2f distance;
                distance << gaussian_center(0) - (float)pixf(0),
                    gaussian_center(1) - (float)pixf(1);

                Eigen::Vector4f con_o = conic_opacity[index];

                float power = (-0.5 * (con_o[0] * distance[0] * distance[0] + con_o[2] * distance[1] * distance[1]) - con_o[1] * distance[0] * distance[1]);
                if (power > 0)
                {
                    continue;
                }

                // Eq. (2) from 3D Gaussian splatting paper.
                // Compute color
                float alpha = std::min(0.99f, con_o(3) * std::exp(power));

                // 这里的两个if判断只是阈值判断，注释掉不影响最终的结果
                //if (alpha < 1 / 255.0f)
                //{
                //    continue;
                //}

                float test_T = T * (1 - alpha);
                if (test_T < 0.0001f)
                {
                    break;
                }

                // Eq. (3) from 3D Gaussian splatting paper.
                sf::Color color = RGB_3DGS[index];

                Eigen::Vector3f color_Tmp = RGB_3DGS_Tmp[index];

                C.r += color.r * alpha * T;
                C.g += color.g * alpha * T;
                C.b += color.b * alpha * T;

                C_Tmp(0) += color_Tmp(0) * alpha * T;
                C_Tmp(1) += color_Tmp(1) * alpha * T;
                C_Tmp(2) += color_Tmp(2) * alpha * T;

                T = test_T;
            }
            sf::Color final_color;

            // 着重分析一下颜色
            // 这样在background_color和C设置成黑色的情况下能显示为红色
            // 我决定还是用这种
            final_color.r = (C.r + T * background_color.r);
            final_color.g = (C.g + T * background_color.g);
            final_color.b = (C.b + T * background_color.b);

            // 这样在background_color_Tmp和C_Tmp设置成{0,0,0}的情况下能显示为灰色
            //final_color.r = (int)(C_Tmp(0) + T * background_color_Tmp(0));
            //final_color.g = (int)(C_Tmp(1) + T * background_color_Tmp(1));
            //final_color.b = (int)(C_Tmp(2) + T * background_color_Tmp(2));


            Screen_image.setPixel(x, y, final_color);
        }
    }

    // 创建一个新的Texture
    Texture Screen_texture;
    Screen_texture.loadFromImage(Screen_image);
    // 创建一个新的Sprite并设置其纹理
    Sprite Screen_sprite;
    Screen_sprite.setTexture(Screen_texture);
    // 把这个Sprite画在屏幕上
    window.draw(Screen_sprite);
}



