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
/*                  ������ʼ������                                                                        */
/*********************************************************************************************************/
// �����λ��
Eigen::Vector3f eye_pos = { 0, 1, 5 };
// ���������
Eigen::Vector3f eye_dir = { 0, 0, -1 };
// ������Ϸ�����
Eigen::Vector3f eye_updir = { 0, 1, 0 };
// FOV
float eye_FOV = 45;

// ��ʼ���λ�ã�Ҳ���ڳ�ʼ��
vector<Eigen::Vector3f> Initial_points{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };
// ��¼���ڵ��λ�ã����ڴ�����ת����
vector<Eigen::Vector3f> Now_points = Initial_points;
// ModelView�任��ĵ��λ�ã�����computeCov2D��������������ظ�����
vector<Eigen::Vector3f> ModelView_points = Initial_points;
// MVP��ViewportͶӰ����λ��
vector<Eigen::Vector3f> Projected_triangle_points = Initial_points;
// ����λ�ã�������ת����ĵ㣬�����ܱ���ͼ������Ļ������ת
Eigen::Vector3f Geometry_center = { 0, 1, -2 };

// ������XYZ�����ת����
// ����������ÿ֡����
Eigen::Matrix4f Totalrotation_matirx = Matrix4f::Identity();

/*********************************************************************************************************/
/*                  ������㲿��                                                                          */
/*********************************************************************************************************/

// �õ� ����������ת����Ƕ� ��ת���������޵����˹��ת��ʽ
// ��������axis_direction���򻯣���˷����������ֱ��������������������
// ������Ķ��巽������������������������һ���ǵ�λ������ָ�����򼴿ɣ�
// �����Ĭ��Ϊԭ��(0, 0, 0)������ʵ���������ʱ��Ӧ��Ҫ�������ĵ�Geometry_center
// ����ķ�����㶼����ά�ģ�����Ҫ�����������
Eigen::Matrix4f get_rotation_arbitrary_axis(Eigen::Vector3f axis_direction, float rotation_angle, Eigen::Vector3f point = Eigen::Vector3f::Zero());

// �õ�MV�任�������ڽ��������ȫ�������ƶ�
// �ƶ�ԭ������̶���ԭ��(0, 0, 0)���泯-Z�������Ϸ�ΪY����
Eigen::Matrix4f get_modelview_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f eye_dir, Eigen::Vector3f eye_updir);

// �õ�ͶӰ�任����
// eye_fovȡˮƽFOV
// aspect_ratio�ǿ�߱�
// ������Pers->Ortho��Ortho�����任
Eigen::Matrix4f get_projection_matrix(float eye_fov = 45, float aspect_ratio = 1, float zNear = 0.1, float zFar = 50);

// viewport�任�����ڰ�͸�Ӻ�����2x2x2��������Ͷ��screenWidth x screenHeight����Ļ��
Eigen::Matrix4f get_viewport_matrix(int screenwidth = screenWidth, int screenheight = screenHeight);

// �ڼ�����ת����Ļ����ϣ��������X, Y, Z����ת�����ս��
// �����Ͼ��ǰ�ԭ��ʵ�ֵ�Rotation_operation�����������ת����Ĳ��ֲ����
// ��ΪҪ������X, Y, Z����ת�����ս������˼����������߼�Ҳ�ڴ˺����У�������R��Initiate��ʼ����ԭ��
Eigen::Matrix4f get_totalrotation_matrix(RenderWindow& window, bool moveflag = 0);

/*********************************************************************************************************/
/*                  ������դ������                                                                        */
/*********************************************************************************************************/
// ��ʼ�������ε�
void Initialize();

// ifinside�����������жϸ�����Ļ�ϵ�P�Ƿ���������ABC��
// ��Ϊ����Ҫ����z���꣬���P���ö�ά����
bool ifinside(Eigen::Vector2f P, Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C);

// ���ڴ洢���
// ��Points_Projection�����д������
vector<float> depth_list(Num_Points);

// ��ÿ�������ͶӰ����������ʵ����ͶӰ����Ļ��
// ���ҽ�ÿ�������ȴ����������
// ��ModelView�任��Ľ����������ModelView_points���±��Ӧ
void Points_Projection();

// SFML�����ز�������������
// ������դ��ͼ��
void Rasterizer_Normal(RenderWindow& window);

/*********************************************************************************************************/
/*                  ͼ����ת����                                                                          */
/*********************************************************************************************************/
// ��¼�����һ�ε�λ�ã���������ÿһ֡������¼��ǰ���λ��
sf::Vector2i mouseLastPosition;
// ����ƶ�flag�������������¼��������и��£�����������ÿ֡����ʱ��0
bool Mousemoveflag = 0;

// ����Ҫע��ľ��ǣ���ת�����Ƕ�ԭʼ��˵ģ�������ͶӰ�����ĵ���������
// �����������Ͽ���һ���磬Ц������
// ���ڰ����������������֣�������XYZ�����յ���ת���󣬶����е�Ӧ�������ת����
// �ú��������������е�Ӧ�������ת���������������ת���󲿷�д��һ�������ĺ�����get_totalrotation_matrix
// �Ѿ���������ת������һ��ȫ�ֱ�������˸ú�������Ĳ���ȫ��ɾ��
void Rotation_operation();

/*********************************************************************************************************/
/*                  ��г��������                                                                          */
/*********************************************************************************************************/
// ���峣��
// ȫ��Ϊfloat��������double����Ϊeigen�ⲻ������ת��
const double SH_C0 = 0.28209479177387814;
const double SH_C1 = 0.4886025119029199;
VectorXf SH_C2{ {1.0925484305920792, -1.0925484305920792, 0.31539156525252005, -1.0925484305920792, 0.5462742152960396} };
VectorXf SH_C3{ {-0.5900435899266435, 2.890611442640554, -0.4570457994644658, 0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435} };
// sh��16x3�ľ���
// ���ص���һ��3ά������������Ԫ��������0��1֮��
// �����pos��Now_points
Eigen::Vector3f computeColorFromSH(Eigen::Vector3f pos, Eigen::Vector3f campos, Eigen::MatrixXf sh, int deg = 3);

/*********************************************************************************************************/
/*                  3DGS����                                                                             */
/*********************************************************************************************************/

// ͳһ��Matrix3f
// covariance = RS[S^T][R^T]
// modҲ�����ţ�Ĭ��Ϊ1
// rotation���Ǵ���������ת����
Eigen::Matrix3f computeCov3D(Eigen::Vector3f scale, double mod, Eigen::Matrix4f rotation = Matrix4f::Identity());


// �ſ˱Ⱦ�����ؼ��㣬�������ModelView_pointΪ����ModelView�任�ĵ�
// ����ÿ�ζ���Ҫ��MV_matrix�������
// ����Դ�������Աȣ�����compute��������ȫһ��
// ���صİ�����˵һ����һ��2x2�ľ��󣬵���Ϊ�ǶԳƵģ�����ֻ����3��ֵ����
Eigen::Vector3f computeCov2D(Eigen::Vector3f ModelView_point, float focal_x, float focal_y, float tan_FOV, Eigen::Matrix3f cov3D, Eigen::Matrix4f MV_matrix);

// �Ƿ���õ㣬Ĭ�϶�����
// ��Ҫ�����������Ϊ{ 1,1,1 }
bool pointFlags[Num_Points] = { 1,1,1 };

// scales: Num_Pointsx3��ȫ1����
Eigen::MatrixXf scales = Eigen::MatrixXf::Ones(Num_Points, 3);
// cov3Ds
vector<Eigen::Matrix3f> cov3Ds(Num_Points);
// conic_opacity
vector<Eigen::Vector4f> conic_opacity(Num_Points);
// radius
vector<float> radiuslist(Num_Points);
// ��˹�����ɫ����sf::Color�洢
// �������������Ϊ���Աȣ������Color������Ϊ��ɫ�������Vector���㣬���������ת��Ϊint�����Ϊ��ɫ�������
vector<sf::Color> RGB_3DGS(Num_Points);
vector<Eigen::Vector3f> RGB_3DGS_Tmp(Num_Points);

// shs��ÿһ������16x3�ľ���
// Ϊ�˷�������������Ϊ�̶�ֵ
// sh�����ֵ�ᵼ�����Ͻ���ʧ�����½�����
//vector<Eigen::MatrixXf> shs(Num_Points, Eigen::MatrixXf::Constant(16, 3, 0.5f));
vector<Eigen::MatrixXf> shs(Num_Points, Eigen::MatrixXf::Constant(16, 3, 0.1f));

// Ԥ������
// ����Ĳ���tan_half_FOV��FOV��һ���tanֵ
// �м侭�����㣬����cov3Ds, pointFlags, conic_opacity, radiuslist, RGB_3DGS
void Preprocess(float focal_x, float focal_y, float tan_half_FOV);

// ����ṹ�壬�洢�����Ϣ���±�
struct DepthIndex {
    float depth;
    int index;
};

// ����ȽϺ���
// ���������������Ϊ����-Z����ԽԶ�����ԽС��Խ�������Խ��
// ��Ⱦ��ʱ������ȾԶ���ģ�����Ⱦ�����ģ��ܰ�Զ�����ڵ�ס
bool compareDepth(const DepthIndex& a, const DepthIndex& b);

// 3DGS��Ⱦ���������Ļ��
// ʧ����ʧ����ʧ����ʧ����ʧ����ʧ����ʧ����ʧ���ˣ�Ϊʲôֻ��һ����˹��
// �ư��ˣ���ǰ����ɫ���õ�ʱ���õ�255*(int)result(0)��ֱ������Ϊ0��
// ����ʵ�ֹ�����
void Rasterizer_3DGS(RenderWindow& window);


/*********************************************************************************************************/
/*                  ����������                                                                            */
/*********************************************************************************************************/
int main()
{
    sf::RenderWindow window_normal(sf::VideoMode(screenWidth, screenHeight), "Rasterizer");
    sf::RenderWindow window_3DGS(sf::VideoMode(screenWidth, screenHeight), "3DGS");

    // ��ȡ��Ļ�ķֱ���
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int FullscreenWidth = desktop.width;
    int FullscreenHeight = desktop.height;

    // ���㴰�ڵ���λ�ã����в������������໥������
    sf::Vector2i position1(FullscreenWidth / 2 - screenWidth, FullscreenHeight / 2 - screenHeight / 2);
    sf::Vector2i position2(FullscreenWidth / 2 - screenWidth + screenWidth + 3, FullscreenHeight / 2 - screenHeight / 2);

    // ���ô��ڵ�λ��
    window_normal.setPosition(position1);
    window_3DGS.setPosition(position2);


    while (window_normal.isOpen() && window_3DGS.isOpen())
    {
        sf::Event event;
        while (window_normal.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window_normal.close();
            // ����ƶ��¼�
            if (event.type == sf::Event::MouseMoved)
            {
                Mousemoveflag = 1;
            }
        }

        window_normal.clear(Color::White);
        window_3DGS.clear(Color::White);

        // MVP, ViewportͶӰ
        Points_Projection();
        // ������դ��
        Rasterizer_Normal(window_normal);
        // ��Ⱦ3DGS
        Rasterizer_3DGS(window_3DGS);

        // �������յ���ת����
        Totalrotation_matirx = get_totalrotation_matrix(window_normal, Mousemoveflag);
        // ��ȫ���������ת����
        Rotation_operation();

        window_normal.display();
        window_3DGS.display();

        // ��¼��һ֡����λ�ã�������һ֡��LastPosition
        mouseLastPosition = sf::Mouse::getPosition(window_normal);
        // ���flag��0
        Mousemoveflag = 0;
    }


    return 0;
}

/*********************************************************************************************************/
/*                  ����ʵ�ֲ���                                                                          */
/*********************************************************************************************************/
// �õ� ����������ת����Ƕ� ��ת���������޵����˹��ת��ʽ
// ��������axis_direction���򻯣���˷����������ֱ��������������������
// ������Ķ��巽������������������������һ���ǵ�λ������ָ�����򼴿ɣ�
// �����Ĭ��Ϊԭ��(0, 0, 0)������ʵ���������ʱ��Ӧ��Ҫ�������ĵ�
// ����ķ�����㶼����ά�ģ�����Ҫ�����������
Eigen::Matrix4f get_rotation_arbitrary_axis(Eigen::Vector3f axis_direction, float rotation_angle, Eigen::Vector3f point)
{
    // �Ƚ�����������ĵ��ƻ�ԭ��
    Eigen::Matrix4f Translate;
    Translate << 1, 0, 0, -point(0),
        0, 1, 0, -point(1),
        0, 0, 1, -point(2),
        0, 0, 0, 1;
    // ����ת��Ӧ�Ƕȣ������޵����˹��ת��ʽ
    // ������������
    axis_direction.normalize();
    // ����Ƕ�
    float theta = rotation_angle / 180 * MY_PI;
    // ����cos��sin
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    // ���ݹ�ʽ������ת���󣬹�ʽ��Դ: GAMES101_Lecture_04 P10
    // Rodrigues�� Rotation Formula: Rotation by angle theta around axis n
    // R(n, theta) = cos(theta) * I + (1 - cos(theta)) * n * n.T + sin(theta) * N
    // N = { 0, -nz, ny,
    //      nz, 0, -nx,
    //      -ny, nx, 0 }
    // ע�⣺��������������3x3�ģ����Ҫ�����������4x4����ת����
    Matrix3f N;
    N << 0, -axis_direction(2), axis_direction(1),
        axis_direction(2), 0, -axis_direction(0),
        -axis_direction(1), axis_direction(0), 0;
    Matrix3f Rotation_3x3;
    Matrix3f Eye = Matrix3f::Identity();
    Rotation_3x3 = cos_theta * Eye + (1 - cos_theta) * axis_direction * axis_direction.transpose() + sin_theta * N;
    // ��3x3��ת������չΪ4x4��ת����
    Matrix4f Rotation_4x4 = Matrix4f::Identity();
    Rotation_4x4.block<3, 3>(0, 0) = Rotation_3x3; // ��3x3��ת����Ƕ�뵽���Ͻ�
    // �������������ĵ��ƻ�ȥ
    Eigen::Matrix4f Translate_back;
    Translate_back << 1, 0, 0, point(0),
        0, 1, 0, point(1),
        0, 0, 1, point(2),
        0, 0, 0, 1;
    // �������յ�4x4����
    Eigen::Matrix4f result = Translate_back * Rotation_4x4 * Translate;
    return result;
}

// �õ�MV�任�������ڽ��������ȫ�������ƶ�
// �ƶ�ԭ������̶���ԭ��(0, 0, 0)���泯-Z�������Ϸ�ΪY����
Eigen::Matrix4f get_modelview_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f eye_dir, Eigen::Vector3f eye_updir)
{
    Eigen::Matrix4f MV_matrix = Eigen::Matrix4f::Identity();

    // �Ƚ�����ƻ�ԭ��
    Eigen::Matrix4f Translate;
    Translate << 1, 0, 0, -eye_pos(0),
        0, 1, 0, -eye_pos(1),
        0, 0, 1, -eye_pos(2),
        0, 0, 0, 1;
    // �ټ����Ӧ��ת����
    Eigen::Matrix4f Rotation;
    // ���������������Ҫ��׼��
    eye_dir.normalize();
    eye_updir.normalize();
    // ���������������Ϸ�������
    Eigen::Vector3f crossproduct = eye_dir.cross(eye_updir);
    Rotation << crossproduct(0), crossproduct(1), crossproduct(2), 0,
        eye_updir(0), eye_updir(1), eye_updir(2), 0,
        -eye_dir(0), -eye_dir(1), -eye_dir(2), 0,
        0, 0, 0, 1;

    MV_matrix = Rotation * Translate;
    return MV_matrix;
}

// �õ�ͶӰ�任����
// eye_fovȡˮƽFOV
// aspect_ratio�ǿ�߱�
// ������Pers->Ortho��Ortho�����任
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // GAMES101�γ��н���zNear��zFar���Ǹ���
    // ���Ǵ����ֵ��������
    // ��˰��γ����ݽ�������ʱ����Ҫ��-1
    //zNear *= -1;
    //zFar *= -1;

    // ͸�Ӿ���Pers->Ortho
    Eigen::Matrix4f Persp;
    Persp << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // ͨ��fov�Ϳ�߱��������l, r, b, t
    // aspect_ratio���ǿ�߱�
    float left, right, bottom, top;
    float angle = eye_fov / 180 * MY_PI;
    // ע��γ����������ó��ȣ�����ʱҪ�� abs �����ֵ
    left = -tan(angle / 2) * abs(zNear);
    right = tan(angle / 2) * abs(zNear);
    bottom = left / aspect_ratio;
    top = right / aspect_ratio;

    // Ortho�任������ƽ��������
    // λ�ƾ���
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(left + right) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    // ���ž���
    Eigen::Matrix4f scale;
    scale << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    projection = scale * translate * Persp * projection;

    return projection;
}

// viewport�任�����ڰ�͸�Ӻ�����2x2x2��������Ͷ��screenWidth x screenHeight����Ļ��
Eigen::Matrix4f get_viewport_matrix(int screenwidth, int screenheight)
{
    Eigen::Matrix4f Viewport;
    Viewport << screenwidth / 2, 0, 0, screenwidth / 2,
        0, screenheight / 2, 0, screenheight / 2,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return Viewport;
}

// ��ʼ�������ε�
void Initialize()
{
    Projected_triangle_points = Initial_points;
    Now_points = Initial_points;
}

// ifinside�����������жϸ�����Ļ�ϵ�P�Ƿ���������ABC��
// ��Ϊ����Ҫ����z���꣬���P���ö�ά����
bool ifinside(Eigen::Vector2f P, Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C)
{
    Eigen::Vector2f AB{ {B(0) - A(0),B(1) - A(1)} };
    Eigen::Vector2f BC{ {C(0) - B(0),C(1) - B(1)} };
    Eigen::Vector2f CA{ {A(0) - C(0),A(1) - C(1)} };

    Eigen::Vector2f AP{ {P(0) - A(0),P(1) - A(1)} };
    Eigen::Vector2f BP{ {P(0) - B(0),P(1) - B(1)} };
    Eigen::Vector2f CP{ {P(0) - C(0),P(1) - C(1)} };

    //// ������cross�������ά�����Ĳ�ˣ��ᱨ��
    //Eigen::Vector2f cross_product1 = AP.cross(AB);
    //Eigen::Vector2f cross_product2 = BP.cross(BC);
    //Eigen::Vector2f cross_product3 = CP.cross(CA);
    // ֱ���ù�ʽ������
    float cross_product1 = AP(0) * AB(1) - AP(1) * AB(0);
    float cross_product2 = BP(0) * BC(1) - BP(1) * BC(0);
    float cross_product3 = CP(0) * CA(1) - CP(1) * CA(0);

    // ��������ȫ��ͬ˵���������������
    if (cross_product1 * cross_product2 >= 0 && cross_product1 * cross_product3 >= 0)
    {
        return true;
    }

    return false;
}

// SFML�����ز�������������
// ������դ��ͼ��
void Rasterizer_Normal(RenderWindow& window)
{
    // ����һ���µ�Image
    Image Screen_image;
    Screen_image.create(screenWidth, screenHeight, Color::White);

    // ��Ӧλ��������Image
    // Ѱ�������ε�bound����
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
            // Eigen�ⲻ���Զ�ת���������ͣ����ͳһʹ��float
            Eigen::Vector2f P{ {(float)x + 0.5f,(float)y + 0.5f} };
            if (ifinside(P, Projected_triangle_points[0], Projected_triangle_points[1], Projected_triangle_points[2]))
            {
                Screen_image.setPixel(x, y, Color::Red);
            }
        }
    }


    // ����һ���µ�Texture
    Texture Screen_texture;
    Screen_texture.loadFromImage(Screen_image);
    // ����һ���µ�Sprite������������
    Sprite Screen_sprite;
    Screen_sprite.setTexture(Screen_texture);
    // �����Sprite������Ļ��
    window.draw(Screen_sprite);
}

// ����Ҫע��ľ��ǣ���ת�����Ƕ�ԭʼ��˵ģ�������ͶӰ�����ĵ���������
// �����������Ͽ���һ���磬Ц������
// ���ڰ����������������֣�������XYZ�����յ���ת���󣬶����е�Ӧ�������ת����
// �ú��������������е�Ӧ�������ת���������������ת���󲿷�д��һ�������ĺ�����get_totalrotation_matrix
// �Ѿ���������ת������һ��ȫ�ֱ�������˸ú�������Ĳ���ȫ��ɾ��
void Rotation_operation()
{
    // �������ÿ����Ӧ����ת����
    for (int i = 0; i < Num_Points; i++)
    {
        // תΪ��������������
        Eigen::Vector4f HomogeneousTmp{ {Now_points[i](0), Now_points[i](1), Now_points[i](2), 1} };
        // Ӧ����ת����Totalrotation_matirx��ȫ�ֱ�������¼������ת����
        HomogeneousTmp = Totalrotation_matirx * HomogeneousTmp;
        // ��¼�任�������
        Now_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        Now_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        Now_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
    }
}


// sh��16x3�ľ���
// ���ص���һ��3ά������������Ԫ��������0��1֮��
// �����pos��Now_points
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

    // ��result�����е�����Ԫ��������0��1֮��
    return result.cwiseMax(0).cwiseMin(1);
}

// �ڼ�����ת����Ļ����ϣ��������X, Y, Z����ת�����ս��
// �����Ͼ��ǰ�ԭ��ʵ�ֵ�Rotation_operation�����������ת����Ĳ��ֲ����
// ��ΪҪ������X, Y, Z����ת�����ս������˼����������߼�Ҳ�ڴ˺����У�������R��Initiate��ʼ����ԭ��
Eigen::Matrix4f get_totalrotation_matrix(RenderWindow& window, bool moveflag)
{
    float rotationAngle_X = 0;
    float rotationAngle_Y = 0;
    float rotationAngle_Z = 0;

    // ������ת��
    Eigen::Vector3f rotationAxis_X{ { 1, 0, 0 } };
    Eigen::Vector3f rotationAxis_Y{ { 0, 1, 0 } };
    Eigen::Vector3f rotationAxis_Z{ { 0, 0, 1 } };


    sf::Vector2i mouseCurrentPosition = sf::Mouse::getPosition(window);
    // ���������ƶ�����
    sf::Vector2i delta = mouseCurrentPosition - mouseLastPosition;
    if (moveflag)
    {
        // ��������������£���ô��ת����
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            // ������ת�Ƕȣ����������ƶ��������ת�Ƕȳ�����
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

    // ������ת����
    // ��X��
    Eigen::Matrix4f rotationMatrix_X = get_rotation_arbitrary_axis(rotationAxis_X, rotationAngle_X, Geometry_center);
    // ��Y��
    Eigen::Matrix4f rotationMatrix_Y = get_rotation_arbitrary_axis(rotationAxis_Y, rotationAngle_Y, Geometry_center);
    // ��Z��
    Eigen::Matrix4f rotationMatrix_Z = get_rotation_arbitrary_axis(rotationAxis_Z, rotationAngle_Z, Geometry_center);

    // �������յ���ת����
    return rotationMatrix_Z * rotationMatrix_Y * rotationMatrix_X;
}

// ��ÿ�������ͶӰ����������ʵ����ͶӰ����Ļ��
// ���ҽ�ÿ�������ȴ����������
// ��ModelView�任��Ľ����������ModelView_points���±��Ӧ
void Points_Projection()
{
    // viewport * Ortho * Pers->Ortho * ModelView * ԭʼ����
    for (int i = 0; i < Num_Points; i++)
    {
        // ����������ԭʼ����
        Eigen::Vector4f HomogeneousTmp{ {Now_points[i](0), Now_points[i](1), Now_points[i](2), 1} };
        //// ��һ����һ�ν���MVP, Viewport�任��������������Ҫ��ModelView�任��Ľ����������������һ��һ����
        //HomogeneousTmp = get_viewport_matrix(screenWidth, screenHeight) * get_projection_matrix(45, 1, 0.1, 50) * get_modelview_matrix(eye_pos, eye_dir, eye_updir) * HomogeneousTmp;
        // ��һ�н���MV�任
        HomogeneousTmp = get_modelview_matrix(eye_pos, eye_dir, eye_updir) * HomogeneousTmp;
        // MV�任��������
        ModelView_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        ModelView_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        ModelView_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
        // �ٽ���ʣ�µı任
        HomogeneousTmp = get_viewport_matrix(screenWidth, screenHeight) * get_projection_matrix(45, 1, 0.1, 50) * HomogeneousTmp;

        // ��¼�任�������
        Projected_triangle_points[i](0) = HomogeneousTmp(0) / HomogeneousTmp(3);
        Projected_triangle_points[i](1) = HomogeneousTmp(1) / HomogeneousTmp(3);
        Projected_triangle_points[i](2) = HomogeneousTmp(2) / HomogeneousTmp(3);
        // �����Ϣ�����������
        depth_list[i] = Projected_triangle_points[i](2);
    }
}

// ͳһ��Matrix3f
// covariance = RS[S^T][R^T]
// modҲ�����ţ�Ĭ��Ϊ1
// rotation���Ǵ���������ת����
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

// �ſ˱Ⱦ�����ؼ��㣬�������ModelView_pointΪ����ModelView�任�ĵ�
// ����ÿ�ζ���Ҫ��MV_matrix�������
// ����Դ�������Աȣ�����compute��������ȫһ��
// ���صİ�����˵һ����һ��2x2�ľ��󣬵���Ϊ�ǶԳƵģ�����ֻ����3��ֵ����
Eigen::Vector3f computeCov2D(Eigen::Vector3f ModelView_point, float focal_x, float focal_y, float tan_FOV, Eigen::Matrix3f cov3D, Eigen::Matrix4f MV_matrix)
{
    // ����һ��������
    float limX = 1.3 * tan_FOV;
    float limY = 1.3 * tan_FOV;
    float txtz = ModelView_point(0) / ModelView_point(2);
    float tytz = ModelView_point(1) / ModelView_point(2);
    ModelView_point(0) = std::min(limX, std::max(-limX, txtz)) * ModelView_point(2);
    ModelView_point(1) = std::min(limY, std::max(-limY, tytz)) * ModelView_point(2);

    // �ſɱȾ���
    Eigen::Matrix3f J;
    J << focal_x / ModelView_point[2], 0, -(focal_x * ModelView_point[0]) / (ModelView_point[2] * ModelView_point[2]),
        0, focal_y / ModelView_point[2], -(focal_y * ModelView_point[1]) / (ModelView_point[2] * ModelView_point[2]),
        0, 0, 0;
    // ����MV����ֻ����ǰ����ǰ����
    Eigen::Matrix3f W = MV_matrix.block<3, 3>(0, 0);
    Eigen::Matrix3f T = J * W;

    Eigen::Matrix3f cov = T * cov3D;
    cov = cov * T.transpose();

    // EWA splatting�˲�
    // Apply low - pass filter
    // Every Gaussia should be at least one pixel wide / high
    // Discard 3rd row and column
    cov(0, 0) += 0.3;
    cov(1, 1) += 0.3;

    Eigen::Vector3f result{ cov(0,0),cov(0,1),cov(1,1) };
    return result;
}

// Ԥ������
// ����Ĳ���tan_half_FOV��FOV��һ���tanֵ
// �м侭�����㣬����cov3Ds, pointFlags, conic_opacity, radiuslist, RGB_3DGS
void Preprocess(float focal_x, float focal_y, float tan_half_FOV)
{
    Eigen::Matrix4f MV_matrix = get_modelview_matrix(eye_pos, eye_dir, eye_updir);

    for (int i = 0; i < Num_Points; i++)
    {
        // Ĭ�ϵ㶼���д�����������ʽ���Ϊ0����0
        pointFlags[i] = 1;

        // compute 3d covarance by scaling and rotation parameters
        Eigen::Vector3f scale{ scales.row(i)(0), scales.row(i)(1), scales.row(i)(2) };
        Eigen::Matrix3f cov3D = computeCov3D(scale, 1, Totalrotation_matirx);
        cov3Ds[i] = cov3D;

        // compute 2D screen-space covariance matrix
        // based on splatting, -> JW Sigma W^ T J^ T
        Eigen::Vector3f cov = computeCov2D(ModelView_points[i], focal_x, focal_y, tan_half_FOV, cov3D, MV_matrix);

        // invert covarance(EWA splatting)
        // ��������ʽ
        float determinant = cov(0) * cov(2) - cov(1) * cov(1);
        // �������ʽ����0������������㣬������һ����
        // ������Ĺ��̣���һ��flag����洢���Ƿ�Ե���в�����0Ϊ������
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
        conic_opacity[i](3) = 1; // �˴�Ӧ����opacity����ĵ�i�������������n��1�У�ȫ��1����������дΪ1��ʵ�ֹ��ܺ����޸�

        // compute radius, by finding eigenvalues of 2d covariance
        // transfrom point from NDC to Pixel
        float mid = 0.5 * (cov(0) + cov(1));
        // std::max����������Ҫƥ�䣬��˰�0.1д��0.1f
        float lambda1 = mid + (float)sqrt(std::max(0.1f, mid * mid - determinant));
        float lambda2 = mid - (float)sqrt(std::max(0.1f, mid * mid - determinant));
        float my_radius = (float)std::ceil(3 * (float)sqrt(std::max(lambda1, lambda2)));
        radiuslist[i] = my_radius;

        // convert spherical harmonics coefficients to RGB color
        Eigen::MatrixXf sh = shs[i];
        Eigen::Vector3f result = computeColorFromSH(Now_points[i], eye_pos, sh, 3);

        // result�������0-1��С���������255
        // ע�ⲻ����(int)result(0)������ֻ�ܵõ�0
        RGB_3DGS[i] = sf::Color((int)(255 * result(0)), (int)(255 * (int)result(1)), (int)(255 * (int)result(2)));

        RGB_3DGS_Tmp[i](0) = result(0) * 255;
        RGB_3DGS_Tmp[i](1) = result(1) * 255;
        RGB_3DGS_Tmp[i](2) = result(2) * 255;
    }
}

// ����ȽϺ���
// ���������������Ϊ����-Z����ԽԶ�����ԽС��Խ�������Խ��
// ��Ⱦ��ʱ������ȾԶ���ģ�����Ⱦ�����ģ��ܰ�Զ�����ڵ�ס
bool compareDepth(const DepthIndex& a, const DepthIndex& b)
{
    return a.depth < b.depth;
}

// 3DGS��Ⱦ���������Ļ��
// ʧ����ʧ����ʧ����ʧ����ʧ����ʧ����ʧ����ʧ���ˣ�Ϊʲôֻ��һ����˹��
// �ư��ˣ���ǰ����ɫ���õ�ʱ���õ�255*(int)result(0)��ֱ������Ϊ0��
// ����ʵ�ֹ�����
void Rasterizer_3DGS(RenderWindow& window)
{
    // ����һ���µ�Image
    Image Screen_image;
    Screen_image.create(screenWidth, screenHeight, Color::White);

    // �˲��ֲ�����Ļÿһ������

    //�ٵ���Ԥ������
    float FOVangle = eye_FOV / 180 * MY_PI;

    // �ȼ���focal
    //float focal_y = screenHeight / (2 * tan(FOVangle / 2));
    //float focal_x = screenWidth / (2 * tan(FOVangle / 2));
    float focal_y = screenHeight / (1 * 1);
    float focal_x = screenWidth / (1 * 1);

    Preprocess(focal_x, focal_y, tan(FOVangle / 2));

    // �����ṹ���б��洢��Ⱥ��±�
    vector<DepthIndex> depth_index_list(Num_Points);
    for (int i = 0; i < Num_Points; i++)
    {
        depth_index_list[i].depth = depth_list[i];
        depth_index_list[i].index = i;
    }
    // ���������������ΪԽԶ�����ԽС��Խ�������Խ��
    // ������Ļᱻ�����Ⱦ
    sort(depth_index_list.begin(), depth_index_list.end(), compareDepth);



    // ��������ͼ
    for (int y = 0; y < screenHeight; y++)
    {
        for (int x = 0; x < screenWidth; x++)
        {
            // ����ĳ�x,y
            Eigen::Vector2i pixf{ x,y };
            sf::Color background_color = sf::Color::Black;
            sf::Color C = sf::Color::Black;

            Eigen::Vector3f background_color_Tmp{ 0,0,0 };
            Eigen::Vector3f C_Tmp{ 0,0,0 };

            float T = 1;
            // �������������
            //for (int index = 0; index < Num_Points; index++)
            // �����������
            for (int i = 0; i < Num_Points; i++)
            {
                // ���У���ȫʧ��
                // ̫����ˣ������index����ת�Ķ�������
                // �Ҹо���ֻ����һ����˹�����������������򣬻��ڸ�˹���������ı�ʱ����һ����
                // ��D��ֱ�ӻ�����Ǹ������֤�����ҵĲ��� ��
                int index = depth_index_list[i].index;

                if (pointFlags[index] == 0)
                {
                    continue;
                }

                // init helper variables, transmirrance
                T = 1;

                // ��˹��ϣ����ȼ�����ÿ����ĸ�˹���Ȩ�أ����Ȩ����ͨ���������ص㵽��˹���ĵľ��룬Ȼ������˹�����õ���
                // ���Ȩ�ط�ӳ�����ص����ɫӦ���ܵ������˹����ɫ�Ķ��Ӱ��
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

                // ���������if�ж�ֻ����ֵ�жϣ�ע�͵���Ӱ�����յĽ��
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

            // ���ط���һ����ɫ
            // ������background_color��C���óɺ�ɫ�����������ʾΪ��ɫ
            // �Ҿ�������������
            final_color.r = (C.r + T * background_color.r);
            final_color.g = (C.g + T * background_color.g);
            final_color.b = (C.b + T * background_color.b);

            // ������background_color_Tmp��C_Tmp���ó�{0,0,0}�����������ʾΪ��ɫ
            //final_color.r = (int)(C_Tmp(0) + T * background_color_Tmp(0));
            //final_color.g = (int)(C_Tmp(1) + T * background_color_Tmp(1));
            //final_color.b = (int)(C_Tmp(2) + T * background_color_Tmp(2));


            Screen_image.setPixel(x, y, final_color);
        }
    }

    // ����һ���µ�Texture
    Texture Screen_texture;
    Screen_texture.loadFromImage(Screen_image);
    // ����һ���µ�Sprite������������
    Sprite Screen_sprite;
    Screen_sprite.setTexture(Screen_texture);
    // �����Sprite������Ļ��
    window.draw(Screen_sprite);
}



