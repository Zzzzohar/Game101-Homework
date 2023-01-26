// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();//rast头文件中下一个id
    pos_buf.emplace(id, positions);//在id指向的元素前创建一个值为positions的元素

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //通过点叉积获取方向  然后判断三个方向是否同向
    Eigen::Vector3f P = { x,y,0 };

    Eigen::Vector3f PA = _v[0] - P;
    Eigen::Vector3f PB = _v[1] - P;
    Eigen::Vector3f PC = _v[2] - P; 
    //在一个平面判断
    PA[2] = 1;
    PB[2] = 1;
    PC[2] = 1; 

    //叉积获取向量
    Eigen::Vector3f PAB = PA.cross(PB);
    Eigen::Vector3f PBC = PB.cross(PC);
    Eigen::Vector3f PCA = PC.cross(PA);

    //点积判断是否都同向
    float dot1 = PAB.dot(PBC);
    float dot2 = PBC.dot(PCA);
    float dot3 = PCA.dot(PAB);

    if (dot1 > 0 && dot2 > 0 && dot3 > 0) {
        return true;

    }
    else {
        return false;
    }
    
    
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int BB_L = std::min(v[0][0],std::min(v[1][0],v[2][0]));
    int BB_R = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    int BB_B = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    int BB_T = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    // If so, use the following code to get the interpolated z value.  插值示例 
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    std::vector<float >sample_offset = { 0.25,0.25,0.75,0.75,0.25 };
   


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    for (int x = BB_L; x <= BB_R; x++) {
        for (int y = BB_B; y <= BB_T; y++) {
            float cover = 0;
            float dep = INT_MAX;
            for (int i = 0; i < 4; i++) {
                if (insideTriangle(x + sample_offset[i], y + sample_offset[i + 1], t.v)) { //次像素判断在不在三角形内部

                     //深度插值
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + sample_offset[i], y + sample_offset[i + 1], t.v); //计算每个次像素
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    
                    //当前次像素的深度 和缓冲区的深度判断
                    if (-z_interpolated < depth_sample[get_index(x, y) * 4 + i]) { //次像素判断  然后更新缓冲区
                        depth_sample[get_index(x, y) * 4 + i] = -z_interpolated;  
                        frame_sample[get_index(x, y) * 4 + i] = t.getColor() / 4; //提前权重计算
                    }
                    dep = std::min(dep, depth_sample[get_index(x, y) * 4 + i]); //判断次像素最小深度
                }  
            }
            //最后设置frame depth buf
            set_pixel({ (float)x,(float)y,1 }, frame_sample[get_index(x, y) * 4]+ frame_sample[get_index(x, y) * 4+1]+ frame_sample[get_index(x, y) * 4+2]+ frame_sample[get_index(x, y) * 4+3]);
            depth_buf[get_index(x, y)] = std::min(depth_buf[get_index(x, y)], dep);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //初始化 每个像素下有4个次像素
    depth_sample.resize(w * h * 4);
    frame_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on