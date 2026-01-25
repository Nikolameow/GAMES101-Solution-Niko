#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	float rad_angle = rotation_angle / 180.0 * MY_PI;

    model << cos(rad_angle), -sin(rad_angle), 0, 0,
             sin(rad_angle),  cos(rad_angle), 0, 0,
             0,               0,              1, 0,
		     0,               0,              0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float n = -zNear, f = -zFar;
    // 透视投影->正交投影  挤压
    Eigen::Matrix4f Mpersp_orhtho;
    Mpersp_orhtho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    // 正交投影->正则立方体
    float fovY = eye_fov / 180 * MY_PI;// 角度转弧度
    float t = tan(fovY / 2) * zNear, b = -t;// 
    float r = aspect_ratio * t, l = -r;
    // 转换到正则立方体
    Eigen::Matrix4f Mortho, Mtrans, Mscale;
    Mtrans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Mscale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
		0, 0, 2 / (f - n), 0, // 注意符号，是far - near（负值），实现反向映射。更远的点映射达到1，更近的点映射到-1
        0, 0, 0, 1;
    Mortho = Mscale * Mtrans;
    // 计算得到投影矩阵
    projection = Mortho * Mpersp_orhtho;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float rad_angle = angle / 180.0 * MY_PI;
    float c = cos(rad_angle);
    float s = sin(rad_angle);
    rotation << c + axis.x() * axis.x() * (1 - c), axis.x() * axis.y() * (1 - c) - axis.z() * s, axis.x() * axis.z() * (1 - c) + axis.y() * s, 0,
                axis.y() * axis.x() * (1 - c) + axis.z() * s, c + axis.y() * axis.y() * (1 - c), axis.y() * axis.z() * (1 - c) - axis.x() * s, 0,
                axis.z() * axis.x() * (1 - c) - axis.y() * s, axis.z() * axis.y() * (1 - c) + axis.x() * s, c + axis.z() * axis.z() * (1 - c), 0,
                0, 0, 0, 1;
    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-1, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
