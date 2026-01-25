# Niko的GAMES101作业记录

## 01 旋转和投影

### Model变换

- **绕z轴进行旋转**

```c++
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
```

- **绕过原点的任意轴旋转，使用罗德里格斯旋转公式**

```c++
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
```

### View变换

```c++
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    // 相当于将相机和物体同步位移，相机移动到原点上，但是只处理了平移
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}
```

### Projetion变换

```c++
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
```

### 结果

为了确保没有轴被错误翻转，选择了其他点来保证图形不是轴对称的

 <img src=".\MDImages\as1.png" alt="as1" style="zoom:50%;" />

## 02 光栅化和Z-buffer

### Z-buffer

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // Find out the bounding box of current triangle.

	float minX = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
	float maxX = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
	float minY = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
	float maxY = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    // iterate through the pixel and find if the current pixel is inside the triangle


    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha / v[0].w() * v[0].z()  + beta / v[1].w() * v[1].z()  + gamma / v[2].w() * v[2].z();
                z_interpolated *= w_reciprocal;
                
                // 这是经过透视校正的z-buffer，算出了未经过透视变换的w值，从结果上来看更像w-buffer(保存并比较透视变换前的深度信息)
                // 如果不经过校正，直接计算x,y点对应的z插值（透视变换之后的），就是z-buffer

                if (z_interpolated < depth_buf[get_index(x, y)]) {
                    depth_buf[get_index(x, y)] = z_interpolated;
                    // set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                }
            }
        }
	}
}
```

### 2xSSAA

```c++
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    //--snip--
    if (Enable2xSSAA) {
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                Vector3f final_color(0, 0, 0);
                for (int i = 0; i < 4; ++i) {
                    if (depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)] < std::numeric_limits<float>::infinity()) {
                        // If the subpixel is covered, accumulate its color
                        final_color += frame_buf_2xSSAA[get_index_2xSSAA(x, y, i)] / 4.0f;
                    }
                }
                set_pixel(Eigen::Vector3f(x, y, 1), final_color);
            }
        }
    }
}
```

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // Find out the bounding box of current triangle.

    float minX = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    float maxX = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    float minY = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    float maxY = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            if (Enable2xSSAA) {
                float subpixel_offsets[4][2] = {
                    {0.25f, 0.25f},
                    {0.75f, 0.25f},
                    {0.25f, 0.75f},
                    {0.75f, 0.75f}
                };
                for (int i = 0; i < 4; ++i) {
                    float sub_x = x + subpixel_offsets[i][0];
                    float sub_y = y + subpixel_offsets[i][1];
                    if (insideTriangle(sub_x, sub_y, t.v)) {
            
                        auto [alpha, beta, gamma] = computeBarycentric2D(sub_x, sub_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha / v[0].w() * v[0].z() + beta / v[1].w() * v[1].z() + gamma / v[2].w() * v[2].z();
                        z_interpolated *= w_reciprocal;

                        if (z_interpolated < depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)]) {
                            depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)] = z_interpolated;
							frame_buf_2xSSAA[get_index_2xSSAA(x, y, i)] = t.getColor();
                        }
                    }
                }
                
            }
            else {
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha / v[0].w() * v[0].z() + beta / v[1].w() * v[1].z() + gamma / v[2].w() * v[2].z();
                    z_interpolated *= w_reciprocal;

                    if (z_interpolated < depth_buf[get_index(x, y)]) {
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                    }
                }
            }
            
        }
    }
}
```

### 结果

 <img src=".\MDImages\as2_1.png" alt="as2_1" style="zoom: 33%;" />

<img src=".\MDImages\as2_3.jpeg" alt="as2_3" style="zoom: 80%;" /> <img src=".\MDImages\as2_4.jpeg" alt="as2_4" style="zoom:80%;" />

右图蓝色三角形上侧的白边锯齿改进明显

## 03 着色器
