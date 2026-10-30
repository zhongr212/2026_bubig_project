#ifndef Q_MATH_H
#define Q_MATH_H

float Q_sqrt(float number);


typedef struct {
    float data[4][4];  // 4x4浮点数矩阵，用于表示变换矩阵
} Matrix4x4;

// 定义4D向量结构（用于齐次坐标）
typedef struct {
    float x, y, z, w;  // x,y,z: 3D坐标，w: 齐次坐标分量（通常为1）
} Vector4;

// 定义2D向量结构（用于屏幕坐标）
typedef struct {
    float x, y;        // x,y: 2D坐标
} Vector2;


#endif
